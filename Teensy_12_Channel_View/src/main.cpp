// ============================================================================
// Teensy 4.1 - 12-Channel Logic Analyzer (edge capture, cycle timestamps)
// ============================================================================
//
// Samples up to 12 digital pins as fast as possible (typically 5-8 MS/s at
// 600 MHz, i.e. ~120-200 cycles per sample) and records only state CHANGES
// together with CPU-cycle timestamps (1.67 ns resolution). This is ideal for
// narrow pulses (e.g. 1 us pulses at 100 Hz): the pulse is sampled several
// times and the exact edge times are preserved, while the USB link only
// carries the (tiny) list of transitions.
//
//   - The 12 pins are fully re-configurable from the host:  PINS <12 pins>
//   - Three output drivers (OE / EN / RST) are assignable to any pin and can
//     be pulsed HIGH from the host:  SETPIN / PULSE / PULSEW
//   - Capture runs in windows with interrupts disabled, so edge timing is
//     deterministic.  Between windows the ring buffer is streamed to USB.
//   - Gaps (time not sampled) are reported explicitly so the viewer can
//     show "unknown" regions instead of guessing.
//
// Wire protocol (newline-terminated text over USB Serial):
//
//   Host -> Teensy:
//     PINS <p0> <p1> ... <p11>   configure the 12 channels (pins 0..41)
//     START                      begin capture (time base resets to 0)
//     STOP                       stop capture
//     WIN <ms>                   capture-window length (10..5000 ms)
//     SETPIN <OE|EN|RST> <pin>   assign (or -1 to release) an output pin
//     PULSE  <OE|EN|RST>         pulse that output HIGH for PULSEW ms
//     PULSEW <ms>                output pulse width (1..3000 ms)
//     INFO                       re-send header / pin / output / status lines
//
//   Teensy -> Host:
//     #FW <name> <version>       firmware identity
//     #FCPU <hz>                 actual CPU frequency
//     #NCH <n>                   number of channels (always 12)
//     #PINS <p0> ... <p11>       current channel->pin mapping
//     #OUT <OE|EN|RST> <pin>     output pin assignment (-1 = none)
//     #PULSEW <ms>               output pulse width
//     #PULSE <name>              output pulse acknowledgement
//     #WIN <ms>                  current window length
//     #READY                     configuration acknowledged
//     #START / #STOP             capture state acknowledgements
//     #RATE <samples>            samples taken in the last window
//     #OVF <n>                   ring-buffer events dropped (data too fast)
//     #ERR <text>                command rejected
//     S <t> <hex>                set state: at cycle t the 12-bit state is hex
//     E <t> <hex>                edge: state changed to hex at cycle t
//     G <t0> <t1>                gap: no sampling between cycles t0 and t1
//
// Timestamps are CPU cycles relative to the last START (wraps every ~7 s at
// 600 MHz; the viewer unwraps them).
// ============================================================================

#include <Arduino.h>
#include <core_pins.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define FW_VERSION     "1.0.0"
#define MAX_CHANNELS   12
#define MAX_PIN        41            // Teensy 4.1 usable GPIO pins are 0..41

// ---------------------------------------------------------------------------
// Event ring buffer (8 bytes per event)
// ---------------------------------------------------------------------------
#define RB_SIZE        32768         // 256 KB - plenty for edge-only data
#define RB_MASK        (RB_SIZE - 1)
#define EV_EDGE        0x80000000UL  // event flag: 0 = S (set state), 1 = E (edge)

struct Event {
    uint32_t t;                      // cycle count (relative to capture start)
    uint32_t s;                      // 12-bit state (+ EV_EDGE flag for edges)
};

static Event rb[RB_SIZE];
static volatile uint32_t rbHead = 0; // written by capture loop (IRQs disabled)
static uint32_t rbTail = 0;          // read by main loop

// ---------------------------------------------------------------------------
// Channel configuration.  Default pins can be edited here; the viewer can
// also reconfigure them at runtime with the PINS command.
// ---------------------------------------------------------------------------
static uint8_t chPins[MAX_CHANNELS] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static uint8_t chCount = MAX_CHANNELS;

// Per channel: pointer to the GPIO pad-status register (PSR) and bit mask.
// Reading PSR gives the actual pin level for input pins.
struct Chan {
    volatile uint32_t *psr;
    uint32_t mask;
};
static Chan chans[MAX_CHANNELS];

// ---------------------------------------------------------------------------
// Capture state
// ---------------------------------------------------------------------------
static volatile bool capturing = false;
static uint32_t tBase = 0;           // absolute cycle count at last START
static uint32_t gapStartAbs = 0;     // absolute cycle when last window ended
static uint32_t winCycles = 0;       // window length in cycles
static uint32_t windowSamples = 0;   // samples taken in the last window
static uint32_t overflow = 0;        // events dropped because the ring filled

// ---------------------------------------------------------------------------
// Output drivers (OE / EN / RST).  Each is an idle-LOW output pin that can be
// pulsed HIGH for a configurable width when the host sends "PULSE <name>".
// Pin 255 means "not assigned".  Pulse timing uses the cycle counter so it
// keeps running even while capture windows have interrupts disabled.
// ---------------------------------------------------------------------------
#define NUM_OUTS       3
#define PIN_NONE       255
static const char *outNames[NUM_OUTS] = {"OE", "EN", "RST"};
static uint8_t  outPins[NUM_OUTS] = {PIN_NONE, PIN_NONE, PIN_NONE};
static uint32_t outPulseStartCyc[NUM_OUTS] = {0, 0, 0};  // pulse start (0 = idle)
static uint32_t pulseWidthCyc = 0;   // pulse length in cycles (set in setup())

// ---------------------------------------------------------------------------
// Read the 12-bit state of all channels in one shot.  The loop bound is a
// compile-time constant so the compiler can fully unroll it.
// ---------------------------------------------------------------------------
static inline uint32_t readState(void) {
    uint32_t s = 0;
#pragma GCC unroll 12
    for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
        if (*chans[i].psr & chans[i].mask) s |= (1u << i);
    }
    return s;
}

static inline uint32_t rbUsed(void) {
    return rbHead - rbTail;          // free-running counters, safe modulo 2^32
}

static inline bool rbHasSpace(uint32_t n) {
    return (RB_SIZE - 1 - rbUsed()) >= n;
}

// ---------------------------------------------------------------------------
// Run one capture window with interrupts disabled.  All edges are recorded
// with cycle-accurate timestamps into the ring buffer.
// ---------------------------------------------------------------------------
static uint32_t windowStartAbs = 0;  // absolute cycle at window start

static void runWindow(void) {
    __disable_irq();                 // deterministic timing: no preemption

    uint32_t t0 = ARM_DWT_CYCCNT;    // window start (absolute cycles)
    windowStartAbs = t0;
    uint32_t last = readState();
    uint32_t samples = 1;

    // Always emit the state at window start (re-establishes the baseline,
    // also across gaps).
    if (rbHasSpace(2)) {
        rb[rbHead & RB_MASK].t = t0 - tBase;
        rb[rbHead & RB_MASK].s = last;   // S event
        rbHead++;
    } else {
        overflow++;
    }

    while (1) {
        uint32_t s = readState();
        samples++;

        if (s != last) {
            last = s;
            if (rbHasSpace(2)) {
                rb[rbHead & RB_MASK].t = ARM_DWT_CYCCNT - tBase;
                rb[rbHead & RB_MASK].s = s | EV_EDGE;  // E event
                rbHead++;
            } else {
                overflow++;
            }
        }

        // Check window end / buffer pressure every 64 samples to keep the
        // hot loop lean.
        if (!(samples & 0x3F)) {
            uint32_t now = ARM_DWT_CYCCNT;
            if ((now - t0) >= winCycles) break;
            if (!rbHasSpace(256)) break;   // ring nearly full -> drain soon
        }
    }

    gapStartAbs = ARM_DWT_CYCCNT;    // end of sampling (start of the gap)
    windowSamples = samples;

    __enable_irq();
}

// ---------------------------------------------------------------------------
// Stream the ring buffer to USB.
// ---------------------------------------------------------------------------
static void drain(void) {
    while (rbTail != rbHead) {
        if (Serial.availableForWrite() < 48) break;   // never block here
        Event e = rb[rbTail & RB_MASK];
        rbTail++;
        char buf[40];
        int n = snprintf(buf, sizeof(buf), "%c %lu %lX\n",
                         (e.s & EV_EDGE) ? 'E' : 'S',
                         (unsigned long)e.t,
                         (unsigned long)(e.s & 0x0FFF));
        Serial.write(buf, n);
    }
}

// ---------------------------------------------------------------------------
// Header / status lines
// ---------------------------------------------------------------------------
static void sendHeader(void) {
    Serial.print("#FW Teensy12ChannelLA ");
    Serial.println(FW_VERSION);
    Serial.print("#FCPU ");
    Serial.println(F_CPU_ACTUAL);
    Serial.print("#NCH ");
    Serial.println(chCount);
    Serial.print("#WIN ");
    Serial.println(winCycles / (F_CPU_ACTUAL / 1000UL));
}

static void sendPins(void) {
    Serial.print("#PINS");
    for (uint8_t i = 0; i < chCount; i++) {
        Serial.print(' ');
        Serial.print(chPins[i]);
    }
    Serial.println();
}

// ---------------------------------------------------------------------------
// Output drivers (OE / EN / RST)
// ---------------------------------------------------------------------------
static void sendOuts(void) {
    for (uint8_t i = 0; i < NUM_OUTS; i++) {
        Serial.print("#OUT ");
        Serial.print(outNames[i]);
        Serial.print(' ');
        Serial.println(outPins[i] == PIN_NONE ? -1 : (int)outPins[i]);
    }
    Serial.print("#PULSEW ");
    Serial.println(pulseWidthCyc / (F_CPU_ACTUAL / 1000UL));
}

static int8_t outIndexByName(const char *name) {
    for (uint8_t i = 0; i < NUM_OUTS; i++) {
        if (strcmp(name, outNames[i]) == 0) return (int8_t)i;
    }
    return -1;
}

/* Assign (or release, pin < 0) the pin used by an output driver. */
static bool configureOut(const char *name, long pin) {
    int8_t i = outIndexByName(name);
    if (i < 0) return false;
    if (pin != -1 && (pin < 0 || pin > MAX_PIN)) return false;

    if (outPins[i] != PIN_NONE) {          // release the old pin
        pinMode(outPins[i], INPUT);
        outPins[i] = PIN_NONE;
    }
    outPulseStartCyc[i] = 0;               // cancel any pending pulse
    if (pin >= 0) {
        outPins[i] = (uint8_t)pin;
        pinMode(outPins[i], OUTPUT);
        digitalWrite(outPins[i], LOW);     // idle LOW
    }
    return true;
}

/* Pulse the output HIGH for the configured width, then back LOW. */
static bool pulseOut(const char *name) {
    int8_t i = outIndexByName(name);
    if (i < 0 || outPins[i] == PIN_NONE) return false;
    digitalWrite(outPins[i], HIGH);
    outPulseStartCyc[i] = ARM_DWT_CYCCNT;
    return true;
}

/* Called from the main loop: bring pulsed outputs back LOW once the
 * configured width has elapsed.  Uses signed cycle deltas so the 32-bit
 * cycle-counter wrap (every ~7 s) is handled correctly. */
static void updateOutPulses(void) {
    for (uint8_t i = 0; i < NUM_OUTS; i++) {
        if (outPulseStartCyc[i] == 0) continue;
        if ((int32_t)(ARM_DWT_CYCCNT - outPulseStartCyc[i]) >= (int32_t)pulseWidthCyc) {
            digitalWrite(outPins[i], LOW);
            outPulseStartCyc[i] = 0;
        }
    }
}

// ---------------------------------------------------------------------------
// Configure the channel set.  Returns false on an invalid pin list.
// ---------------------------------------------------------------------------
static bool configurePins(const uint8_t pins[MAX_CHANNELS]) {
    for (uint8_t i = 0; i < chCount; i++) {
        if (pins[i] > MAX_PIN) return false;
        for (uint8_t j = 0; j < i; j++) {
            if (pins[i] == pins[j]) return false;   // duplicate pin
        }
    }
    capturing = false;               // reconfigure only while stopped
    for (uint8_t i = 0; i < chCount; i++) {
        chPins[i] = pins[i];
        pinMode(chPins[i], INPUT);
        chans[i].psr = portInputRegister(chPins[i]);   // -> PSR (pad status)
        chans[i].mask = digitalPinToBitMask(chPins[i]);
    }
    return true;
}

// ---------------------------------------------------------------------------
// Command dispatch
// ---------------------------------------------------------------------------
static void dispatch(const char *line) {
    if (strncmp(line, "PINS", 4) == 0) {
        const char *p = line + 4;
        uint8_t pins[MAX_CHANNELS];
        bool ok = true;
        for (uint8_t i = 0; i < chCount; i++) {
            while (*p == ' ') p++;
            char *end;
            long v = strtol(p, &end, 10);
            if (end == p || v < 0 || v > MAX_PIN) { ok = false; break; }
            pins[i] = (uint8_t)v;
            p = end;
        }
        if (ok && configurePins(pins)) {
            sendPins();
            Serial.println("#READY");
        } else {
            Serial.println("#ERR invalid pin list (need 12 distinct pins 0..41)");
        }
    } else if (strncmp(line, "START", 5) == 0) {
        if (!capturing) {
            tBase = ARM_DWT_CYCCNT;  // time base resets to zero
            gapStartAbs = 0;
            rbHead = rbTail = 0;     // safe: capture is stopped here
            overflow = 0;
            capturing = true;
            Serial.println("#START");
        }
    } else if (strncmp(line, "STOP", 4) == 0) {
        if (capturing) {
            capturing = false;
            drain();                 // flush whatever was captured
            Serial.println("#STOP");
        }
    } else if (strncmp(line, "WIN", 3) == 0) {
        long ms = strtol(line + 3, NULL, 10);
        if (ms < 10) ms = 10;
        if (ms > 5000) ms = 5000;
        winCycles = (uint32_t)ms * (F_CPU_ACTUAL / 1000UL);
        Serial.print("#WIN ");
        Serial.println(ms);
    } else if (strncmp(line, "INFO", 4) == 0) {
        sendHeader();
        sendPins();
        sendOuts();
        Serial.println(capturing ? "#START" : "#READY");
    } else if (strncmp(line, "SETPIN", 6) == 0) {
        // SETPIN <OE|EN|RST> <pin|-1>
        char name[8];
        const char *p = line + 6;
        while (*p == ' ') p++;
        int n = 0;
        while (*p && *p != ' ' && n < 7) name[n++] = *p++;
        name[n] = 0;
        long pin = strtol(p, NULL, 10);
        if (configureOut(name, pin)) {
            sendOuts();
        } else {
            Serial.println("#ERR invalid output (use: SETPIN OE|EN|RST <pin|-1>)");
        }
    } else if (strncmp(line, "PULSE", 5) == 0) {
        // PULSE <OE|EN|RST>
        char name[8];
        const char *p = line + 5;
        while (*p == ' ') p++;
        int n = 0;
        while (*p && *p != ' ' && n < 7) name[n++] = *p++;
        name[n] = 0;
        if (pulseOut(name)) {
            Serial.print("#PULSE ");
            Serial.println(name);
        } else {
            Serial.println("#ERR no pin assigned to that output");
        }
    } else if (strncmp(line, "PULSEW", 6) == 0) {
        long ms = strtol(line + 6, NULL, 10);
        if (ms < 1) ms = 1;
        if (ms > 3000) ms = 3000;        // keep within the signed cycle range
        pulseWidthCyc = (uint32_t)ms * (F_CPU_ACTUAL / 1000UL);
        Serial.print("#PULSEW ");
        Serial.println(ms);
    } else {
        Serial.println("#ERR unknown command");
    }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);            // baud is ignored for USB CDC

    // The startup code already enables the DWT cycle counter; make sure.
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

    winCycles = 100UL * (F_CPU_ACTUAL / 1000UL);   // 100 ms default window
    pulseWidthCyc = 100UL * (F_CPU_ACTUAL / 1000UL); // 100 ms default pulse

    uint8_t pins[MAX_CHANNELS];
    memcpy(pins, chPins, sizeof(pins));
    configurePins(pins);

    delay(500);                      // let USB enumerate
    sendHeader();
    sendPins();
    sendOuts();
    Serial.println("#READY");
}

void loop() {
    // 1. Commands from the host
    static char cmd[96];
    static uint8_t len = 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (len) {
                cmd[len] = 0;
                dispatch(cmd);
                len = 0;
            }
        } else if (len < sizeof(cmd) - 1) {
            cmd[len++] = c;
        }
    }

    // 2. Stream captured data
    drain();

    // 2b. Bring pulsed outputs back LOW when their width has elapsed
    updateOutPulses();

    // 3. Keep capturing in windows
    if (capturing) {
        uint32_t prevEnd = gapStartAbs;      // end of the previous window (0 = first)
        runWindow();                         // windowStartAbs = start, gapStartAbs = end
        // Report the unsampled time between the previous window's end and this
        // window's start, BEFORE the ring is drained, so the stream reads:
        // ... E, G, S, E ...  (the S re-establishes the state after the gap).
        if (prevEnd && windowStartAbs != prevEnd) {
            Serial.print("G ");
            Serial.print(prevEnd - tBase);
            Serial.print(' ');
            Serial.println(windowStartAbs - tBase);
        }
        drain();
        Serial.print("#RATE ");
        Serial.println(windowSamples);
        if (overflow) {
            Serial.print("#OVF ");
            Serial.println(overflow);
            overflow = 0;
        }
    }
}
