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
//   - An ADXL355/ADXL359 3-axis accelerometer on SPI can stream alongside the
//     digital capture (ACC / ACCODR commands).  The sensor's 32-sample FIFO is
//     drained continuously: between capture windows AND a few times inside
//     each window (the brief SPI pauses are reported as gaps so the viewer
//     shades them).  Samples carry cycle timestamps relative to START, so the
//     viewer plots them on the same time base as the digital signals.
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
//     ACC ON|OFF                 enable/disable accelerometer streaming
//     ACCODR <hz>                accelerometer ODR: 500|1000|2000|4000
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
//     #ACC <0|1>                 accelerometer present+enabled / absent
//     #ACCODR <hz>               accelerometer output data rate
//     #ACCRANGE <g>              accelerometer full-scale range (default +-10g)
//     #ACCOVF <n>                staged accel samples dropped (host too slow)
//     #ACCERR <text>             accelerometer init / communication failure
//     #READY                     configuration acknowledged
//     #START / #STOP             capture state acknowledgements
//     #RATE <samples>            samples taken in the last window
//     #OVF <n>                   ring-buffer events dropped (data too fast)
//     #ERR <text>                command rejected
//     S <t> <hex>                set state: at cycle t the 12-bit state is hex
//     E <t> <hex>                edge: state changed to hex at cycle t
//     G <t0> <t1>                gap: no sampling between cycles t0 and t1
//     A <t> <x> <y> <z>          accel sample at cycle t; x/y/z in mg (int)
//
// Timestamps are CPU cycles relative to the last START (wraps every ~7 s at
// 600 MHz; the viewer unwraps them).
// ============================================================================

#include <Arduino.h>
#include <core_pins.h>
#include <SPI.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define FW_VERSION     "1.1.0"
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
// ADXL355 / ADXL359 3-axis accelerometer (SPI).  Same register map for both
// parts; this firmware accepts either.  See README for the wiring:
//
//     CS   -> pin 35 (any spare GPIO)      SCK/MOSI/MISO -> pins 13/11/12
//     VDDIO/VSUPPLY -> 3.3 V, GND -> GND (share ground with the DUT)
//
// Do NOT use the SPI pins as channels/outputs while the accel is enabled; the
// viewer warns about collisions.  Registers (ADI datasheet):
//
//     0x00 DEVID_AD = 0xAD     0x05 FIFO_ENTRIES (0..96 LOCATIONS)
//     0x11 FIFO_DATA (stream)  0x28 FILTER: bits[3:0] ODR code
//     0x29 FIFO_SAMPLES (watermark, default 0x60)     0x2D POWER_CTL
//
// ODR codes: 0=4000, 1=2000, 2=1000, 3=500 Hz (LPF corner = ODR/4).
// RANGE reset = +-10 g -> 51200 LSB/g (this firmware leaves the default
// range and reports #ACCRANGE 10).
//
// The FIFO holds 96 21-bit LOCATIONS = 32 SAMPLES (X,Y,Z each 3 bytes; bit 0
// of a group's 3rd byte marks the X-axis group).  Stream mode: when full, the
// oldest sample is overwritten.  Reads pop locations in sets of 3 per 9-byte
// transaction.  20-bit twos-complement decode per group:
//     raw = (b0 << 12) | (b1 << 4) | (b2 >> 4), sign-extended.
//
// We drain the FIFO a few times INSIDE each capture window (so nothing is
// missed at high ODR) and again between windows.  The in-window SPI pauses
// are reported to the host as G lines, so the digital capture is never
// silently wrong.  Samples are timestamped by back-dating from the drain
// instant: the oldest of n drained samples was taken ~n/ODR earlier, each
// successive one 1/ODR later (sub-ms accuracy at ODR <= 4000 Hz), so the
// viewer plots them on the same time base as the logic signals.
// ---------------------------------------------------------------------------
#define ACC_CS_PIN       35           // any spare GPIO
#define ACC_SPI_HZ       10000000UL   // ADXL355/359 SPI max clock
#define ACC_LSB_PER_G    51200        // at +-10 g (reset range)
#define ACC_FIFO_SAMPLES 32           // 96 locations / 3 per sample
#define ACC_STAGE_MAX    512          // staged samples (power of two)
#define ACC_PAUSES_MAX   16           // max in-window SPI pauses per window

#define ACC_REG_DEVID    0x00
#define ACC_REG_PARTID   0x02
#define ACC_REG_FIFO_N   0x05
#define ACC_REG_FIFO_DAT 0x11
#define ACC_REG_FILTER   0x28
#define ACC_REG_FIFO_SMP 0x29
#define ACC_REG_POWER    0x2D
#define ACC_POWER_MEASURE 0x00
#define ACC_POWER_STANDBY 0x01

// {ODR in Hz, FILTER code}
static const uint16_t accOdrTable[][2] = {
    {4000, 0}, {2000, 1}, {1000, 2}, {500, 3}
};

static bool     accelOn = false;      // present AND streaming
static bool     accelPresent = false; // chip answered on the bus
static uint32_t accOdr = 1000;        // current output data rate (Hz)
static uint32_t accOdrCyc = 0;        // cycles per sample at accOdr
static uint32_t accDrainPeriod = 0;   // cycles between in-window FIFO drains
static uint32_t lastAccelDrainAbs = 0;// absolute cycle of the last FIFO drain
static uint32_t accelOverflow = 0;    // staged samples dropped
static uint32_t accelIdle = 0;        // consecutive empty FIFO drains (resync)

// Staging ring: accel samples collected during a window are emitted to USB
// between windows.  t is relative to tBase, like S/E events.
struct AccSample { uint32_t t; int32_t x, y, z; };
static AccSample accStage[ACC_STAGE_MAX];
static volatile uint32_t accStageHead = 0;   // written by capture loop
static uint32_t accStageTail = 0;            // read by main loop

// In-window SPI pauses (absolute cycles); emitted as G lines between windows.
struct AccPause { uint32_t t0, t1; };
static AccPause accPauses[ACC_PAUSES_MAX];
static uint8_t  accPauseCount = 0;

static inline uint32_t accStageUsed(void) {
    return accStageHead - accStageTail;      // free-running counters
}

// --- low-level SPI ----------------------------------------------------------
static uint8_t accReadReg(uint8_t reg) {
    SPI.beginTransaction(SPISettings(ACC_SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWriteFast(ACC_CS_PIN, LOW);
    SPI.transfer((reg << 1) | 0x01);         // read bit
    uint8_t v = SPI.transfer(0x00);
    digitalWriteFast(ACC_CS_PIN, HIGH);
    SPI.endTransaction();
    return v;
}

static void accWriteReg(uint8_t reg, uint8_t val) {
    SPI.beginTransaction(SPISettings(ACC_SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWriteFast(ACC_CS_PIN, LOW);
    SPI.transfer(reg << 1);
    SPI.transfer(val);
    digitalWriteFast(ACC_CS_PIN, HIGH);
    SPI.endTransaction();
}

// --- staging ----------------------------------------------------------------
static void stageAccel(uint32_t tRel, int32_t x, int32_t y, int32_t z) {
    if (accStageUsed() >= ACC_STAGE_MAX - 1) {  // drop the oldest sample
        accStageTail++;
        accelOverflow++;
    }
    accStage[accStageHead & (ACC_STAGE_MAX - 1)] = {tRel, x, y, z};
    accStageHead++;
}

// --- FIFO drain -------------------------------------------------------------
// Read every sample currently in the chip FIFO, decode it, back-date it and
// stage it for USB.  Caller decides when; tNowAbs is the absolute cycle
// counter at drain start.  Runs with IRQs disabled during capture windows
// (polling SPI - no interrupts involved).
/* Returns the FIFO location count read (0..96), or -1 if accel is off.
 * A healthy chip in measure mode shows a growing count; 0 here means the
 * chip is not filling its FIFO (e.g. stuck in standby). */
static int accelCollect(uint32_t tNowAbs) {
    if (!accelOn) return -1;
    uint8_t nLoc = accReadReg(ACC_REG_FIFO_N) & 0x7F;
    if (nLoc > 96) nLoc = 0;                    // garbage read -> treat as empty
    int nSamp = nLoc / 3;                       // 3 locations per sample
    if (nSamp > ACC_FIFO_SAMPLES) nSamp = ACC_FIFO_SAMPLES;
    if (nSamp <= 0) return (int)nLoc;

    // One transaction: address byte, then nSamp * 9 bytes of FIFO stream.
    SPI.beginTransaction(SPISettings(ACC_SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWriteFast(ACC_CS_PIN, LOW);
    SPI.transfer((ACC_REG_FIFO_DAT << 1) | 0x01);

    int taken = 0;                              // valid samples staged
    for (int k = 0; k < nSamp; k++) {
        uint8_t b[9];
        for (int i = 0; i < 9; i++) b[i] = SPI.transfer(0x00);
        if ((b[2] & 0x01) == 0) continue;       // not an X group: skip stray
        int32_t out[3];
        for (int a = 0; a < 3; a++) {
            const uint8_t *p = &b[a * 3];
            int32_t raw = ((int32_t)p[0] << 12) | ((int32_t)p[1] << 4) |
                          ((int32_t)p[2] >> 4);
            if (raw & 0x80000) raw -= 0x100000; // sign-extend 20 bits
            out[a] = raw;
        }
        // Back-date: this is the (taken)-th sample of nSamp, i.e. it was
        // taken (nSamp - taken) ODR periods before the drain instant.
        int64_t tRel = (int64_t)(tNowAbs - tBase) -
                       (int64_t)(nSamp - taken) * accOdrCyc;
        if (tRel < 0) continue;                 // pre-capture data
        stageAccel((uint32_t)tRel,
                   (int32_t)((int64_t)out[0] * 1000 / ACC_LSB_PER_G),
                   (int32_t)((int64_t)out[1] * 1000 / ACC_LSB_PER_G),
                   (int32_t)((int64_t)out[2] * 1000 / ACC_LSB_PER_G));
        taken++;
    }

    digitalWriteFast(ACC_CS_PIN, HIGH);
    SPI.endTransaction();
    return (int)nLoc;
}

// --- config -----------------------------------------------------------------
static void accSetOdr(uint32_t hz) {
    for (size_t i = 0; i < sizeof(accOdrTable) / sizeof(accOdrTable[0]); i++) {
        if (accOdrTable[i][0] == hz) {
            accOdr = hz;
            accOdrCyc = F_CPU_ACTUAL / accOdr;
            // Drain every 3/4 of the FIFO fill time so we never overrun.
            accDrainPeriod = accOdrCyc * (ACC_FIFO_SAMPLES * 3 / 4);
            if (accelPresent) {
                // Change the ODR in standby, then resume measuring: switching
                // the filter rate mid-stream can stall the FIFO on this part.
                accWriteReg(ACC_REG_POWER, ACC_POWER_STANDBY);
                accWriteReg(ACC_REG_FILTER, (uint8_t)accOdrTable[i][1]);
                accWriteReg(ACC_REG_POWER, ACC_POWER_MEASURE);
                delay(10);                       // let the new rate settle
            }
            return;
        }
    }
}

static void accelInit(bool enable) {
    accelPresent = false;
    accelOn = false;
    uint8_t devid = accReadReg(ACC_REG_DEVID);
    if (devid != 0xAD) {
        Serial.println("#ACCERR no ADXL355/359 found on SPI (DEVID_AD != 0xAD)");
        return;
    }
    accelPresent = true;
    // Soft reset first: clears any stuck state (standby, FIFO pointers,
    // datapath) left over from a previous session or a mid-stream glitch.
    accWriteReg(0x2F, 0x52);                     // RESET register (0x2F)
    delay(20);
    devid = accReadReg(ACC_REG_DEVID);
    if (devid != 0xAD) {
        Serial.println("#ACCERR chip did not return after soft reset");
        return;
    }
    accWriteReg(ACC_REG_POWER, ACC_POWER_STANDBY);   // config in standby
    accWriteReg(ACC_REG_FIFO_SMP, 0x60);             // watermark = max
    accSetOdr(accOdr);                               // writes the FILTER reg
    accWriteReg(ACC_REG_POWER, ACC_POWER_MEASURE);   // start measuring
    accelOn = enable;
    if (enable) {
        // Verify the FIFO actually starts filling; report loudly if not.
        delay(30);
        uint8_t fifo = accReadReg(ACC_REG_FIFO_N) & 0x7F;
        if (fifo == 0) {
            Serial.println("#ACCERR init-check: FIFO empty after 30ms (chip not measuring)");
        } else {
            Serial.print("#ACCN ");
            Serial.println(fifo);
        }
    }
}

static void sendAccel(void) {
    Serial.print("#ACC ");
    Serial.println(accelOn ? 1 : 0);
    Serial.print("#ACCODR ");
    Serial.println(accOdr);
    Serial.println("#ACCRANGE 10");
    if (accelOverflow) {
        Serial.print("#ACCOVF ");
        Serial.println(accelOverflow);
        accelOverflow = 0;
    }
}

// Stream staged accel samples to USB (A lines).
static void emitStagedAccel(void) {
    while (accStageTail != accStageHead) {
        if (Serial.availableForWrite() < 48) break;  // never block here
        AccSample e = accStage[accStageTail & (ACC_STAGE_MAX - 1)];
        accStageTail++;
        char buf[48];
        int n = snprintf(buf, sizeof(buf), "A %lu %ld %ld %ld\n",
                         (unsigned long)e.t,
                         (long)e.x, (long)e.y, (long)e.z);
        Serial.write(buf, n);
    }
}

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

            // Accelerometer FIFO is due for a drain.  The SPI transaction
            // pauses digital sampling for a fraction of a ms.  Only report
            // the pause as a gap when an edge was actually lost (the pin
            // state changed during the SPI transaction); reporting every
            // tiny pause as a gap would clear the pulse measurements in the
            // viewer on every window.
            if (accelOn && (now - lastAccelDrainAbs) >= accDrainPeriod) {
                accelCollect(now);
                uint32_t pauseEnd = ARM_DWT_CYCCNT;
                lastAccelDrainAbs = pauseEnd;
                // Pins may have changed during the pause: re-establish the
                // state with an S event (not E - the edge time is unknown).
                uint32_t s2 = readState();
                if (s2 != last) {
                    last = s2;
                    if (accPauseCount < ACC_PAUSES_MAX) {
                        accPauses[accPauseCount].t0 = now;
                        accPauses[accPauseCount].t1 = pauseEnd;
                        accPauseCount++;
                    }
                    if (rbHasSpace(2)) {
                        rb[rbHead & RB_MASK].t = pauseEnd - tBase;
                        rb[rbHead & RB_MASK].s = s2;   // S event
                        rbHead++;
                    } else {
                        overflow++;
                    }
                }
            }
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
            accStageHead = accStageTail = 0;  // drop stale accel data
            accPauseCount = 0;
            accelOverflow = 0;
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
    } else if (strncmp(line, "ACCODR", 6) == 0) {
        long hz = strtol(line + 6, NULL, 10);
        bool ok = false;
        for (size_t i = 0; i < sizeof(accOdrTable) / sizeof(accOdrTable[0]); i++) {
            if (accOdrTable[i][0] == (uint16_t)hz) ok = true;
        }
        if (ok) {
            accSetOdr((uint32_t)hz);
            sendAccel();
        } else {
            Serial.println("#ERR invalid ODR (use 500|1000|2000|4000)");
        }
    } else if (strncmp(line, "ACC", 3) == 0 && (line[3] == ' ' || line[3] == 0)) {
        const char *p = line + 3;
        while (*p == ' ') p++;
        if (strncmp(p, "ON", 2) == 0 && (p[2] == ' ' || p[2] == 0)) {
            if (!accelPresent) accelInit(true);   // (re)probe the bus
            else accelOn = true;
            sendAccel();
        } else if (strncmp(p, "OFF", 3) == 0 && (p[3] == ' ' || p[3] == 0)) {
            accelOn = false;
            sendAccel();
        } else {
            Serial.println("#ERR invalid (use: ACC ON|OFF)");
        }
    } else if (strncmp(line, "INFO", 4) == 0) {
        sendHeader();
        sendPins();
        sendOuts();
        sendAccel();
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
    } else if (strncmp(line, "PULSE", 5) == 0 && (line[5] == ' ' || line[5] == 0)) {
        // PULSE <OE|EN|RST>   (delimiter guard so "PULSEW ..." is not caught here)
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

    // Accelerometer: default SPI pins (SCK 13 / MOSI 11 / MISO 12) + CS 35.
    // If no ADXL355/359 answers, accel stays off and the viewer shows it.
    SPI.begin();
    pinMode(ACC_CS_PIN, OUTPUT);
    digitalWriteFast(ACC_CS_PIN, HIGH);
    accSetOdr(1000);                 // initialize accOdrCyc / accDrainPeriod
    accelInit(true);                 // probe, configure, enable

    delay(500);                      // let USB enumerate
    sendHeader();
    sendPins();
    sendOuts();
    sendAccel();
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
        // In-window accelerometer SPI pauses, as G lines (their times are
        // inside this window; the viewer tolerates out-of-order G lines).
        for (uint8_t i = 0; i < accPauseCount; i++) {
            Serial.print("G ");
            Serial.print(accPauses[i].t0 - tBase);
            Serial.print(' ');
            Serial.println(accPauses[i].t1 - tBase);
        }
        accPauseCount = 0;
        // Accel samples collected inside the window, then whatever else
        // accumulated between windows.
        emitStagedAccel();
        if (accelOn) {
            uint32_t now = ARM_DWT_CYCCNT;
            int n = accelCollect(now);
            lastAccelDrainAbs = now;
            emitStagedAccel();
            // Diagnostics: report the FIFO location count the chip showed at
            // drain time + how many staged samples are waiting for USB.  A
            // healthy chip shows a nonzero count each drain.
            Serial.print("#ACCN ");
            Serial.print(n);
            Serial.print(' ');
            Serial.println(accStageUsed());
            // Self-healing: if the FIFO has been empty for a while the chip
            // is stuck (standby/glitch) - re-initialize it and say so.
            if (n < 3) {
                if (++accelIdle >= 40) {             // ~4 s of empty drains
                    accelIdle = 0;
                    Serial.println("#ACCERR resync: FIFO empty too long, re-initializing");
                    accelInit(true);
                }
            } else {
                accelIdle = 0;
            }
        }
        Serial.print("#RATE ");
        Serial.println(windowSamples);
        if (overflow) {
            Serial.print("#OVF ");
            Serial.println(overflow);
            overflow = 0;
        }
        if (accelOverflow) {
            Serial.print("#ACCOVF ");
            Serial.println(accelOverflow);
            accelOverflow = 0;
        }
    }
}
