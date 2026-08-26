# Teensy 4.1 — 12-Channel Logic Analyzer

Watch 12 digital signals on a Teensy 4.1 with sub-microsecond timing, streamed
live to a browser. Built for signals like yours: **~100 Hz repetition with
1 µs-wide pulses** — narrow pulses that ordinary serial/sample logging would
miss. An **ADXL355/ADXL359 accelerometer on SPI** can stream X/Y/Z vibration
alongside, plotted on the same time base as the logic signals.

Two parts:

| Part | Where | What it does |
|------|-------|--------------|
| **Firmware** | `src/main.cpp` (PlatformIO, board `teensy41`) | Samples the 12 chosen pins at several MS/s, records every edge with CPU-cycle timestamps, streams them over USB Serial; optionally reads an ADXL355/359 accelerometer FIFO (500–4000 Hz) with cycle timestamps |
| **Viewer** | `viewer/index.html` | Web app (Chrome/Edge, Web Serial API): pick the 12 pins, live scrolling waveforms, zoom/pan/cursors, per-channel pulse-width / period / frequency measurements, accelerometer X/Y/Z lanes in mg, CSV export |

No libraries to install beyond PlatformIO + the Teensy core (already present in
this project).

---

## 1. Wiring

* Connect the 12 signals to any of the Teensy 4.1 **digital pins 0–41**.
* **Share a common ground** between the Teensy and the device under test.
* **The Teensy 4.1 is 3.3 V logic and its pins are NOT 5 V tolerant.**
  Level-shift anything above 3.3 V first.
* The pins are read as plain digital inputs (no pull-ups). If your signal is
  open-collector, add your own pull-up.

### Accelerometer (optional, ADXL355 or ADXL359)

Wire it to the default SPI pins plus any spare GPIO for CS:

| ADXL355/359 | Teensy 4.1 pin |
|-------------|----------------|
| CS          | **35** (any spare GPIO; change `ACC_CS_PIN` in `src/main.cpp`) |
| SCK         | 13 |
| MOSI        | 11 |
| MISO        | 12 |
| VDDIO / VSUPPLY | 3.3 V |
| GND         | GND (common ground with everything else) |

* The chip is a 3.3 V part — do not feed it 5 V.
* **Keep pins 11, 12, 13 and the CS pin out of the 12 channel and OE/EN/RST
  lists while the accelerometer is enabled** — the viewer warns if you don't.
* The default channel preset (0–11) includes pin 11 (MOSI): switch the preset
  to e.g. 12–23 (or 30–41) when you use the accelerometer.
* If no chip answers, the firmware simply reports `#ACC 0` and the viewer shows
  “not detected” — the logic analyzer itself is unaffected.

## 2. Build & upload the firmware

```bash
pio run -t upload        # compiles and flashes the Teensy
pio device monitor       # optional: watch the raw serial stream
```

The default channel pins are `0,1,2,...,11` (edit `chPins` in `src/main.cpp`),
but you normally set them from the viewer instead — no re-flash needed.

> Arduino IDE users: the firmware is plain Arduino-compatible code — copy the
> contents of `src/main.cpp` into a sketch named `Teensy12ChannelLA.ino`
> (same-named folder), select *Teensy 4.1* under *Tools → Boards → Teensy*,
> USB Type **Serial**, and upload.

## 3. Run the viewer

Web Serial needs a secure context, so serve the folder (file:// often works in
Chrome, but localhost is guaranteed):

```bash
cd viewer
python3 -m http.server 8000
```

Then open **http://localhost:8000** in Chrome or Edge and click **Connect** —
the Teensy shows up as *Teensy USB Serial*. (Firefox/Safari do not support Web
Serial.)

No hardware handy? Click **Demo** to watch synthetic 100 Hz / 1 µs pulses —
plus an 8 Hz synthetic vibration on the accelerometer lanes — flow through the
whole pipeline.

### Picking the 12 pins

Use the **CH1…CH12 dropdowns** (one per channel). Each lists Teensy pins 0–41
with their pad names (e.g. `Pin 0 — RX1`). The mapping is:

* **applied to the Teensy immediately** when *auto-apply* is on (default),
* **remembered** across reloads (localStorage), and
* **re-applied automatically** the next time you connect.

Preset buttons (0–11, 12–23, 24–35, 30–41, spread) set all twelve at once.
Pins must be 12 distinct values 0–41.

### Using the scope

| Action | Effect |
|--------|--------|
| **Start / Stop** | begin / end a capture (time base resets to 0 at Start) |
| **Reset** | wipe the graph, zoom/pan and cursors only — does **not** start or stop the capture. Keyboard: `R` |
| **OE / EN / RST** | output drivers: pick a pin per output, then **Pulse** drives it HIGH for the configured width (default 100 ms), idle LOW |
| scroll wheel | zoom in/out around the cursor |
| drag | pan (turns off Follow) |
| click / shift+click | place cursors **A** / **B** (Δ time + frequency shown) |
| double-click a lane | zoom to that channel's last pulse |
| **Follow** | keep the view pinned to the newest data |
| **CSV** | download the visible window as a CSV table |
| **Window** | Teensy capture-window length (10 ms–1 s); see “How it works” |

The **Pulse measurements** table tracks, per channel: current level, last
positive-pulse width, last rise-to-rise period, frequency, and pulse count.
Click a pulse-width value to zoom to that pulse.

### Output drivers (OE / EN / RST)

Three extra buttons for driving control lines of your device under test:

* Give each of **OE**, **EN**, **RST** its own pin via the dropdowns (same
  pin list as the channels), or leave one at *“none”*.
* Click **Pulse** (or the ▶ button) to drive that pin **HIGH for the pulse
  width** (default 100 ms, adjustable 1–3000 ms), then back LOW.
* Outputs are **idle LOW, 3.3 V**, configured as outputs only when assigned.
* Config is remembered across reloads and re-applied on connect, exactly like
  the channel pins. Avoid using a pin that you also sample as a channel — the
  viewer warns if you do.
* Note: the pulse timing is accurate to within one capture window; lower the
  **Window** value (e.g. 10–25 ms) if you need tight pulse widths while
  capturing.

### Accelerometer (X / Y / Z lanes)

Once the ADXL355/359 is wired and the firmware is flashed:

* The accelerometer card has an **Accel: ON/OFF** toggle and an **ODR** select
  (500 / 1000 / 2000 / 4000 Hz; default 1000 Hz). State is applied to the
  Teensy immediately and re-synced on connect.
* When ON, three lanes — **AX / AY / AZ**, values in **mg** — appear below the
  12 digital lanes, sharing the exact same time axis (zoom / pan / follow /
  cursors all apply). Hovering shows the accel values at the cursor time.
* The traces autoscale to the visible window. The Z axis sits around +1000 mg
  (1 g) when the board is flat.
* **CSV** merges both domains: accel rows carry the digital state at that
  moment, so one file has everything time-aligned.
* The status line warns if a channel or output pin collides with the SPI pins.

**How the timing works:** the sensor's FIFO only holds 32 samples (8 ms at
4 kHz), so the firmware drains it continuously — a few times *inside* each
capture window and again between windows. The brief SPI pauses (~0.3 ms) show
up as the same orange shaded regions as normal window gaps: the digital
capture is never silently wrong. Samples are timestamped by back-dating from
the drain instant, so each accel point lands within ~1 ms of its true time —
plenty to correlate vibration with your pulses. (If you ever need µs-level
correlation, wire the ADXL's **DRDY** output to one of the 12 channel pins and
watch it as a digital signal.)

## 4. How it works (read this if pulses look wrong)

A 1 µs pulse at 600 MHz is only **600 CPU cycles** wide — too short for
interrupts or normal logging. Instead:

1. The firmware polls all 12 pins (GPIO pad-status registers) in a tight,
   interrupt-free loop — **typically 5–8 million samples/second**, so a 1 µs
   pulse is caught 5–8 times. Every sample is timestamped with the ARM cycle
   counter (1.67 ns resolution).
2. Only **edges** (state changes) are kept: `(timestamp, 12-bit state)` pairs.
   For a 100 Hz signal with 1 µs pulses that's ~2400 edges/s → a few KB/s on
   USB. The viewer reconstructs the waveforms from the edges.
3. Capture runs in **windows** (default 100 ms) with interrupts disabled for
   deterministic timing. Between windows the buffered edges are streamed to
   USB, and the unsampled time is reported as a **gap** — the viewer shades
   those regions orange instead of guessing. At typical data rates the gaps
   are only a few ms every 100 ms.

**Timing accuracy:** edge timestamps are exact to the sampled cycle (±1 sample
period, ~150–200 ns worst case), so pulse widths are measured to well under a
microsecond.

### Limits (be honest with yourself)

* **Minimum pulse you can trust:** roughly 2 sample periods (≈ 300–400 ns).
  A 1 µs pulse is fine; pulses below ~0.3 µs may be missed or distorted.
* **Maximum edge rate:** bounded by the USB drain between windows. Thousands
  of edges per second are trivial; if you see `⚠ ring overflow`, your signal
  toggles too fast for the current window setting — lower the Window value.
* **Windows mean brief pauses.** The viewer marks them; if you need a gapless
  long capture, this design (edge capture) still records everything meaningful.
* **Accelerometer limits:** 500–4000 Hz ODR, ±10 g at 51200 LSB/g (the reset
  range — leave `RANGE` alone unless you change the scale in the firmware).
  Each in-window FIFO drain pauses digital sampling for ~0.3 ms (shown as a
  gap); at 4000 Hz that's about 5 % duty of brief pauses. Accel timestamps are
  good to ~1 ms — µs-level correlation needs the DRDY-on-a-channel trick.

## 5. Wire protocol (for your own tools)

Newline-terminated text over USB Serial (115200 baud, ignored for USB CDC).

**Host → Teensy**

```
PINS 0 1 2 3 4 5 6 7 8 9 10 11   configure the 12 channels (pins 0..41)
START                            begin capture, time base resets to 0
STOP                             stop capture
WIN 100                          capture-window length in ms (10..5000)
SETPIN OE 22                     assign output pin (or -1 to release)
PULSE  RST                       pulse that output HIGH for PULSEW ms
PULSEW 100                       output pulse width in ms (1..3000)
ACC ON|OFF                       enable/disable the accelerometer stream
ACCODR 1000                      accelerometer ODR in Hz (500|1000|2000|4000)
INFO                             re-send header/pin/output/status lines
```

**Teensy → Host**

```
#FW Teensy12ChannelLA 1.0.0      firmware identity
#FCPU 600000000                  actual CPU clock in Hz
#NCH 12
#PINS 0 1 2 ... 11               current channel->pin mapping
#OUT OE 22                       output pin assignment (-1 = none)
#PULSEW 100                      output pulse width
#PULSE RST                       output pulse acknowledgement
#WIN 100                         window length
#ACC 1                           accelerometer streaming on (0 = off/absent)
#ACCODR 1000                     accelerometer output data rate
#ACCRANGE 10                     accelerometer full-scale range in g
#ACCOVF 3                        staged accel samples dropped (host too slow)
#ACCERR no ADXL355/359...        accelerometer not found / comm failure
#READY / #START / #STOP          acknowledgements
#RATE 812345                     samples taken in the last window
#OVF 7                           events dropped (signal too fast)
#ERR ...                         command rejected
S <t> <hex>                      set state: at cycle t the 12-bit state is hex
E <t> <hex>                      edge: state changed to hex at cycle t
G <t0> <t1>                      gap: no sampling between cycles t0 and t1
A <t> <x> <y> <z>                accel sample at cycle t; x/y/z in mg (int)
```

`t` is CPU cycles relative to the last START (wraps every ~7 s at 600 MHz; the
viewer unwraps). State bits are channel 1 = bit 0 … channel 12 = bit 11.
Accel samples are back-dated from the FIFO drain instant (oldest of n samples
≈ n/ODR before the drain), so their timestamps land within ~1 ms of truth;
occasional tiny backwards steps near window boundaries are tolerated by the
viewer's unwrap.

## 6. Project layout

```
src/main.cpp            Teensy firmware (PlatformIO)
platformio.ini          board = teensy41, -O2
viewer/index.html       viewer UI
viewer/style.css        viewer styling
viewer/app.js           viewer logic (data core is Node-testable)
tests/test-core.js      unit tests for the viewer's parser/measurement core
legacy_vibration/       archived ADXL359 vibration project (previous contents)
```

Run the unit tests with `node tests/test-core.js`.

## 7. Troubleshooting

* **Viewer can't see Web Serial** → use Chrome/Edge over http://localhost (not
  file://, not Safari/Firefox).
* **Connect succeeds but no waveform** → press Start; check the status badge
  shows “Capturing”; confirm your signal is 0–3.3 V with common ground.
* **Pulses show as tiny spikes or are missing** → zoom in (wheel) or double-
  click the lane; verify the pulse is wider than ~0.3 µs; check the samples/s
  readout in the toolbar (5–8 M is normal).
* **`⚠ ring overflow`** → the signal is changing too fast for the stream;
  lower the Window to 10–25 ms.
* **Orange shaded regions** → normal: they are the tiny gaps between capture
  windows (plus the ~0.3 ms SPI pauses while the accelerometer FIFO is
  drained; they appear more often with accel ON).
* **Accelerometer says “not detected”** → check the wiring (CS=35, SCK=13,
  MOSI=11, MISO=12, VDDIO on 3.3 V, common ground) and that the part is an
  ADXL355/ADXL359; the DEVID check must read 0xAD. Fix it and click **Accel:
  ON** — no re-flash needed (or re-flash if you changed `ACC_CS_PIN`).
* **Accel: ON click does nothing / “no reply from the Teensy”** → the Teensy
  is almost certainly still running the OLD firmware (before accelerometer
  support). Re-flash with `pio run -t upload` and check the log line
  `Teensy firmware: Teensy12ChannelLA 1.1.0` after reconnecting. If you see
  `1.0.0` or no version, the new firmware did not take.
* **Accelerometer says “accel samples dropped”** → the USB link can't keep up
  at this ODR; lower the ODR or shorten the **Window**.
* **Accel: ON but empty lanes / no `A` lines in the monitor** → the chip's
  FIFO isn't filling (stuck in standby or a glitch). Firmware 1.1.0+ prints
  `#ACCN <n>` after every window (0 = chip not measuring) and auto-recovers:
  after ~4 s of empty drains it re-initializes the chip and prints
  `#ACCERR resync...`. If you see `#ACCN 0` repeatedly, check the wiring
  (MISO=12, VDDIO, CS=35) and click **Accel: OFF** then **ON** to re-probe.
* **Teensy disappeared after upload** → normal: it resets; click Connect again
  (the viewer re-applies your saved pins automatically).
