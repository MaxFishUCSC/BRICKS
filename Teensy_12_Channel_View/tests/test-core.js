// Unit test for the viewer's core data logic (runs in Node, no browser needed).
// Simulates a Teensy stream of S/E/G lines and checks parsing, unwrapping,
// pulse measurement, and gap handling.
'use strict';
const assert = require('assert');
const { CORE, handleLine, handleMeta, fmtTime, fmtFreq } = require('../viewer/app.js');

function reset() {
  CORE.fcpu = 600000000;
  CORE.evT = []; CORE.evS = []; CORE.gaps = [];
  CORE.lastRaw = null; CORE.wrapAcc = 0;
  CORE.meas = [];
  require('../viewer/app.js'); // ensure initMeas ran; re-run below
  // re-init measurements
  const { initMeas } = require('../viewer/app.js');
  initMeas();
}
const { initMeas } = require('../viewer/app.js');

let failures = 0;
function check(name, fn) {
  try { fn(); console.log('  ok  ' + name); }
  catch (e) { failures++; console.log('FAIL  ' + name + '\n      ' + e.message); }
}

// ---------------------------------------------------------------- basic parse
reset();
check('metadata + S baseline', () => {
  handleLine('#FCPU 600000000');
  assert.strictEqual(CORE.fcpu, 600000000);
  handleLine('S 0 000');
  assert.strictEqual(CORE.evT.length, 1);
  assert.strictEqual(CORE.evS[0], 0);
  assert.strictEqual(CORE.evT[0], 0);
});

// ----------------------------------------- 100 Hz / 1 us pulse on channel 0
reset();
check('100 Hz 1us pulse measurement (CH1)', () => {
  handleLine('#FCPU 600000000');
  handleLine('S 0 000');
  // channel 0 (bit 0): rise at 5 ms, fall at 5.001 ms, rise at 15 ms
  const cyc = (ms) => Math.round(ms * 1e-3 * 600000000);
  handleLine('E ' + cyc(5) + ' 001');
  handleLine('E ' + cyc(5.001) + ' 000');
  handleLine('E ' + cyc(15) + ' 001');
  handleLine('E ' + cyc(15.001) + ' 000');
  const m = CORE.meas[0];
  assert.ok(Math.abs(m.pulseW - 1e-6) < 1e-9, 'pulseW=' + m.pulseW);
  assert.ok(Math.abs(m.period - 0.01) < 1e-6, 'period=' + m.period);
  assert.strictEqual(m.pulses, 2);
  assert.strictEqual(m.val, 0);
  assert.ok(Math.abs(CORE.evT[4] - 15.001e-3) < 1e-9);
});

// ------------------------------------------------------- multiple channels
reset();
check('multi-channel edges + measurement isolation', () => {
  handleLine('#FCPU 600000000');
  handleLine('S 0 000');
  const cyc = (ms) => Math.round(ms * 1e-3 * 600000000);
  // CH1 pulse 1us, CH2 pulse 2us
  handleLine('E ' + cyc(2) + ' 003');   // both rise
  handleLine('E ' + cyc(2.001) + ' 001'); // CH2 falls
  handleLine('E ' + cyc(2.002) + ' 000'); // CH1 falls
  assert.ok(Math.abs(CORE.meas[0].pulseW - 2e-6) < 1e-9, 'ch1 pulseW=' + CORE.meas[0].pulseW);
  assert.ok(Math.abs(CORE.meas[1].pulseW - 1e-6) < 1e-9, 'ch2 pulseW=' + CORE.meas[1].pulseW);
});

// ---------------------------------------------------------------- cycle wrap
reset();
check('32-bit cycle wrap unwrapping', () => {
  handleLine('#FCPU 600000000');
  const WRAP = 4294967296;
  handleLine('S 0 000');
  // event near the wrap point, then a small timestamp after wrap
  handleLine('E ' + (WRAP - 1000) + ' 001');
  handleLine('E ' + 10 + ' 000');      // wrapped
  assert.ok(CORE.evT[1] > CORE.evT[0], 'times must stay monotonic after wrap: ' + CORE.evT[1] + ' vs ' + CORE.evT[0]);
  assert.ok(Math.abs(CORE.evT[2] - (WRAP + 10) / 600000000) < 1e-6);
});

// ------------------------------------------------------------------- gaps
reset();
check('gap handling + measurement invalidation', () => {
  handleLine('#FCPU 600000000');
  handleLine('S 0 000');
  const cyc = (ms) => Math.round(ms * 1e-3 * 600000000);
  handleLine('E ' + cyc(5) + ' 001');
  handleLine('E ' + cyc(5.001) + ' 000');
  assert.ok(CORE.meas[0].pulseW !== null);
  handleLine('G ' + cyc(20) + ' ' + cyc(20.5));   // gap
  assert.strictEqual(CORE.gaps.length, 1);
  assert.strictEqual(CORE.meas[0].pulseW, null, 'pulse width must reset across a gap');
  // state changed inside the gap -> S re-establishes
  handleLine('S ' + cyc(20.5) + ' 002');          // CH2 high now
  assert.strictEqual(CORE.meas[1].val, 1);
  // new pulse after gap still measures correctly
  handleLine('E ' + cyc(30) + ' 000');
  handleLine('E ' + cyc(30.001) + ' 002');
  handleLine('E ' + cyc(30.002) + ' 000');
  assert.ok(Math.abs(CORE.meas[1].pulseW - 1e-6) < 1e-9);
});

// ----------------------------------------------------------- time formatting
check('time formatting', () => {
  assert.strictEqual(fmtTime(0), '0');
  assert.strictEqual(fmtTime(1e-9), '1 ns');
  assert.strictEqual(fmtTime(1e-6), '1.00 µs');
  assert.strictEqual(fmtTime(1e-3), '1.00 ms');
  assert.strictEqual(fmtTime(0.5), '500.00 ms');
  assert.strictEqual(fmtTime(1), '1.000 s');
  assert.strictEqual(fmtFreq(100), '100.0 Hz');
  assert.strictEqual(fmtFreq(5000), '5.00 kHz');
  assert.strictEqual(fmtFreq(1e6), '1.00 MHz');
});

// ------------------------------------------------------- bad input tolerance
reset();
check('robust against junk lines', () => {
  handleLine('#GARBAGE nonsense here');
  handleLine('XYZ 1 2 3');
  handleLine('');
  handleLine('E notanumber xyz');
  handleLine('G 5');                  // incomplete
  handleLine('S 10');                 // incomplete
  assert.strictEqual(CORE.evT.length, 0);
  assert.strictEqual(CORE.gaps.length, 0);
});

// ------------------------------------------------------------ demo generator
reset();
check('demo generator produces a sane stream', () => {
  // replay the demo event math directly against the core (time-ordered)
  handleLine('#FCPU 600000000');
  handleLine('S 0 000');
  const PERIOD = 0.010, PW = 1e-6, T = 0.1; // 10 periods
  const evs = [];
  for (let ch = 0; ch < 12; ch++) {
    const phase = (ch / 12) * PERIOD;
    for (let k = 1; k * PERIOD + phase <= T; k++) {
      const rise = k * PERIOD + phase;
      evs.push([rise, 1 << ch]);
      evs.push([rise + PW, 1 << ch]);
    }
  }
  evs.sort((a, b) => a[0] - b[0]);
  let s = 0;
  for (const [t, bit] of evs) {
    s ^= bit;   // each channel's events strictly alternate rise/fall
    handleLine('E ' + Math.round(t * CORE.fcpu) + ' ' + s.toString(16));
  }
  assert.ok(CORE.evT.length > 100);
  assert.ok(Math.abs(CORE.meas[0].pulseW - PW) < 1e-8, 'ch1 pulseW=' + CORE.meas[0].pulseW);
  assert.ok(Math.abs(CORE.meas[0].period - PERIOD) < 1e-6, 'ch1 period=' + CORE.meas[0].period);
  // state at a known time: inside ch5's k=5 pulse
  const { stateAt } = require('../viewer/app.js');
  const tMid = (5 + 5 / 12) * PERIOD + 0.5e-6;
  const sAt = stateAt(tMid);
  assert.ok(sAt !== null && (sAt & (1 << 5)) !== 0, 'ch5 should be high at tMid');
  const tOff = (5 + 5 / 12) * PERIOD + 5e-6;  // 5 us later, pulse over
  assert.ok(((stateAt(tOff) >> 5) & 1) === 0, 'ch5 should be low at tOff');
});

// ----------------------------------------- real generator, 1 full simulated s
reset();
check('real demoEvent generator over 1 s (100 Hz, 1 us pulses, 12 ch)', () => {
  const { demoEvent } = require('../viewer/app.js');
  handleLine('#FCPU 600000000');
  handleLine('S 0 000');
  demoEvent(0, 1.0);
  assert.ok(Math.abs(CORE.evT.length - 2400) <= 4, 'expected ~2400 events, got ' + CORE.evT.length);
  // monotonic timestamps
  for (let i = 1; i < CORE.evT.length; i++) {
    assert.ok(CORE.evT[i] > CORE.evT[i - 1], 'event times must be strictly increasing at ' + i);
  }
  // every channel measured the 1 us pulse and 10 ms period
  for (let ch = 0; ch < 12; ch++) {
    assert.ok(Math.abs(CORE.meas[ch].pulseW - 1e-6) < 1e-8, 'ch' + (ch + 1) + ' pulseW=' + CORE.meas[ch].pulseW);
    assert.ok(Math.abs(CORE.meas[ch].period - 0.010) < 1e-6, 'ch' + (ch + 1) + ' period=' + CORE.meas[ch].period);
    assert.ok(CORE.meas[ch].pulses >= 99, 'ch' + (ch + 1) + ' pulses=' + CORE.meas[ch].pulses);
  }
  // sampling the state at several instants matches the known pattern
  const { stateAt } = require('../viewer/app.js');
  const t = 0.5550005;                // 0.5 us after ch6's k=55 rising edge
  const expectedHigh = [];
  for (let ch = 0; ch < 12; ch++) {
    const phase = (ch / 12) * 0.010;
    const rise = 55 * 0.010 + phase;
    if (t >= rise && t < rise + 1e-6) expectedHigh.push(ch);
  }
  assert.strictEqual(expectedHigh.length, 1, 'exactly one channel high at t, got ' + expectedHigh);
  const st = stateAt(t);
  for (let ch = 0; ch < 12; ch++) {
    assert.strictEqual(((st >> ch) & 1) === 1, expectedHigh.includes(ch),
      'ch' + (ch + 1) + ' at t=' + t);
  }
});

// -------------------------------------------------- trim keeps things bounded
reset();
check('trimData bounds memory', () => {
  const { demoEvent, trimData } = require('../viewer/app.js');
  handleLine('#FCPU 600000000');
  handleLine('S 0 000');
  demoEvent(0, 700);                  // > 1.5M events and > 600 s -> trimmed
  const before = CORE.evT.length;
  trimData();
  assert.ok(before > 1500000, 'generated plenty of data: ' + before);
  assert.ok(CORE.evT.length <= 1500000, 'event cap respected: ' + CORE.evT.length);
  assert.ok(CORE.evT[CORE.evT.length - 1] - CORE.evT[0] <= 601, 'time window capped');
});

console.log(failures === 0 ? '\nALL TESTS PASSED' : '\n' + failures + ' TEST(S) FAILED');
process.exit(failures === 0 ? 0 : 1);
