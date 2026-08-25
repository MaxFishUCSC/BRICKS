/* ============================================================================
 * Teensy 4.1 - 12-Channel Logic Analyzer viewer
 *
 * Talks to the Teensy over the Web Serial API (Chrome / Edge, secure context:
 * http://localhost or https).  Parses the text protocol documented in the
 * firmware source, renders scrolling logic-analyzer waveforms on a canvas and
 * measures pulse widths / periods per channel.
 *
 * The "core" (data parsing + measurement math) is DOM-free so it can be unit
 * tested in Node.
 * ==========================================================================*/
'use strict';

/* --------------------------------------------------------------------------
 * Core: constants, state, parsing, measurement
 * ------------------------------------------------------------------------*/
const NCH = 12;
const MAX_PIN = 41;
const CYCLE_WRAP = 4294967296; // 2^32

const PIN_NAMES = {
  0:'RX1', 1:'TX1', 2:'RX2', 3:'TX2', 4:'RX3', 5:'TX3', 6:'RX4', 7:'TX4',
  8:'RX5', 9:'TX5', 10:'RX6', 11:'TX6', 12:'RX7', 13:'TX7', 14:'RX8', 15:'TX8',
  16:'RX9', 17:'TX9', 18:'RX10', 19:'TX10', 20:'RX11', 21:'TX11',
  22:'RX12', 23:'TX12',
  24:'AD_B0_12', 25:'AD_B0_13', 26:'AD_B1_14', 27:'AD_B1_15',
  28:'EMC_32', 29:'EMC_31', 30:'EMC_37', 31:'EMC_36',
  32:'B0_12', 33:'EMC_07', 34:'B1_13', 35:'B1_12', 36:'B1_02', 37:'B1_03',
  38:'AD_B1_12', 39:'AD_B1_13', 40:'AD_B1_04', 41:'AD_B1_05'
};

const CH_COLORS = [
  '#ff5252', '#ff9800', '#ffd740', '#9ccc65', '#26c6da', '#42a5f5',
  '#ab47bc', '#ec407a', '#ff7043', '#aed581', '#4dd0e1', '#f06292'
];

const DEFAULT_PINS = [0,1,2,3,4,5,6,7,8,9,10,11];

/* Output drivers (OE / EN / RST): assignable pins that pulse HIGH on demand. */
const OUT_NAMES = ['OE', 'EN', 'RST'];
const OUT_COLORS = { OE: '#ffd54f', EN: '#4db6ac', RST: '#f48fb1' };
const DEFAULT_OUTS = { oe: -1, en: -1, rst: -1, pw: 100 };   // -1 = none

/* Shared core state (also used by the demo generator). */
const CORE = {
  fcpu: 0,
  winMs: 100,
  pins: DEFAULT_PINS.slice(),
  outs: Object.assign({}, DEFAULT_OUTS),
  evT: [],          // event times in seconds (monotonic, after unwrap)
  evS: [],          // 12-bit states
  gaps: [],         // {t0, t1} in seconds - not sampled
  lastRaw: null,    // last raw cycle timestamp (unwrap tracking)
  wrapAcc: 0,       // accumulated 2^32 wraps
  wallStart: null,  // performance.now() when capture started
  capturing: false,
  overflow: false,
  lastRate: 0, rateAt: 0,
  evCountAt: 0, evCountT: 0, eventsPerSec: 0,
  meas: []          // per-channel measurement state (see initMeas)
};

function initMeas() {
  CORE.meas = [];
  for (let i = 0; i < NCH; i++) {
    CORE.meas.push({
      val: null,           // current level (0/1/null=unknown)
      rise: null,          // time of the most recent rising edge
      armed: false,        // true between a rise and the following fall
      pulseW: null,        // width of last completed positive pulse (s)
      lastRise: null,      // time of last completed pulse's rise
      lastFall: null,      // time of last completed pulse's fall
      period: null,        // last rise-to-rise period (s)
      pulses: 0
    });
  }
}
initMeas();

function lowerBound(arr, v) {
  let lo = 0, hi = arr.length;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (arr[mid] < v) lo = mid + 1; else hi = mid;
  }
  return lo;
}

/* Convert a raw 32-bit cycle timestamp into an unwrapped cycle count. */
function unwrap(raw) {
  if (CORE.lastRaw !== null && raw < CORE.lastRaw) CORE.wrapAcc += CYCLE_WRAP;
  CORE.lastRaw = raw;
  return CORE.wrapAcc + raw;
}

function stateAt(t) {
  const i = lowerBound(CORE.evT, t);
  if (i === 0) return null;            // before the first event: unknown
  return CORE.evS[i - 1];
}

/* ---- events ---- */

function applySet(ts, s) {
  CORE.evT.push(ts);
  CORE.evS.push(s);
  for (let i = 0; i < NCH; i++) {
    const bit = (s >> i) & 1;
    const m = CORE.meas[i];
    if (m.val !== null && m.val !== bit) {
      // Level changed during a gap: rise/fall timing is unknown.
      m.rise = null; m.armed = false;
    }
    m.val = bit;
  }
}

function applyEdge(ts, s) {
  const prev = CORE.evS.length ? CORE.evS[CORE.evS.length - 1] : 0;
  CORE.evT.push(ts);
  CORE.evS.push(s);
  for (let i = 0; i < NCH; i++) {
    const old = (prev >> i) & 1, nw = (s >> i) & 1;
    if (old === nw) continue;
    const m = CORE.meas[i];
    if (nw === 1) {                       // rising edge
      if (m.lastRise !== null) m.period = ts - m.lastRise;
      m.rise = ts;
      m.armed = true;
    } else if (m.armed && m.rise !== null) { // falling edge -> pulse complete
      m.pulseW = ts - m.rise;
      m.lastRise = m.rise;
      m.lastFall = ts;
      m.pulses++;
      m.armed = false;
    }
    m.val = nw;
  }
}

function invalidateMeas() {
  for (let i = 0; i < NCH; i++) {
    const m = CORE.meas[i];
    m.rise = null; m.armed = false;
    m.pulseW = null; m.period = null;
    m.lastRise = null; m.lastFall = null;
  }
}

/* Keep memory bounded: drop data older than 10 minutes or beyond 1.5M events. */
function trimData() {
  const MAX_N = 1500000, MAX_T = 600;
  if (CORE.evT.length > MAX_N) {
    const cut = CORE.evT.length - MAX_N;
    CORE.evT.splice(0, cut);
    CORE.evS.splice(0, cut);
  }
  if (CORE.evT.length && CORE.evT[CORE.evT.length - 1] - CORE.evT[0] > MAX_T) {
    const t0 = CORE.evT[CORE.evT.length - 1] - MAX_T;
    const i = lowerBound(CORE.evT, t0);
    if (i > 0) { CORE.evT.splice(0, i); CORE.evS.splice(0, i); }
  }
  if (CORE.evT.length) {
    const gmin = CORE.evT[0];
    if (CORE.gaps.length) CORE.gaps = CORE.gaps.filter(g => g.t1 >= gmin);
  }
}

/* --------------------------------------------------------------------------
 * Demo generator (DOM-free): synthetic 100 Hz signals with 1 us pulses,
 * phase-staggered across the 12 channels.  Feeds the normal parse path.
 * Events are sorted by time before emission: the viewer assumes monotonic
 * timestamps (the real firmware produces them in time order).
 * ------------------------------------------------------------------------*/
function demoEvent(tStart, tEnd) {
  const PERIOD = 0.010, PW = 1e-6;
  const evs = [];
  for (let ch = 0; ch < NCH; ch++) {
    const phase = (ch / NCH) * PERIOD;
    let k = Math.floor((tStart - phase) / PERIOD) + 1;
    let rise = k * PERIOD + phase;
    while (rise < tEnd) {
      evs.push([rise, 1 << ch]);
      evs.push([rise + PW, 1 << ch]);
      k++;
      rise = k * PERIOD + phase;
    }
  }
  evs.sort((a, b) => a[0] - b[0]);
  for (const [t, bit] of evs) {
    const prev = CORE.evS.length ? CORE.evS[CORE.evS.length - 1] : 0;
    handleLine('E ' + Math.round(t * CORE.fcpu) + ' ' + ((prev ^ bit).toString(16)).toUpperCase().padStart(3, '0'));
  }
}

/* ---- line parser ---- */

function handleLine(line) {
  if (!line) return;
  if (line[0] === '#') return handleMeta(line);
  const sp = line.indexOf(' ');
  if (sp < 0) return;
  const kind = line.slice(0, sp);
  const rest = line.slice(sp + 1).trim();
  if (kind === 'S' || kind === 'E') {
    const sp2 = rest.indexOf(' ');
    if (sp2 < 0) return;
    const rawT = parseInt(rest.slice(0, sp2), 10);
    const s = parseInt(rest.slice(sp2 + 1).trim(), 16);
    if (!isFinite(rawT) || !isFinite(s)) return;
    if (CORE.fcpu > 0) {
      const ts = unwrap(rawT) / CORE.fcpu;
      if (kind === 'S') applySet(ts, s); else applyEdge(ts, s);
    }
  } else if (kind === 'G') {
    const p = rest.split(/\s+/);
    if (p.length < 2) return;
    const t0 = unwrap(parseInt(p[0], 10));
    const t1 = unwrap(parseInt(p[1], 10));
    if (CORE.fcpu > 0) {
      CORE.gaps.push({ t0: t0 / CORE.fcpu, t1: t1 / CORE.fcpu });
      invalidateMeas();
    }
  }
}

function handleMeta(line) {
  const p = line.split(/\s+/);
  switch (p[0]) {
    case '#FW':      break;                       // handled by UI log
    case '#FCPU':    CORE.fcpu = parseInt(p[1], 10) || 0; break;
    case '#NCH':     break;
    case '#PINS': {
      const pins = p.slice(1).map(Number);
      if (pins.length === NCH && pins.every(n => Number.isInteger(n) && n >= 0 && n <= MAX_PIN)) {
        CORE.pins = pins;
      }
      break;
    }
    case '#WIN':     CORE.winMs = parseInt(p[1], 10) || 100; break;
    case '#READY':   break;                       // handled by UI
    case '#START':   break;                       // handled by UI
    case '#STOP':    break;
    case '#RATE':    CORE.lastRate = parseInt(p[1], 10) || 0; CORE.rateAt = Date.now(); break;
    case '#OVF':     CORE.overflow = true; break;
    default:         break;
  }
}

/* ---- formatting helpers (used by UI, kept here for testing) ---- */

function fmtTime(t) {
  if (t === null || t === undefined || !isFinite(t)) return '—';
  if (t === 0) return '0';
  const a = Math.abs(t);
  if (a >= 1) return t.toFixed(a >= 100 ? 1 : 3) + ' s';
  if (a >= 1e-3) return (t * 1e3).toFixed(a >= 1 ? 1 : 2) + ' ms';
  if (a >= 1e-6) return (t * 1e6).toFixed(a >= 1 ? 1 : 2) + ' µs';
  return (t * 1e9).toFixed(0) + ' ns';
}

function fmtFreq(f) {
  if (f === null || f === undefined || !isFinite(f) || f <= 0) return '—';
  if (f >= 1e6) return (f / 1e6).toFixed(2) + ' MHz';
  if (f >= 1e3) return (f / 1e3).toFixed(2) + ' kHz';
  return f.toFixed(f >= 100 ? 1 : 2) + ' Hz';
}

function fmtRate(r) {
  if (r >= 1e6) return (r / 1e6).toFixed(1) + ' M';
  if (r >= 1e3) return (r / 1e3).toFixed(1) + ' k';
  return r.toFixed(0);
}

/* Export core for Node tests */
if (typeof module !== 'undefined' && module.exports) {
  module.exports = { CORE, NCH, MAX_PIN, handleLine, handleMeta, lowerBound,
                     unwrap, applySet, applyEdge, stateAt, initMeas, trimData,
                     demoEvent, fmtTime, fmtFreq, fmtRate,
                     CH_COLORS, PIN_NAMES, DEFAULT_PINS };
}

/* ==========================================================================
 * UI layer (only runs in the browser)
 * ==========================================================================*/
if (typeof document !== 'undefined') {

const $ = (id) => document.getElementById(id);
const els = {
  btnConnect: $('btnConnect'), connStatus: $('connStatus'), devInfo: $('devInfo'),
  btnStart: $('btnStart'), btnStop: $('btnStop'), btnDemo: $('btnDemo'),
  btnReset: $('btnReset'),
  btnApply: $('btnApply'), chkAutoApply: $('chkAutoApply'), pinList: $('pinList'), pinErr: $('pinErr'),
  btnApplyOuts: $('btnApplyOuts'), chkAutoApplyOuts: $('chkAutoApplyOuts'),
  pulseMs: $('pulseMs'), outList: $('outList'), outErr: $('outErr'),
  winSel: $('winSel'), spanSel: $('spanSel'), chkFollow: $('chkFollow'),
  btnExport: $('btnExport'), stats: $('stats'), wave: $('wave'), waveWrap: $('waveWrap'),
  cursorRead: $('cursorRead'), measBody: $('measBody'), log: $('log')
};

const VIEW = { span: 0.5, right: 0.001, follow: true, cursorA: null, cursorB: null, hoverT: null };
const UI = { connected: false, demo: false, syncing: false, dirty: true,
             lastStats: 0, lastTrim: 0, lastNoDataWarn: 0, infoSentAt: 0,
             lastConnWarn: 0, dragging: null, demoStart: 0, demoHandle: null };

/* ---------- log ---------- */
function log(msg, cls) {
  const d = document.createElement('div');
  if (cls) d.className = cls;
  d.textContent = msg;
  els.log.appendChild(d);
  while (els.log.childNodes.length > 300) els.log.removeChild(els.log.firstChild);
  els.log.scrollTop = els.log.scrollHeight;
}

function setStatus(text, cls) {
  els.connStatus.textContent = text;
  els.connStatus.className = 'badge ' + (cls || 'off');
}

function updateDevInfo() {
  els.devInfo.textContent = CORE.fcpu ? (CORE.fcpu / 1e6) + ' MHz · ' + NCH + ' ch' : '';
}

/* ---------- serial ---------- */
let port = null, reader = null, writer = null;

async function connect() {
  if (port) { await disconnect(); return; }
  if (!('serial' in navigator)) {
    setStatus('Web Serial unavailable', 'err');
    log('Web Serial API not available. Use Chrome or Edge, served from http://localhost or https.', 'err');
    return;
  }
  try {
    if (UI.demo) toggleDemo();          // demo owns the data stream - stop it
    port = await navigator.serial.requestPort();
    await port.open({ baudRate: 115200 });   // baud is ignored for USB CDC
    reader = port.readable.getReader();
    writer = port.writable.getWriter();
    UI.connected = true;
    els.btnConnect.textContent = 'Disconnect';
    els.btnStart.disabled = false;
    updateApplyBtn();
    updateApplyOutsBtn();
    setStatus('Connected', 'ok');
    logPortIdentity();
    port.addEventListener('disconnect', () => { disconnect('Teensy disconnected (reset or unplugged?)'); });
    readLoop();
    sendCmd('INFO');
    UI.infoSentAt = performance.now();
  } catch (e) {
    if (e && e.name === 'NotFoundError') return;         // user cancelled picker
    setStatus('Connect failed', 'err');
    log('Connect failed: ' + (e && e.message ? e.message : e), 'err');
    port = null;
  }
}

/* Log which serial port we actually opened, so a wrong-port pick is obvious. */
async function logPortIdentity() {
  try {
    let info = null;
    const r = port.getInfo ? port.getInfo() : null;
    info = (r && typeof r.then === 'function') ? await r : r;
    const name = (info && info.usbProductName) ? info.usbProductName : '';
    const vid = info && info.usbVendorId ? info.usbVendorId : null;
    const pid = info && info.usbProductId ? info.usbProductId : null;
    let msg = 'Connected to: ' + (name || 'serial device');
    if (vid) msg += ' (VID:0x' + vid.toString(16).toUpperCase() + ' PID:0x' + (pid || 0).toString(16).toUpperCase() + ')';
    log(msg);
    if (vid && vid !== 0x16C0) {
      log('Note: that vendor ID is not PJRC (Teensy). If this is not your Teensy, disconnect and pick the right port.', 'warn');
    }
  } catch (e) { /* getInfo not supported - no problem */ }
}

async function disconnect(msg) {
  UI.connected = false;
  try { if (reader) { await reader.cancel(); } } catch (e) {}
  try { if (writer) { writer.releaseLock(); } } catch (e) {}
  try { if (port) { await port.close(); } } catch (e) {}
  reader = null; writer = null; port = null;
  els.btnConnect.textContent = '\u{1F50C} Connect';
  els.btnStart.disabled = true;
  els.btnStop.disabled = true;
  updateApplyBtn();
  updateApplyOutsBtn();
  setStatus('Not connected', 'off');
  if (msg) log(msg);
}

function sendCmd(cmd) {
  if (writer) {
    const enc = new TextEncoder();
    writer.write(enc.encode(cmd + '\n')).catch(() => {});
  }
}

async function readLoop() {
  const decoder = new TextDecoder();
  let buf = '';
  try {
    while (reader) {
      const { value, done } = await reader.read();
      if (done) break;
      buf += decoder.decode(value, { stream: true });
      let idx;
      while ((idx = buf.indexOf('\n')) >= 0) {
        const line = buf.slice(0, idx).replace(/\r$/, '').trim();
        buf = buf.slice(idx + 1);
        if (line) { handleLine(line); onDataLine(line); }
      }
    }
  } catch (e) { /* stream cancelled on disconnect */ }
  if (UI.connected) disconnect('Serial stream closed');
}

function onDataLine(line) {
  if (line[0] !== '#') return;
  const p = line.split(/\s+/);
  switch (p[0]) {
    case '#FW':   log('Teensy firmware: ' + p.slice(1).join(' ')); break;
    case '#READY': syncPinUI(false); maybeAutoApply(); maybeAutoApplyOuts(); break;
    case '#START': startCaptureLocal('Teensy acknowledged: capture started'); break;
    case '#STOP':  log('Teensy acknowledged: capture stopped'); stopCaptureLocal(); break;
    case '#PINS':  syncPinUI(false); break;
    case '#OUT':   if (p[1] && FW_OUTS[p[1].toLowerCase()] !== undefined) { FW_OUTS[p[1].toLowerCase()] = parseInt(p[2], 10); } break;
    case '#PULSEW': FW_OUTS.pw = parseInt(p[1], 10) || 100; break;
    case '#PULSE': log('Teensy: ' + p[1] + ' pulsed'); break;
    case '#WIN':   if (!UI.syncing) els.winSel.value = String(CORE.winMs); break;
    case '#ERR':   log('teensy: ' + p.slice(1).join(' '), 'err'); break;
    case '#OVF':   setStatus('\u26A0 ring overflow \u2014 signal too fast', 'warn'); log('Ring buffer overflow: events dropped.', 'warn'); break;
  }
}

/* ---------- capture control ---------- */
function doStart() {
  if (UI.demo) return;
  if (!UI.connected) {
    setStatus('Connect the Teensy first', 'warn');
    log('Click Connect (top-left) and pick the Teensy USB Serial port, then Start.', 'warn');
    return;
  }
  sendCmd('START');
  log('\u2192 START sent');
  setStatus('Waiting for Teensy\u2026', 'ok');
}
function doStop() {
  if (!UI.connected || UI.demo) return;
  sendCmd('STOP');
  log('\u2192 STOP sent');
}

function startCaptureLocal(msg) {
  CORE.evT = []; CORE.evS = []; CORE.gaps = [];
  CORE.lastRaw = null; CORE.wrapAcc = 0;
  CORE.wallStart = performance.now();
  CORE.capturing = true; CORE.overflow = false;
  CORE.lastRate = 0; CORE.rateAt = 0;
  CORE.evCountAt = 0; CORE.evCountT = 0; CORE.eventsPerSec = 0;
  initMeas();
  VIEW.right = 0; VIEW.follow = true;
  VIEW.cursorA = null; VIEW.cursorB = null; VIEW.hoverT = null;
  els.chkFollow.checked = true;
  els.btnStart.disabled = true;
  els.btnStop.disabled = false;
  UI.lastNoDataWarn = 0;
  setStatus('Capturing\u2026', 'ok');
  if (msg) log(msg);
  UI.dirty = true;
}

function stopCaptureLocal() {
  CORE.capturing = false;
  els.btnStart.disabled = !UI.connected || UI.demo;
  els.btnStop.disabled = true;
  setStatus('Stopped', 'ok');
  UI.dirty = true;
}

/* Reset the graph: wipe all data, measurements and view state.  Deliberately
 * does NOT touch the capture state on the Teensy - if it was capturing it
 * keeps capturing, if stopped it stays stopped.  Use Start/Stop for that. */
function doReset() {
  CORE.evT = []; CORE.evS = []; CORE.gaps = [];
  CORE.lastRaw = null; CORE.wrapAcc = 0;
  CORE.overflow = false;
  CORE.lastRate = 0; CORE.rateAt = 0;
  CORE.evCountAt = 0; CORE.evCountT = 0; CORE.eventsPerSec = 0;
  initMeas();
  VIEW.cursorA = null; VIEW.cursorB = null; VIEW.hoverT = null;
  VIEW.follow = true;
  els.chkFollow.checked = true;
  VIEW.span = parseFloat(els.spanSel.value) || 0.5;
  VIEW.right = 0;
  UI.dirty = true;

  if (UI.demo) {
    UI.demoStart = performance.now();
    handleLine('S 0 000');
    log('\u21BB Reset: demo graph cleared and restarted');
    return;
  }
  log('\u21BB Reset: graph cleared (capture state untouched)');
}

/* If we are connected but the Teensy never answered INFO (no #FCPU/#READY),
 * it is almost certainly running old firmware.  Make that visible. */
function checkConnection() {
  if (!UI.connected || CORE.fcpu > 0) return;
  const now = performance.now();
  if (!UI.infoSentAt || now - UI.infoSentAt < 2500) return;
  if (UI.lastConnWarn && now - UI.lastConnWarn < 5000) return;
  UI.lastConnWarn = now;
  setStatus('Connected \u2014 no response', 'warn');
  log('The Teensy is not answering (no #FCPU / #READY received). Check, in order: (1) you picked the port named "Teensy USB Serial" / "usbmodem…" — NOT a "debug console" port, which just streams binary garbage; (2) the new firmware is flashed (pio run -t upload); (3) no other program holds the port.', 'warn');
}

/* If we are "capturing" but no edges ever arrive, tell the user why it is
 * likely (old firmware on the Teensy / nothing connected to the pins). */
function checkNoData() {
  if (!CORE.capturing) return;
  const now = performance.now();
  if (UI.lastNoDataWarn && now - UI.lastNoDataWarn < 4000) return;
  const lastT = CORE.evT.length ? CORE.evT[CORE.evT.length - 1] : null;
  if (lastT === null) {
    if (now - (CORE.wallStart || 0) > 2500) {
      UI.lastNoDataWarn = now;
      setStatus('Capturing \u2014 no data yet', 'warn');
      log('No data received. If this keeps up: the Teensy may still run the old firmware (re-flash with pio run -t upload), or no signal is connected to the selected pins.', 'warn');
    }
  } else if ((now / 1000) - lastT > 2) {
    UI.lastNoDataWarn = now;
    setStatus('Capturing \u2014 no new edges for 2 s', 'warn');
    log('No new edges for 2 s: signals may be idle, or the wrong pins are selected.', 'warn');
  }
}

/* ---------- pin selection ---------- */
function buildPinUI() {
  els.pinList.innerHTML = '';
  for (let ch = 0; ch < NCH; ch++) {
    const row = document.createElement('div');
    row.className = 'pinrow';
    const sw = document.createElement('span');
    sw.className = 'swatch';
    sw.style.background = CH_COLORS[ch];
    const lbl = document.createElement('span');
    lbl.className = 'ch';
    lbl.style.color = CH_COLORS[ch];
    lbl.textContent = 'CH' + (ch + 1);
    const sel = document.createElement('select');
    sel.dataset.ch = ch;
    for (let p = 0; p <= MAX_PIN; p++) {
      const o = document.createElement('option');
      o.value = String(p);
      o.textContent = 'Pin ' + p + (PIN_NAMES[p] ? ' \u2014 ' + PIN_NAMES[p] : '');
      sel.appendChild(o);
    }
    sel.value = String(CORE.pins[ch] !== undefined ? CORE.pins[ch] : DEFAULT_PINS[ch]);
    sel.addEventListener('change', onPinsChanged);
    row.appendChild(sw); row.appendChild(lbl); row.appendChild(sel);
    els.pinList.appendChild(row);
  }
}

function readPinsFromUI() {
  return Array.from(els.pinList.querySelectorAll('select')).map(s => parseInt(s.value, 10));
}

function pinsValid(pins) {
  return pins.length === NCH && new Set(pins).size === NCH &&
         pins.every(p => Number.isInteger(p) && p >= 0 && p <= MAX_PIN);
}

function onPinsChanged() {
  const pins = readPinsFromUI();
  const ok = pinsValid(pins);
  updateApplyBtn();
  els.pinErr.textContent = ok ? '' : '\u26A0 Pins must be 12 distinct values between 0 and 41.';
  if (ok) {
    const changed = pins.join(',') !== CORE.pins.join(',');
    CORE.pins = pins.slice();
    if (changed) {
      // A different mapping invalidates the meaning of captured states.
      CORE.evT = []; CORE.evS = []; CORE.gaps = [];
      CORE.lastRaw = null; CORE.wrapAcc = 0;
      initMeas();
      UI.dirty = true;
    }
    savePins();
    if (els.chkAutoApply.checked && UI.connected) sendPins();
  }
  UI.dirty = true;
}

function updateApplyBtn() {
  els.btnApply.disabled = !(UI.connected && pinsValid(readPinsFromUI()));
}

function sendPins() {
  sendCmd('PINS ' + CORE.pins.join(' '));
  log('\u2192 PINS ' + CORE.pins.join(' '));
}

function syncPinUI(updateSelects) {
  UI.syncing = true;
  const sels = els.pinList.querySelectorAll('select');
  for (let ch = 0; ch < NCH && ch < sels.length; ch++) {
    if (updateSelects || sels[ch].value !== String(CORE.pins[ch])) {
      sels[ch].value = String(CORE.pins[ch]);
    }
  }
  UI.syncing = false;
}

/* Persist the user's pin mapping so it survives reloads. */
function savePins() {
  try { localStorage.setItem('t4la.pins', JSON.stringify(CORE.pins)); } catch (e) {}
}
function loadPins() {
  try {
    const v = JSON.parse(localStorage.getItem('t4la.pins'));
    if (Array.isArray(v) && v.length === NCH && pinsValid(v)) {
      CORE.pins = v;
      return true;
    }
  } catch (e) {}
  return false;
}

/* After (re)connect: if the saved mapping differs from what the Teensy has,
 * push ours so the board matches the dropdowns. */
function maybeAutoApply() {
  if (!UI.connected) return;
  const saved = loadPins();            // true -> CORE.pins now holds the saved set
  if (!saved) return;
  const sels = els.pinList.querySelectorAll('select');
  let differs = false;
  for (let ch = 0; ch < NCH; ch++) {
    if (parseInt(sels[ch].value, 10) !== CORE.pins[ch]) differs = true;
  }
  if (differs) { syncPinUI(true); sendPins(); }
}

/* ---------- output drivers (OE / EN / RST) ---------- */
function buildOutUI() {
  els.outList.innerHTML = '';
  OUT_NAMES.forEach(name => {
    const row = document.createElement('div');
    row.className = 'outrow';
    const dot = document.createElement('span');
    dot.className = 'odot';
    dot.style.background = OUT_COLORS[name];
    const lbl = document.createElement('span');
    lbl.className = 'outlabel';
    lbl.style.color = OUT_COLORS[name];
    lbl.textContent = name;
    const sel = document.createElement('select');
    sel.dataset.out = name;
    const none = document.createElement('option');
    none.value = '-1';
    none.textContent = '\u2014 none \u2014';
    sel.appendChild(none);
    for (let p = 0; p <= MAX_PIN; p++) {
      const o = document.createElement('option');
      o.value = String(p);
      o.textContent = 'Pin ' + p + (PIN_NAMES[p] ? ' \u2014 ' + PIN_NAMES[p] : '');
      sel.appendChild(o);
    }
    sel.value = String(CORE.outs[name.toLowerCase()] !== undefined ? CORE.outs[name.toLowerCase()] : -1);
    sel.addEventListener('change', onOutChanged);
    const btn = document.createElement('button');
    btn.className = 'pulsebtn';
    btn.dataset.out = name;
    btn.textContent = '\u25B6 ' + name;
    btn.title = 'Pulse ' + name + ' HIGH for the configured width';
    btn.addEventListener('click', () => pulseOut(name));
    row.appendChild(dot); row.appendChild(lbl); row.appendChild(sel); row.appendChild(btn);
    els.outList.appendChild(row);
  });
}

function readOutsFromUI() {
  const o = {};
  OUT_NAMES.forEach(name => {
    o[name.toLowerCase()] = parseInt(els.outList.querySelector('select[data-out="' + name + '"]').value, 10);
  });
  o.pw = parseInt(els.pulseMs.value, 10) || 100;
  return o;
}

function outsValid(o) {
  const assigned = OUT_NAMES.map(n => o[n.toLowerCase()]).filter(p => p >= 0);
  return assigned.every(p => Number.isInteger(p) && p >= 0 && p <= MAX_PIN) &&
         new Set(assigned).size === assigned.length;
}

function onOutChanged() {
  const o = readOutsFromUI();
  const ok = outsValid(o);
  updateApplyOutsBtn();
  let msg = '';
  if (!ok) msg = '\u26A0 Outputs must each use a distinct pin between 0 and 41.';
  else {
    // warn (not block) when an output shares a pin with a sampled channel
    const clash = [];
    OUT_NAMES.forEach(n => {
      const p = o[n.toLowerCase()];
      if (p >= 0 && CORE.pins.includes(p)) clash.push(n + '=P' + p);
    });
    if (clash.length) msg = '\u26A0 ' + clash.join(', ') + ' is also a channel input \u2014 the reading will be corrupted.';
  }
  els.outErr.textContent = msg;
  if (ok) {
    const changed = OUT_NAMES.some(n => o[n.toLowerCase()] !== CORE.outs[n.toLowerCase()]) ||
                    o.pw !== CORE.outs.pw;
    CORE.outs = o;
    saveOuts();
    if (changed && els.chkAutoApplyOuts.checked && UI.connected) sendOutsConfig();
  }
}

function updateApplyOutsBtn() {
  els.btnApplyOuts.disabled = !(UI.connected && outsValid(readOutsFromUI()));
}

function sendOutsConfig() {
  OUT_NAMES.forEach(n => {
    const p = CORE.outs[n.toLowerCase()];
    sendCmd('SETPIN ' + n + ' ' + p);
    log('\u2192 SETPIN ' + n + ' ' + p);
  });
  sendCmd('PULSEW ' + CORE.outs.pw);
  log('\u2192 PULSEW ' + CORE.outs.pw + ' ms');
}

function pulseOut(name) {
  const key = name.toLowerCase();
  const pin = CORE.outs[key];
  if (UI.demo) { log('Outputs are not active in demo mode.', 'warn'); return; }
  if (!UI.connected) {
    setStatus('Connect the Teensy first', 'warn');
    log('Connect the Teensy, then pulse ' + name + '.', 'warn');
    return;
  }
  if (pin < 0) {
    setStatus('Assign a pin to ' + name + ' first', 'warn');
    log('Pick a pin for the ' + name + ' output (dropdown) before pulsing.', 'warn');
    return;
  }
  // Make sure the Teensy has this assignment before pulsing (auto-apply may
  // be off); commands are processed in order, so SETPIN then PULSE is safe.
  if (FW_OUTS[key] !== pin) {
    sendCmd('SETPIN ' + name + ' ' + pin);
    FW_OUTS[key] = pin;
  }
  sendCmd('PULSE ' + name);
  log('\u2192 PULSE ' + name + ' (P' + pin + ')');
  // visual feedback for the (local estimate of the) pulse duration
  const btn = els.outList.querySelector('button[data-out="' + name + '"]');
  if (btn) {
    btn.classList.add('pulsing');
    btn.textContent = '\u25A0 ' + name;
    setTimeout(() => {
      btn.classList.remove('pulsing');
      btn.textContent = '\u25B6 ' + name;
    }, Math.max(50, CORE.outs.pw));
  }
}

function syncOutUI() {
  UI.syncing = true;
  OUT_NAMES.forEach(name => {
    const sel = els.outList.querySelector('select[data-out="' + name + '"]');
    if (sel) sel.value = String(CORE.outs[name.toLowerCase()]);
  });
  els.pulseMs.value = String(CORE.outs.pw);
  UI.syncing = false;
  onOutChanged();                       // refresh warnings + CORE.outs
}

function saveOuts() {
  try { localStorage.setItem('t4la.outs', JSON.stringify(CORE.outs)); } catch (e) {}
}
function loadOuts() {
  try {
    const v = JSON.parse(localStorage.getItem('t4la.outs'));
    if (v && typeof v === 'object' && OUT_NAMES.every(n => v[n.toLowerCase()] !== undefined)) {
      CORE.outs = { oe: v.oe, en: v.en, rst: v.rst, pw: Math.min(3000, Math.max(1, v.pw || 100)) };
      return true;
    }
  } catch (e) {}
  return false;
}

/* After (re)connect: if the saved output config differs from what the Teensy
 * reported (FW_OUTS), push ours so the board matches the UI. */
const FW_OUTS = { oe: -1, en: -1, rst: -1, pw: 100 };   // what the Teensy has
function maybeAutoApplyOuts() {
  if (!UI.connected) return;
  const saved = loadOuts();             // true -> CORE.outs now holds the saved set
  if (!saved) return;
  const differs = OUT_NAMES.some(n => FW_OUTS[n.toLowerCase()] !== CORE.outs[n.toLowerCase()]) ||
                  FW_OUTS.pw !== CORE.outs.pw;
  if (differs) { syncOutUI(); sendOutsConfig(); }
}

/* ---------- demo mode (no hardware needed) ---------- */
function toggleDemo() {
  UI.demo = !UI.demo;
  els.btnDemo.textContent = UI.demo ? 'Stop demo' : 'Demo';
  if (UI.demo) {
    if (UI.connected) disconnect();
    CORE.fcpu = 600000000;
    updateDevInfo();
    startCaptureLocal('Demo mode: synthetic 100 Hz, 1 \u00B5s pulses');
    handleLine('S 0 000');
    UI.demoStart = performance.now();
    UI.demoHandle = setInterval(emitDemoTick, 10);
    els.btnStart.disabled = true; els.btnStop.disabled = true;
  } else {
    if (UI.demoHandle) clearInterval(UI.demoHandle);
    UI.demoHandle = null;
    CORE.capturing = false;
    els.btnStart.disabled = true; els.btnStop.disabled = true;
    setStatus('Demo stopped', 'off');
  }
}

function emitDemoTick() {
  const tEnd = (performance.now() - UI.demoStart) / 1000;
  demoEvent(Math.max(0, tEnd - 0.010), tEnd);
  UI.dirty = true;
}

/* ---------- canvas rendering ---------- */
const cv = els.wave;
const ctx = cv.getContext('2d');
const GUTTER = 104, AXIS_H = 30, LANE_H = 26;
let cw = 0, chH = 0, dpr = 1;

function resize() {
  dpr = window.devicePixelRatio || 1;
  cw = els.waveWrap.clientWidth - 2;
  chH = AXIS_H + NCH * LANE_H + 6;
  cv.style.width = cw + 'px';
  cv.style.height = chH + 'px';
  cv.width = Math.max(1, Math.round(cw * dpr));
  cv.height = Math.max(1, Math.round(chH * dpr));
  UI.dirty = true;
}

function draw(now) {
  if (cw === 0) return;
  if (VIEW.follow) {
    let tNow = CORE.evT.length ? CORE.evT[CORE.evT.length - 1] : 0;
    if (CORE.capturing && CORE.wallStart) {
      const wall = (performance.now() - CORE.wallStart) / 1000;
      if (wall > tNow) tNow = wall;
    }
    VIEW.right = Math.max(tNow + 0.0005, 0.000001);
    VIEW.right = Math.max(VIEW.right, VIEW.span);
  }
  const x0 = VIEW.right - VIEW.span, x1 = VIEW.right;
  const plotW = cw - GUTTER;
  const X = (t) => GUTTER + ((t - x0) / VIEW.span) * plotW;

  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, cw, chH);
  ctx.fillStyle = '#0a0e12';
  ctx.fillRect(0, 0, cw, chH);

  drawLanes();
  drawTimeAxis(x0, x1, X);
  drawGaps(x0, x1, X);
  drawWaves(x0, x1, X);
  drawGutter();
  drawCursors(x0, x1, X);
  UI.dirty = false;
}

function drawLanes() {
  for (let i = 0; i < NCH; i++) {
    const y = AXIS_H + i * LANE_H;
    if (i % 2 === 0) {
      ctx.fillStyle = 'rgba(255,255,255,0.025)';
      ctx.fillRect(GUTTER, y, cw - GUTTER, LANE_H);
    }
    ctx.strokeStyle = '#1b222b';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(cw, y);
    ctx.stroke();
  }
  ctx.strokeStyle = '#222b36';
  ctx.beginPath(); ctx.moveTo(0, AXIS_H); ctx.lineTo(cw, AXIS_H); ctx.stroke();
}

function drawTimeAxis(x0, x1, X) {
  const pxPerSec = (cw - GUTTER) / VIEW.span;
  const target = 90 / pxPerSec;                 // desired seconds between ticks
  const mag = Math.pow(10, Math.floor(Math.log10(target)));
  let step = mag;
  for (const m of [1, 2, 5, 10]) {
    if (mag * m >= target) { step = mag * m; break; }
  }
  ctx.fillStyle = '#7d8b99';
  ctx.font = '10.5px ui-monospace, Menlo, monospace';
  ctx.textBaseline = 'top';
  ctx.strokeStyle = 'rgba(255,255,255,0.06)';
  ctx.lineWidth = 1;
  const t0 = Math.ceil(x0 / step) * step;
  for (let t = t0; t <= x1 + step * 0.5; t += step) {
    const x = Math.round(X(t));
    if (x < GUTTER || x > cw) continue;
    ctx.beginPath(); ctx.moveTo(x, AXIS_H); ctx.lineTo(x, chH); ctx.stroke();
    ctx.fillText(fmtTime(t), x + 3, 8);
  }
}

function drawGaps(x0, x1, X) {
  for (const g of CORE.gaps) {
    const a = Math.max(g.t0, x0), b = Math.min(g.t1, x1);
    if (a >= b) continue;
    const xa = X(a), xb = X(b);
    ctx.fillStyle = 'rgba(240,130,60,0.07)';
    ctx.fillRect(xa, AXIS_H, xb - xa, chH - AXIS_H);
    ctx.strokeStyle = 'rgba(240,130,60,0.25)';
    ctx.setLineDash([2, 3]);
    ctx.beginPath(); ctx.moveTo(xa, AXIS_H); ctx.lineTo(xa, chH); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(xb, AXIS_H); ctx.lineTo(xb, chH); ctx.stroke();
    ctx.setLineDash([]);
  }
}

function drawWaves(x0, x1, X) {
  const i0 = lowerBound(CORE.evT, x0);
  const i1 = lowerBound(CORE.evT, x1);
  let stride = 1;
  const cnt = i1 - i0;
  if (cnt > 20000) stride = Math.ceil(cnt / 20000);

  for (let ch = 0; ch < NCH; ch++) {
    const yHi = AXIS_H + ch * LANE_H + 4.5;
    const yLo = AXIS_H + (ch + 1) * LANE_H - 4.5;
    const yOf = (bit) => bit ? yHi : yLo;
    let lvl = -1;
    if (i0 > 0) lvl = (CORE.evS[i0 - 1] >> ch) & 1;

    ctx.strokeStyle = CH_COLORS[ch];
    ctx.lineWidth = 1.6;
    ctx.lineJoin = 'miter';
    ctx.beginPath();

    if (lvl === -1) {
      if (i0 < CORE.evS.length) {
        // unknown from x0 to first event: dashed center line
        const tx = CORE.evT[i0];
        ctx.setLineDash([3, 3]);
        ctx.strokeStyle = 'rgba(125,140,160,0.35)';
        ctx.beginPath();
        ctx.moveTo(X(Math.max(x0, tx - VIEW.span)), (yHi + yLo) / 2);
        ctx.lineTo(X(tx), (yHi + yLo) / 2);
        ctx.stroke();
        ctx.setLineDash([]);
        lvl = (CORE.evS[i0] >> ch) & 1;
        ctx.strokeStyle = CH_COLORS[ch];
        ctx.beginPath();
        ctx.moveTo(X(tx), yOf(lvl));
      } else {
        continue;                       // no data at all
      }
    } else {
      ctx.moveTo(X(x0), yOf(lvl));
    }

    let prevBit = lvl;
    for (let i = i0; i < i1; i += stride) {
      const bit = (CORE.evS[i] >> ch) & 1;
      if (bit === prevBit) continue;
      const xt = X(CORE.evT[i]);
      ctx.lineTo(xt, yOf(prevBit));
      ctx.lineTo(xt, yOf(bit));
      prevBit = bit;
    }
    ctx.lineTo(X(x1), yOf(prevBit));
    ctx.stroke();
  }
}

function drawGutter() {
  ctx.font = '10.5px ui-monospace, Menlo, monospace';
  ctx.textBaseline = 'middle';
  for (let ch = 0; ch < NCH; ch++) {
    const y = AXIS_H + ch * LANE_H + LANE_H / 2;
    ctx.fillStyle = CH_COLORS[ch];
    ctx.fillRect(6, y - 3, 6, 6);
    ctx.fillStyle = '#c9d4de';
    ctx.fillText('CH' + (ch + 1), 18, y);
    ctx.fillStyle = '#6d7b89';
    ctx.fillText('P' + (CORE.pins[ch] !== undefined ? CORE.pins[ch] : '?'), 56, y);
  }
}

function drawCursors(x0, x1, X) {
  const inView = (t) => t !== null && t >= x0 && t <= x1;
  if (inView(VIEW.hoverT)) {
    const x = X(VIEW.hoverT);
    ctx.strokeStyle = 'rgba(255,255,255,0.22)';
    ctx.setLineDash([2, 3]);
    ctx.beginPath(); ctx.moveTo(x, AXIS_H); ctx.lineTo(x, chH); ctx.stroke();
    ctx.setLineDash([]);
  }
  if (inView(VIEW.cursorA)) drawCursor('A', VIEW.cursorA, '#4fc3f7', X);
  if (inView(VIEW.cursorB)) drawCursor('B', VIEW.cursorB, '#ffb74d', X);

  let readout = '';
  if (VIEW.cursorA !== null && VIEW.cursorB !== null) {
    const dt = Math.abs(VIEW.cursorB - VIEW.cursorA);
    readout = '\u0394 ' + fmtTime(dt) + '  \u00B7  f = ' + fmtFreq(1 / dt);
  } else if (VIEW.cursorA !== null) {
    readout = 'A = ' + fmtTime(VIEW.cursorA);
  } else if (VIEW.hoverT !== null) {
    readout = fmtTime(VIEW.hoverT);
    const s = stateAt(VIEW.hoverT);
    if (s !== null) {
      const highs = [];
      for (let i = 0; i < NCH; i++) if ((s >> i) & 1) highs.push('CH' + (i + 1));
      readout += '  \u00B7  ' + (highs.length ? highs.join(' ') : 'all low');
    } else {
      readout += '  \u00B7  state unknown';
    }
  }
  els.cursorRead.textContent = readout;
}

function drawCursor(name, t, color, X) {
  const x = Math.round(X(t));
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.2;
  ctx.beginPath(); ctx.moveTo(x, AXIS_H); ctx.lineTo(x, chH); ctx.stroke();
  ctx.fillStyle = color;
  ctx.font = 'bold 10.5px ui-monospace, Menlo, monospace';
  ctx.textBaseline = 'bottom';
  ctx.fillText(name + ' ' + fmtTime(t), x + 4, AXIS_H - 2);
  ctx.fillRect(x - 3, AXIS_H - 12, 6, 12);
}

/* ---------- interactions ---------- */
function canvasPos(e) {
  const r = cv.getBoundingClientRect();
  return { x: e.clientX - r.left, y: e.clientY - r.top };
}
function timeAtX(x) {
  const plotW = cw - GUTTER;
  const frac = (x - GUTTER) / plotW;
  return VIEW.right - VIEW.span + frac * VIEW.span;
}

cv.addEventListener('wheel', (e) => {
  e.preventDefault();
  const { x } = canvasPos(e);
  const plotW = cw - GUTTER;
  const frac = Math.min(1, Math.max(0, (x - GUTTER) / plotW));
  const k = Math.pow(1.0015, e.deltaY);          // >1 zooms out
  const newSpan = Math.min(120, Math.max(5e-7, VIEW.span * k));
  const tAt = timeAtX(x);
  VIEW.span = newSpan;
  VIEW.right = tAt + (1 - frac) * newSpan;
  UI.dirty = true;
}, { passive: false });

cv.addEventListener('mousedown', (e) => {
  if (e.button !== 0) return;
  UI.dragging = { startX: e.clientX, startRight: VIEW.right, moved: false };
});

window.addEventListener('mousemove', (e) => {
  const r = cv.getBoundingClientRect();
  const inCanvas = e.clientX >= r.left && e.clientX <= r.right && e.clientY >= r.top && e.clientY <= r.bottom;
  VIEW.hoverT = inCanvas ? timeAtX(e.clientX - r.left) : null;
  if (UI.dragging) {
    const dx = e.clientX - UI.dragging.startX;
    if (Math.abs(dx) > 3) UI.dragging.moved = true;
    if (UI.dragging.moved) {
      VIEW.right = UI.dragging.startRight - (dx / (cw - GUTTER)) * VIEW.span;
      VIEW.follow = false;
      els.chkFollow.checked = false;
    }
  }
  UI.dirty = true;
});

window.addEventListener('mouseup', (e) => {
  if (!UI.dragging) return;
  const wasDrag = UI.dragging.moved;
  const startX = UI.dragging.startX;
  UI.dragging = null;
  if (wasDrag) return;
  // click = cursor A (or restart), shift+click = cursor B
  const { x } = canvasPos(e);
  const t = timeAtX(x);
  if (e.shiftKey) {
    VIEW.cursorB = t;
  } else if (VIEW.cursorA === null) {
    VIEW.cursorA = t;
  } else if (VIEW.cursorB === null) {
    VIEW.cursorB = t;
  } else {
    VIEW.cursorA = t;
    VIEW.cursorB = null;
  }
  UI.dirty = true;
});

cv.addEventListener('dblclick', (e) => {
  e.preventDefault();
  const { y } = canvasPos(e);
  const ch = Math.floor((y - AXIS_H) / LANE_H);
  VIEW.cursorA = null; VIEW.cursorB = null;
  if (ch >= 0 && ch < NCH) {
    const m = CORE.meas[ch];
    if (m.lastRise !== null && m.lastFall !== null && m.lastFall > m.lastRise) {
      const w = m.lastFall - m.lastRise;
      VIEW.span = Math.max(w * 8, 5e-7);
      VIEW.right = m.lastFall + VIEW.span * 0.25;
      VIEW.follow = false;
      els.chkFollow.checked = false;
      log('Zoomed to last pulse on CH' + (ch + 1) + ' (width ' + fmtTime(w) + ')');
    } else {
      log('No completed pulse on CH' + (ch + 1) + ' yet');
    }
  }
  UI.dirty = true;
});

/* ---------- measurement table ---------- */
function buildMeasTable() {
  els.measBody.innerHTML = '';
  for (let ch = 0; ch < NCH; ch++) {
    const tr = document.createElement('tr');
    tr.innerHTML =
      '<td class="ch"><span class="dot" style="background:' + CH_COLORS[ch] + '"></span>CH' + (ch + 1) + '</td>' +
      '<td id="mpin' + ch + '"></td>' +
      '<td class="lvl" id="mlvl' + ch + '"></td>' +
      '<td class="big"><a class="pulsezoom" id="mpw' + ch + '" title="double-click the lane also zooms">—</a></td>' +
      '<td id="mper' + ch + '"></td>' +
      '<td id="mfreq' + ch + '"></td>' +
      '<td id="mcnt' + ch + '"></td>';
    tr.querySelector('.pulsezoom').addEventListener('click', () => zoomToPulse(ch));
    els.measBody.appendChild(tr);
  }
}

function zoomToPulse(ch) {
  const m = CORE.meas[ch];
  if (m.lastRise !== null && m.lastFall !== null && m.lastFall > m.lastRise) {
    const w = m.lastFall - m.lastRise;
    VIEW.span = Math.max(w * 8, 5e-7);
    VIEW.right = m.lastFall + VIEW.span * 0.25;
    VIEW.follow = false;
    els.chkFollow.checked = false;
    UI.dirty = true;
  }
}

function updateMeasTable() {
  for (let ch = 0; ch < NCH; ch++) {
    const m = CORE.meas[ch];
    const pin = CORE.pins[ch] !== undefined ? 'P' + CORE.pins[ch] : '—';
    $('mpin' + ch).textContent = pin;
    $('mlvl' + ch).textContent = m.val === null ? '?' : (m.val ? 'H' : 'L');
    const pw = $('mpw' + ch);
    pw.textContent = fmtTime(m.pulseW);
    pw.style.color = m.pulseW !== null ? CH_COLORS[ch] : '';
    $('mper' + ch).textContent = fmtTime(m.period);
    $('mfreq' + ch).textContent = fmtFreq(m.period ? 1 / m.period : null);
    $('mcnt' + ch).textContent = String(m.pulses);
  }
}

/* ---------- CSV export ---------- */
function exportCSV() {
  const x0 = VIEW.right - VIEW.span, x1 = VIEW.right;
  const i0 = lowerBound(CORE.evT, x0), i1 = lowerBound(CORE.evT, x1);
  let csv = '# Teensy 4.1 12-ch logic analyzer export\n';
  csv += '# pins: ' + CORE.pins.join(',') + '\n';
  csv += '# fcpu: ' + CORE.fcpu + ' Hz\n';
  csv += '# events: ' + (i1 - i0) + ' in window ' + fmtTime(VIEW.span) + '\n';
  for (const g of CORE.gaps) {
    if (g.t1 >= x0 && g.t0 <= x1) csv += '# gap ' + g.t0.toFixed(9) + ',' + g.t1.toFixed(9) + '\n';
  }
  const head = ['time_s'];
  for (let i = 0; i < NCH; i++) head.push('ch' + (i + 1) + '_p' + CORE.pins[i]);
  csv += head.join(',') + '\n';
  for (let i = i0; i < i1; i++) {
    const row = [CORE.evT[i].toFixed(9)];
    for (let ch = 0; ch < NCH; ch++) row.push((CORE.evS[i] >> ch) & 1);
    csv += row.join(',') + '\n';
  }
  const blob = new Blob([csv], { type: 'text/csv' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'teensy_la_' + Date.now() + '.csv';
  a.click();
  URL.revokeObjectURL(a.href);
  log('Exported ' + (i1 - i0) + ' events to CSV');
}

/* ---------- stats ---------- */
function updateStats() {
  let s = '';
  if (CORE.fcpu) s += (CORE.fcpu / 1e6) + ' MHz \u00B7 ';
  if (CORE.lastRate && Date.now() - CORE.rateAt < 2000) {
    const rate = CORE.lastRate / (CORE.winMs / 1000);
    s += '\u2248 ' + fmtRate(rate) + ' samples/s \u00B7 ';
  }
  // events per second (over the last second)
  const now = Date.now();
  if (CORE.evCountAt) {
    const dt = (now - CORE.evCountAt) / 1000;
    if (dt > 0.5) {
      CORE.eventsPerSec = (CORE.evT.length - CORE.evCountT) / dt;
      CORE.evCountAt = now; CORE.evCountT = CORE.evT.length;
    }
  } else {
    CORE.evCountAt = now; CORE.evCountT = CORE.evT.length;
  }
  if (CORE.evT.length) s += fmtRate(CORE.eventsPerSec) + ' edges/s \u00B7 ';
  s += CORE.evT.length + ' events';
  if (CORE.gaps.length) s += ' \u00B7 ' + CORE.gaps.length + ' gap(s)';
  if (CORE.overflow) s += ' \u00B7 \u26A0 overflow';
  els.stats.textContent = s;
}

/* ---------- main loop ---------- */
function rafLoop(now) {
  if (now - UI.lastStats > 500) { updateStats(); UI.lastStats = now; }
  if (now - UI.lastTrim > 2000) { trimData(); UI.lastTrim = now; }
  checkConnection();
  checkNoData();
  if (UI.dirty || VIEW.follow || CORE.capturing) draw(now);
  requestAnimationFrame(rafLoop);
}

/* ---------- wiring ---------- */
els.btnConnect.addEventListener('click', connect);
els.btnStart.addEventListener('click', doStart);
els.btnStop.addEventListener('click', doStop);
els.btnReset.addEventListener('click', doReset);
els.btnDemo.addEventListener('click', toggleDemo);
els.btnApply.addEventListener('click', () => { sendPins(); });
els.chkAutoApply.addEventListener('change', () => {
  if (els.chkAutoApply.checked && UI.connected) onPinsChanged();
});
els.winSel.addEventListener('change', () => {
  CORE.winMs = parseInt(els.winSel.value, 10);
  if (UI.connected) sendCmd('WIN ' + CORE.winMs);
});
els.spanSel.addEventListener('change', () => {
  VIEW.span = parseFloat(els.spanSel.value);
  UI.dirty = true;
});
els.chkFollow.addEventListener('change', () => {
  VIEW.follow = els.chkFollow.checked;
  UI.dirty = true;
});
els.btnExport.addEventListener('click', exportCSV);
els.btnApplyOuts.addEventListener('click', () => { sendOutsConfig(); });
els.chkAutoApplyOuts.addEventListener('change', () => {
  if (els.chkAutoApplyOuts.checked && UI.connected) onOutChanged();
});
els.pulseMs.addEventListener('change', () => {
  const ms = Math.min(3000, Math.max(1, parseInt(els.pulseMs.value, 10) || 100));
  els.pulseMs.value = String(ms);
  CORE.outs.pw = ms;
  saveOuts();
  if (UI.connected && els.chkAutoApplyOuts.checked) sendCmd('PULSEW ' + ms);
});
document.querySelectorAll('.presets button').forEach(btn => {
  btn.addEventListener('click', () => {
    const preset = btn.dataset.preset;
    let pins;
    if (preset === 'spread') {
      pins = [];
      for (let ch = 0; ch < NCH; ch++) pins.push((ch * 3) % (MAX_PIN + 1));
      // ensure distinct
      pins = [...new Set(pins)];
      while (pins.length < NCH) { const p = (pins.length * 7 + 5) % (MAX_PIN + 1); if (!pins.includes(p)) pins.push(p); }
    } else {
      const [a, b] = preset.split('-').map(Number);
      pins = [];
      for (let ch = 0; ch < NCH; ch++) pins.push(Math.min(a + ch, b));
    }
    const sels = els.pinList.querySelectorAll('select');
    for (let ch = 0; ch < NCH; ch++) sels[ch].value = String(pins[ch]);
    onPinsChanged();
  });
});
window.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'SELECT' || e.target.tagName === 'INPUT') return;
  if (e.key === ' ') { e.preventDefault(); CORE.capturing ? doStop() : doStart(); }
  else if (e.key === 'f' || e.key === 'F') { els.chkFollow.checked = !els.chkFollow.checked; els.chkFollow.dispatchEvent(new Event('change')); }
  else if (e.key === 'c' || e.key === 'C') { VIEW.cursorA = null; VIEW.cursorB = null; UI.dirty = true; }
  else if (e.key === 'r' || e.key === 'R') { doReset(); }
});
window.addEventListener('resize', resize);

/* ---------- init ---------- */
loadPins();
buildPinUI();
loadOuts();
buildOutUI();
buildMeasTable();
updateApplyBtn();
updateApplyOutsBtn();
resize();
rafLoop(0);
setInterval(updateMeasTable, 150);

} // end UI layer
