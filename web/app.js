'use strict';

/* Robo Racer control app.
 *
 * The app never sends motor values: it sends the state of the buttons and the
 * firmware resolves them into a differential command. That keeps the
 * calibration table in one place (the firmware).
 *
 * Safety behaviour, all of it on purpose:
 *  - a frame is sent every 50 ms while a direction is held,
 *  - releasing a button sends an immediate stop frame,
 *  - losing focus, hiding the page or closing the socket stops the robot,
 *  - the firmware stops on its own if frames stop arriving,
 *  - nothing re-arms automatically after a disconnect.
 */

const KIND = { DRIVE: 1, ARM: 2, DISARM: 3, PING: 4, ACK: 5 };
const FLAG = { ESTOP: 1 };
const BTN = { FWD: 1, BACK: 2, LEFT: 4, RIGHT: 8, STOP: 16 };

const STATE_NAMES = ['BOOT', 'DISARMED', 'ARMED', 'DRIVING', 'FAULT'];
const FAULT_NAMES = [
  'COMMAND TIMEOUT',
  'LINK LOST',
  'BATTERY CRITICAL',
  'DRIVER FAULT',
  'EMERGENCY STOP',
  'INTERNAL FAULT',
];

const DRIVE_PERIOD_MS = 50;   /* 20 Hz while moving */
const POLL_PERIOD_MS = 100;   /* 10 Hz while idle */
const ESTOP_HOLD_MS = 600;

const dot = document.getElementById('dot');
const stateLabel = document.getElementById('state');
const batteryLabel = document.getElementById('battery');
const pad = document.getElementById('pad');
const stopButton = document.getElementById('stop');
const armButton = document.getElementById('arm');
const disarmButton = document.getElementById('disarm');
const ackButton = document.getElementById('ack');
const estopButton = document.getElementById('estop');
const modeButtons = Array.from(document.querySelectorAll('[data-mode]'));
const toastBox = document.getElementById('toast');

let socket = null;
let token = 0;
let seq = 0;
let mode = 1;
let ready = false;
let rebootPending = false;
let connecting = false;
let lastPoll = 0;
let lastUptime = null;
let robotState = 0;
let estopTimer = null;

/** pointerId -> button bit, so several fingers work at once. */
const held = new Map();

/* ------------------------------------------------------------------ frames */

function sendFrame(kind, buttons, flags) {
  if (!ready || !socket || socket.readyState !== WebSocket.OPEN) return;
  const buffer = new ArrayBuffer(12);
  const view = new DataView(buffer);
  view.setUint8(0, 0xa5);
  view.setUint8(1, 1);
  view.setUint8(2, kind);
  view.setUint8(3, buttons & 0x1f);
  view.setUint8(4, mode);
  view.setUint8(5, flags);
  view.setUint16(6, seq, true);
  view.setUint32(8, token, true);
  seq = (seq + 1) & 0xffff;
  socket.send(buffer);
}

function driveMask() {
  let mask = 0;
  for (const bit of held.values()) mask |= bit;
  return mask;
}

function sendNow() {
  sendFrame(KIND.DRIVE, driveMask(), 0);
}

/* ---------------------------------------------------------------- transport */

function connect() {
  if (connecting) return;
  connecting = true;

  const url = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`;
  socket = new WebSocket(url);
  socket.binaryType = 'arraybuffer';

  socket.addEventListener('message', onMessage);

  socket.addEventListener('close', () => {
    connecting = false;
    ready = false;
    token = 0;
    releaseEverything();
    setConnection('bad', 'DISCONNECTED');
    setTimeout(connect, 800);
  });

  socket.addEventListener('error', () => {
    /* the close event does the cleanup */
  });
}

function onMessage(event) {
  if (!(event.data instanceof ArrayBuffer) || event.data.byteLength < 4) return;
  const view = new DataView(event.data);
  if (view.getUint8(0) !== 0xa5) return;

  switch (view.getUint8(2)) {
    case 0x80: /* HELLO */
      if (view.byteLength >= 12) onHello(view);
      break;
    case 0x81: /* STATUS */
      if (view.byteLength >= 16) onStatus(view);
      break;
    case 0x83: /* ERROR */
      toast(`Rejected by robot (code ${view.getUint8(3)})`);
      break;
    default:
      break;
  }
}

function onHello(view) {
  const newToken = view.getUint32(4, true);
  const uptime = view.getUint32(8, true);
  const rebooted = lastUptime !== null && uptime < lastUptime;
  const tokenChanged = token !== 0 && token !== newToken;

  token = newToken;
  lastUptime = uptime;
  connecting = false;
  ready = true;

  if (rebooted || tokenChanged) {
    seq = 0;
    releaseEverything();
    toast('Robot restarted: arm again');
  }
  setConnection('live', 'CONNECTED');
  sendFrame(KIND.PING, 0, 0);
}

function onStatus(view) {
  robotState = view.getUint8(3);
  const fault = view.getUint8(4);
  const flags = view.getUint8(5);
  const millivolts = view.getUint16(6, true);
  const batteryOk = (flags & 0x04) !== 0;

  const running = robotState === 2 || robotState === 3;
  stateLabel.textContent =
    robotState === 4 ? `FAULT: ${FAULT_NAMES[fault] ?? 'UNKNOWN'}` : STATE_NAMES[robotState] ?? 'UNKNOWN';

  dot.className = 'dot';
  if (robotState === 4) dot.classList.add('bad');
  else if (running) dot.classList.add('live');
  else dot.classList.add('warn');

  pad.classList.toggle('stopped', !running);
  ackButton.hidden = robotState !== 4;

  batteryLabel.textContent = millivolts > 0 ? `${(millivolts / 1000).toFixed(1)} V` : '--.- V';
  batteryLabel.className = 'battery';
  if (!batteryOk) batteryLabel.classList.add('critical');
}

function setConnection(level, text) {
  if (!ready) {
    dot.className = 'dot';
    if (level === 'bad') dot.classList.add('bad');
    else if (level === 'warn') dot.classList.add('warn');
  }
  stateLabel.textContent = text;
}

/* ------------------------------------------------------------------ ticking */

function tick() {
  if (!ready) return;
  const mask = driveMask();
  if (mask !== 0) {
    sendNow();
  } else if (performance.now() - lastPoll >= POLL_PERIOD_MS) {
    lastPoll = performance.now();
    sendFrame(KIND.PING, 0, 0);
  }
}

/* ------------------------------------------------------------------- input */

function releaseEverything() {
  held.clear();
  syncPressed();
}

function syncPressed() {
  const mask = driveMask();
  for (const button of document.querySelectorAll('[data-btn]')) {
    button.classList.toggle('on', (mask & Number(button.dataset.btn)) !== 0);
  }
}

function bindHold(button) {
  const bit = Number(button.dataset.btn);

  button.addEventListener('pointerdown', (event) => {
    event.preventDefault();
    button.setPointerCapture(event.pointerId);
    if (bit === BTN.STOP) {
      /* The stop button wins over any direction already held. */
      held.clear();
      syncPressed();
    }
    held.set(event.pointerId, bit);
    button.classList.add('on');
    sendNow();
  });

  const release = (event) => {
    if (!held.delete(event.pointerId)) return;
    syncPressed();
    sendNow();
  };
  button.addEventListener('pointerup', release);
  button.addEventListener('pointercancel', release);
  button.addEventListener('lostpointercapture', release);
}

function bindTap(button, action) {
  button.addEventListener('pointerdown', (event) => {
    event.preventDefault();
    action();
  });
}

/* -------------------------------------------------------------------- init */

for (const button of document.querySelectorAll('[data-btn]')) bindHold(button);

bindTap(armButton, () => {
  sendFrame(KIND.ARM, 0, 0);
  toast('Arming: hold a direction to move');
});

bindTap(disarmButton, () => {
  releaseEverything();
  sendFrame(KIND.DISARM, 0, 0);
});

bindTap(ackButton, () => {
  sendFrame(KIND.ACK, 0, 0);
  toast('Fault cleared: arm again');
});

for (const button of modeButtons) {
  button.addEventListener('pointerdown', (event) => {
    event.preventDefault();
    mode = Number(button.dataset.mode);
    for (const other of modeButtons) other.classList.toggle('on', other === button);
    /* Push the new mode immediately, even when disarmed. */
    sendNow();
  });
}

/* Emergency stop: hold to fire, release early to cancel. */
estopButton.addEventListener('pointerdown', (event) => {
  event.preventDefault();
  estopButton.classList.add('on');
  estopTimer = setTimeout(() => {
    releaseEverything();
    sendFrame(KIND.DRIVE, BTN.STOP, FLAG.ESTOP);
    estopButton.classList.remove('on');
    toast('EMERGENCY STOP SENT');
  }, ESTOP_HOLD_MS);
});

function cancelEstop() {
  if (estopTimer !== null) {
    clearTimeout(estopTimer);
    estopTimer = null;
  }
  estopButton.classList.remove('on');
}
estopButton.addEventListener('pointerup', cancelEstop);
estopButton.addEventListener('pointercancel', cancelEstop);

/* Any loss of focus is treated as "the operator is not watching". */
function panic() {
  releaseEverything();
  if (ready) sendFrame(KIND.DRIVE, BTN.STOP, 0);
}
document.addEventListener('visibilitychange', () => {
  if (document.hidden) panic();
});
window.addEventListener('blur', panic);
window.addEventListener('pagehide', panic);
document.addEventListener('contextmenu', (event) => event.preventDefault());

/* ------------------------------------------------------------------ toast */

let toastTimer = null;
function toast(message) {
  toastBox.textContent = message;
  toastBox.classList.add('show');
  if (toastTimer !== null) clearTimeout(toastTimer);
  toastTimer = setTimeout(() => toastBox.classList.remove('show'), 2200);
}

setInterval(tick, DRIVE_PERIOD_MS);
connect();
