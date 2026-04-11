import { buildPixelCommand, buildRgb332RowFrame } from './protocol.js';

const BLE_SERVICE_UUID = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
const BLE_RX_UUID = '6e400002-b5a3-f393-e0a9-e50e24dcca9e';
const BLE_TX_UUID = '6e400003-b5a3-f393-e0a9-e50e24dcca9e';
const PANEL_WIDTH = 78;

const palette = [
  { name: 'Off', rgb: [0, 0, 0], hex: '#08111f' },
  { name: 'White', rgb: [255, 255, 255], hex: '#ffffff' },
  { name: 'Red', rgb: [255, 0, 0], hex: '#ef4444' },
  { name: 'Green', rgb: [0, 255, 0], hex: '#22c55e' },
  { name: 'Blue', rgb: [0, 0, 255], hex: '#3b82f6' },
  { name: 'Yellow', rgb: [255, 255, 0], hex: '#facc15' },
  { name: 'Magenta', rgb: [255, 0, 255], hex: '#f472b6' },
  { name: 'Cyan', rgb: [0, 255, 255], hex: '#67e8f9' },
];

const state = {
  device: null,
  server: null,
  rxCharacteristic: null,
  txCharacteristic: null,
  textBuffer: '',
  row: Array.from({ length: PANEL_WIDTH }, () => 0),
  pixelColorIndex: 2,
  canvasData: Array.from({ length: PANEL_WIDTH * PANEL_WIDTH }, () => 0), // color indexes
  isSending: false,
};

const els = {
  status: document.querySelector('#connection-status'),
  connectBtn: document.querySelector('#connect-btn'),
  disconnectBtn: document.querySelector('#disconnect-btn'),
  clearLogBtn: document.querySelector('#clear-log-btn'),
  pingBtn: document.querySelector('#ping-btn'),
  clearBtn: document.querySelector('#clear-btn'),
  brightnessBtn: document.querySelector('#brightness-btn'),
  brightnessInput: document.querySelector('#brightness-input'),
  testRgbBtn: document.querySelector('#test-rgb-btn'),
  testGrayscaleBtn: document.querySelector('#test-grayscale-btn'),
  testBorderBtn: document.querySelector('#test-border-btn'),
  testAlignmentBtn: document.querySelector('#test-alignment-btn'),
  testFullWhiteBtn: document.querySelector('#test-full-white-btn'),
  pixelXInput: document.querySelector('#pixel-x-input'),
  pixelYInput: document.querySelector('#pixel-y-input'),
  sendPixelBtn: document.querySelector('#send-pixel-btn'),
  fillCanvasBtn: document.querySelector('#fill-canvas-btn'),
  pixelCanvas: document.querySelector('#pixel-canvas'),
  canvasInfo: document.querySelector('#canvas-info'),
  pixelPreview: document.querySelector('#pixel-preview'),
  rowIndexInput: document.querySelector('#row-index-input'),
  sendRowBtn: document.querySelector('#send-row-btn'),
  rowStrip: document.querySelector('#row-strip'),
  legend: document.querySelector('#legend'),
  log: document.querySelector('#log'),
  presetButtons: Array.from(document.querySelectorAll('.preset-btn')),
};

const ctx = els.pixelCanvas.getContext('2d');

function logLine(message) {
  const line = `[${new Date().toLocaleTimeString()}] ${message}`;
  els.log.textContent = els.log.textContent ? `${els.log.textContent}\n${line}` : line;
  els.log.scrollTop = els.log.scrollHeight;
}

function setStatus(text, connected = false) {
  els.status.textContent = text;
  els.status.dataset.state = connected ? 'connected' : 'idle';
  els.disconnectBtn.disabled = !connected;
  els.pingBtn.disabled = !connected;
  els.clearBtn.disabled = !connected;
  els.brightnessBtn.disabled = !connected;
  els.testRgbBtn.disabled = !connected;
  els.testGrayscaleBtn.disabled = !connected;
  els.testBorderBtn.disabled = !connected;
  els.testAlignmentBtn.disabled = !connected;
  els.testFullWhiteBtn.disabled = !connected;
  els.sendPixelBtn.disabled = !connected;
  els.fillCanvasBtn.disabled = !connected;
  els.sendRowBtn.disabled = !connected;
  els.connectBtn.disabled = connected;
}

function renderLegend() {
  els.legend.innerHTML = '';
  palette.forEach((entry, index) => {
    const item = document.createElement('div');
    item.className = 'legend-item';
    item.style.display = 'inline-flex';
    item.style.alignItems = 'center';
    item.style.gap = '8px';
    item.style.marginRight = '12px';
    item.style.cursor = 'pointer';
    item.innerHTML = `<span class="swatch" style="display:inline-block;width:14px;height:14px;border-radius:4px;border:1px solid rgba(255,255,255,0.2);background:${entry.hex}"></span><span style="font-size:12px;color:var(--muted)">${index}: ${entry.name}</span>`;
    item.addEventListener('click', () => {
      state.pixelColorIndex = index;
      renderPixelPreview();
    });
    els.legend.appendChild(item);
  });
}

function renderPixelPreview() {
  const entry = palette[state.pixelColorIndex] || palette[0];
  els.pixelPreview.style.background = entry.hex;
  els.pixelPreview.style.color = state.pixelColorIndex === 0 ? '#e7efff' : '#08111f';
  els.pixelPreview.textContent = `${state.pixelColorIndex}: ${entry.name}`;
}

function updateCanvas() {
  const imageData = ctx.createImageData(PANEL_WIDTH, PANEL_WIDTH);
  for (let i = 0; i < state.canvasData.length; i++) {
    const colorIndex = state.canvasData[i];
    const rgb = palette[colorIndex].rgb;
    const offset = i * 4;
    imageData.data[offset] = rgb[0];
    imageData.data[offset + 1] = rgb[1];
    imageData.data[offset + 2] = rgb[2];
    imageData.data[offset + 3] = 255;
  }
  ctx.putImageData(imageData, 0, 0);
}

function setCanvasPixel(x, y, colorIndex, send = false) {
  if (x < 0 || x >= PANEL_WIDTH || y < 0 || y >= PANEL_WIDTH) return;
  state.canvasData[y * PANEL_WIDTH + x] = colorIndex;
  updateCanvas();
  if (send) {
    writeTextCommand(buildPixelCommand(x, y, palette[colorIndex].rgb)).catch((e) => logLine(`ERR ${e.message}`));
  }
}

async function sendFullCanvas() {
  if (state.isSending) {
    logLine('Already sending, please wait...');
    return;
  }
  state.isSending = true;
  logLine('Sending full canvas...');
  try {
    for (let y = 0; y < PANEL_WIDTH; y++) {
      const rowPixels = [];
      for (let x = 0; x < PANEL_WIDTH; x++) {
        rowPixels.push(palette[state.canvasData[y * PANEL_WIDTH + x]].rgb);
      }
      const frame = buildRgb332RowFrame(y, rowPixels);
      await writeBinaryFrame(frame, `ROW ${y}`);
      await new Promise((r) => setTimeout(r, 15)); // Slow down for BLE stability
    }
    logLine('Full canvas sent');
  } catch (e) {
    logLine(`ERR sending canvas: ${e.message}`);
  } finally {
    state.isSending = false;
  }
}

function renderRow() {
  els.rowStrip.innerHTML = '';
  state.row.forEach((colorIndex, idx) => {
    const button = document.createElement('button');
    button.type = 'button';
    button.className = 'pixel';
    button.style.background = palette[colorIndex].hex;
    button.title = `Pixel ${idx}: ${palette[colorIndex].name}`;
    button.addEventListener('click', () => {
      state.row[idx] = (state.row[idx] + 1) % palette.length;
      renderRow();
    });
    els.rowStrip.appendChild(button);
  });
}

function applyPreset(name) {
  if (name === 'blank') {
    state.row.fill(0);
  } else if (name === 'white') {
    state.row.fill(1);
  } else if (name === 'red') {
    state.row.fill(2);
  } else if (name === 'green') {
    state.row.fill(3);
  } else if (name === 'blue') {
    state.row.fill(4);
  } else if (name === 'checker') {
    state.row = state.row.map((_, idx) => (idx % 2 === 0 ? 1 : 0));
  } else if (name === 'gradient') {
    state.row = state.row.map((_, idx) => (idx % (palette.length - 1)) + 1);
  }
  renderRow();
}

// Test Pattern Logic
async function testRGBBars() {
  state.canvasData.fill(0);
  const barWidth = Math.floor(PANEL_WIDTH / 3);
  for (let y = 0; y < PANEL_WIDTH; y++) {
    for (let x = 0; x < PANEL_WIDTH; x++) {
      if (x < barWidth) state.canvasData[y * PANEL_WIDTH + x] = 2; // Red
      else if (x < barWidth * 2) state.canvasData[y * PANEL_WIDTH + x] = 3; // Green
      else state.canvasData[y * PANEL_WIDTH + x] = 4; // Blue
    }
  }
  updateCanvas();
  await sendFullCanvas();
}

async function testGrayscale() {
  // We only have 8 colors in the fixed palette, so this is limited
  // But we can simulate it with patterns or just show the 8 colors
  state.canvasData.fill(0);
  const step = Math.floor(PANEL_WIDTH / palette.length);
  for (let y = 0; y < PANEL_WIDTH; y++) {
    for (let x = 0; x < PANEL_WIDTH; x++) {
      const idx = Math.min(palette.length - 1, Math.floor(x / step));
      state.canvasData[y * PANEL_WIDTH + x] = idx;
    }
  }
  updateCanvas();
  await sendFullCanvas();
}

async function testBorder() {
  state.canvasData.fill(0);
  for (let i = 0; i < PANEL_WIDTH; i++) {
    state.canvasData[i] = 1; // Top
    state.canvasData[(PANEL_WIDTH - 1) * PANEL_WIDTH + i] = 1; // Bottom
    state.canvasData[i * PANEL_WIDTH] = 1; // Left
    state.canvasData[i * PANEL_WIDTH + (PANEL_WIDTH - 1)] = 1; // Right
  }
  updateCanvas();
  await sendFullCanvas();
}

async function testAlignment() {
  state.canvasData.fill(0);
  // Vertical lines at segment boundaries (26, 52)
  for (let y = 0; y < PANEL_WIDTH; y++) {
    state.canvasData[y * PANEL_WIDTH + 25] = 2; // Red
    state.canvasData[y * PANEL_WIDTH + 26] = 3; // Green
    state.canvasData[y * PANEL_WIDTH + 51] = 2; // Red
    state.canvasData[y * PANEL_WIDTH + 52] = 3; // Green
  }
  // Horizontal line through the middle
  const midY = Math.floor(PANEL_WIDTH / 2);
  for (let x = 0; x < PANEL_WIDTH; x++) {
    state.canvasData[midY * PANEL_WIDTH + x] = 5; // Yellow
  }
  updateCanvas();
  await sendFullCanvas();
}

async function testFullWhite() {
  state.canvasData.fill(1);
  updateCanvas();
  await sendFullCanvas();
}

async function writeTextCommand(command) {
  if (!state.rxCharacteristic) {
    throw new Error('BLE RX characteristic unavailable');
  }
  logLine(`TX ${command}`);
  const bytes = new TextEncoder().encode(`${command};\n`);
  await state.rxCharacteristic.writeValueWithResponse(bytes);
}

async function writeBinaryFrame(frame, label) {
  if (!state.rxCharacteristic) {
    throw new Error('BLE RX characteristic unavailable');
  }
  logLine(`TX ${label} bytes=${frame.length}`);
  await state.rxCharacteristic.writeValueWithResponse(frame);
}

function onNotification(event) {
  const characteristic = event.target;
  const value = characteristic?.value;
  if (!value) {
    return;
  }

  const text = new TextDecoder().decode(value);
  state.textBuffer += text;

  let idx = state.textBuffer.indexOf('\n');
  while (idx >= 0) {
    const line = state.textBuffer.slice(0, idx).replace(/\r/g, '').trim();
    state.textBuffer = state.textBuffer.slice(idx + 1);
    if (line) {
      logLine(`RX ${line}`);
    }
    idx = state.textBuffer.indexOf('\n');
  }
}

async function connectBle() {
  if (!navigator.bluetooth?.requestDevice) {
    throw new Error('Web Bluetooth is not available in this browser');
  }

  const device = await navigator.bluetooth.requestDevice({
    filters: [{ services: [BLE_SERVICE_UUID] }],
    optionalServices: [BLE_SERVICE_UUID],
  });

  const server = await device.gatt?.connect();
  if (!server) {
    throw new Error('BLE connect failed');
  }

  const service = await server.getPrimaryService(BLE_SERVICE_UUID);
  const rxCharacteristic = await service.getCharacteristic(BLE_RX_UUID);
  const txCharacteristic = await service.getCharacteristic(BLE_TX_UUID);

  state.device = device;
  state.server = server;
  state.rxCharacteristic = rxCharacteristic;
  state.txCharacteristic = txCharacteristic;
  state.textBuffer = '';

  await txCharacteristic.startNotifications();
  txCharacteristic.addEventListener('characteristicvaluechanged', onNotification);

  device.addEventListener('gattserverdisconnected', handleDisconnect);
  setStatus(`Connected: ${device.name || 'Unnamed device'}`, true);
  logLine(`Connected to ${device.name || 'Unnamed device'}`);
}

function handleDisconnect() {
  if (state.txCharacteristic) {
    state.txCharacteristic.removeEventListener('characteristicvaluechanged', onNotification);
  }
  state.server = null;
  state.rxCharacteristic = null;
  state.txCharacteristic = null;
  setStatus('Disconnected', false);
  logLine('BLE disconnected');
}

async function disconnectBle() {
  if (state.server?.connected) {
    state.server.disconnect();
  } else {
    handleDisconnect();
  }
}

els.connectBtn.addEventListener('click', async () => {
  try {
    await connectBle();
  } catch (error) {
    logLine(`ERR ${error instanceof Error ? error.message : String(error)}`);
    setStatus('Disconnected', false);
  }
});

els.disconnectBtn.addEventListener('click', async () => {
  await disconnectBle();
});

els.clearLogBtn.addEventListener('click', () => {
  els.log.textContent = '';
});

els.pingBtn.addEventListener('click', async () => {
  try {
    await writeTextCommand('PING');
  } catch (error) {
    logLine(`ERR ${error instanceof Error ? error.message : String(error)}`);
  }
});

els.clearBtn.addEventListener('click', async () => {
  try {
    await writeTextCommand('CLEAR');
  } catch (error) {
    logLine(`ERR ${error instanceof Error ? error.message : String(error)}`);
  }
});

els.brightnessBtn.addEventListener('click', async () => {
  try {
    const level = Number.parseInt(els.brightnessInput.value, 10);
    if (!Number.isFinite(level) || level < 0 || level > 4) {
      throw new Error('Brightness must be an integer between 0 and 4');
    }
    await writeTextCommand(`BRIGHTNESS ${level}`);
  } catch (error) {
    logLine(`ERR ${error instanceof Error ? error.message : String(error)}`);
  }
});

els.sendPixelBtn.addEventListener('click', async () => {
  try {
    const x = Number.parseInt(els.pixelXInput.value, 10);
    const y = Number.parseInt(els.pixelYInput.value, 10);
    if (!Number.isFinite(x) || x < 0 || x > 77) {
      throw new Error('Pixel X must be between 0 and 77');
    }
    if (!Number.isFinite(y) || y < 0 || y > 77) {
      throw new Error('Pixel Y must be between 0 and 77');
    }

    setCanvasPixel(x, y, state.pixelColorIndex, true);
  } catch (error) {
    logLine(`ERR ${error instanceof Error ? error.message : String(error)}`);
  }
});

els.fillCanvasBtn.addEventListener('click', async () => {
  try {
    state.canvasData.fill(state.pixelColorIndex);
    updateCanvas();
    await sendFullCanvas();
  } catch (error) {
    logLine(`ERR ${error.message}`);
  }
});

els.testRgbBtn.addEventListener('click', testRGBBars);
els.testGrayscaleBtn.addEventListener('click', testGrayscale);
els.testBorderBtn.addEventListener('click', testBorder);
els.testAlignmentBtn.addEventListener('click', testAlignment);
els.testFullWhiteBtn.addEventListener('click', testFullWhite);

els.pixelCanvas.addEventListener('mousemove', (e) => {
  const rect = els.pixelCanvas.getBoundingClientRect();
  const x = Math.floor(((e.clientX - rect.left) / rect.width) * PANEL_WIDTH);
  const y = Math.floor(((e.clientY - rect.top) / rect.height) * PANEL_WIDTH);
  if (x >= 0 && x < PANEL_WIDTH && y >= 0 && y < PANEL_WIDTH) {
    els.canvasInfo.textContent = `X: ${x}, Y: ${y}`;
    if (e.buttons === 1) {
      setCanvasPixel(x, y, state.pixelColorIndex, false);
    }
  }
});

els.pixelCanvas.addEventListener('mousedown', (e) => {
  const rect = els.pixelCanvas.getBoundingClientRect();
  const x = Math.floor(((e.clientX - rect.left) / rect.width) * PANEL_WIDTH);
  const y = Math.floor(((e.clientY - rect.top) / rect.height) * PANEL_WIDTH);
  if (x >= 0 && x < PANEL_WIDTH && y >= 0 && y < PANEL_WIDTH) {
    setCanvasPixel(x, y, state.pixelColorIndex, true);
  }
});

els.sendRowBtn.addEventListener('click', async () => {
  try {
    const rowIndex = Number.parseInt(els.rowIndexInput.value, 10);
    if (!Number.isFinite(rowIndex) || rowIndex < 0 || rowIndex > 77) {
      throw new Error('Row index must be between 0 and 77');
    }
    const pixels = state.row.map((colorIndex) => palette[colorIndex].rgb);
    const frame = buildRgb332RowFrame(rowIndex, pixels);
    await writeBinaryFrame(frame, `RGB332_ROW row=${rowIndex}`);
    
    // Also update canvas for consistency
    for (let x = 0; x < PANEL_WIDTH; x++) {
      state.canvasData[rowIndex * PANEL_WIDTH + x] = state.row[x];
    }
    updateCanvas();
  } catch (error) {
    logLine(`ERR ${error instanceof Error ? error.message : String(error)}`);
  }
});

els.presetButtons.forEach((button) => {
  button.addEventListener('click', () => {
    applyPreset(button.dataset.preset || 'blank');
  });
});

renderLegend();
renderRow();
renderPixelPreview();
updateCanvas();
setStatus('Disconnected', false);
logLine('Ready. Start a local static server and connect in Chrome.');
