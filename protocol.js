export const BINARY_SYNC_1 = 0xaa;
export const BINARY_SYNC_2 = 0x55;
export const BINARY_FRAME_TYPE_RGB332_ROW = 0x02;

export function packRgb332([r, g, b]) {
  const r3 = Math.max(0, Math.min(7, Math.round((r / 255) * 7)));
  const g3 = Math.max(0, Math.min(7, Math.round((g / 255) * 7)));
  const b2 = Math.max(0, Math.min(3, Math.round((b / 255) * 3)));
  return ((r3 & 0x07) << 5) | ((g3 & 0x07) << 2) | (b2 & 0x03);
}

export function computeChecksum(type, payload) {
  let sum = 0;
  const len = payload.length;
  sum = (sum + type) & 0xffff;
  sum = (sum + (len & 0xff)) & 0xffff;
  sum = (sum + ((len >> 8) & 0xff)) & 0xffff;
  for (const byte of payload) {
    sum = (sum + byte) & 0xffff;
  }
  return sum;
}

export function buildRgb332RowFrame(rowIndex, pixels) {
  const payload = new Uint8Array(pixels.length + 1);
  payload[0] = rowIndex & 0xff;

  for (let i = 0; i < pixels.length; i += 1) {
    payload[i + 1] = packRgb332(pixels[i]);
  }

  const checksum = computeChecksum(BINARY_FRAME_TYPE_RGB332_ROW, payload);
  const frame = new Uint8Array(2 + 1 + 2 + payload.length + 2);
  let offset = 0;
  frame[offset++] = BINARY_SYNC_1;
  frame[offset++] = BINARY_SYNC_2;
  frame[offset++] = BINARY_FRAME_TYPE_RGB332_ROW;
  frame[offset++] = payload.length & 0xff;
  frame[offset++] = (payload.length >> 8) & 0xff;
  frame.set(payload, offset);
  offset += payload.length;
  frame[offset++] = checksum & 0xff;
  frame[offset++] = (checksum >> 8) & 0xff;
  return frame;
}

export function buildPixelCommand(x, y, [r, g, b]) {
  return `PIXEL ${x} ${y} ${r} ${g} ${b}`;
}
