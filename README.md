# ELD78x78Minimal

Minimal Arduino project for a fixed `78x78` ESP32-S3 HUB75 panel with BLE transport.

## Target

- Board family: ESP32-S3
- Panel size: fixed `78 x 78`
- HUB75 chain count: `1`
- BLE device name: `ESP32-LED-Panel`

## Included Protocol

Text commands:

- `PING`
- `CLEAR`
- `BRIGHTNESS <0-4>`
- `COLOR <r g b>`

Binary frame types:

- `0x01` mask frame
- `0x02` RGB332 row frame
- `0x03` RGB332 full frame

Frame format:

- sync: `0xAA 0x55`
- type: `1 byte`
- payload length: `2 bytes`, little-endian
- payload: variable
- checksum: `2 bytes`, little-endian sum of type, length bytes, and payload bytes

Responses:

- `PONG`
- `BIN OK`
- `BIN ERR len`
- `BIN ERR checksum`
- `BIN ERR type`

## BLE Debug Page

The project root contains a powerful BLE-based testing interface:

- `index.html`
- `app.js`

Features for precise testing:

- **Interactive 78x78 Canvas**: Click or drag to draw pixels directly. Hover to see coordinates.
- **Test Patterns**:
  - **RGB Bars**: Test red, green, and blue primary colors.
  - **Grayscale**: Test color linearity across the 8-color palette.
  - **Border**: Draw a 1-pixel white border around the screen to check for cutoff.
  - **Alignment (26S)**: Draws vertical lines at the 26th and 52nd columns to verify the ICN2053 26S offset compensation.
  - **Full White**: Test all pixels at once (useful for checking power draw).
- **Row Buffer Editor**: Test individual row addressing and patterns.
- **Color Palette**: 8 fixed colors for quick debugging.

Use a local static server (like `python3 -m http.server`) and open in a Web Bluetooth compatible browser (Chrome/Edge).

## Project Layout

This project keeps the minimum local driver sources needed by the sketch:

- `ELD78x78Minimal.ino`
- `ESP32-HUB75-MatrixPanel-I2S-DMA.*`
- `ESP32-HUB75-MatrixPanel-leddrivers.cpp`
- `Adafruit_GFX.*`
- `gfxfont.h`
- `glcdfont.c`
- `gdma_lcd_parallel16.cpp`
- `platforms/`
- `index.html`
- `app.js`

## Notes

- The panel mapping keeps the existing `MAP_SWAP_XY = true` behavior from the source project.
- The sketch is intentionally single-purpose and does not implement multi-size panel support.
- The debug page is for local bring-up only, not production use.
