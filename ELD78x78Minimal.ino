#include "ESP32-HUB75-MatrixPanel-I2S-DMA.h"
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define R1_PIN 2
#define G1_PIN 1
#define B1_PIN 3
#define R2_PIN 6
#define G2_PIN 5
#define B2_PIN 4
#define A_PIN 7
#define B_PIN 8
#define C_PIN 9
#define D_PIN -1
#define E_PIN -1
#define LAT_PIN 11
#define OE_PIN 12
#define CLK_PIN 13

#define PANEL_RES_X 78
#define PANEL_RES_Y 78
#define PANEL_CHAIN 1

// 78x78 26S panels with 45 DP5125F chips (15 per color = 240 channels)
// Since the logical height is 52, the HUB75 library sends data out over BOTH R1G1B1 and R2G2B2.
// R1 drives rows 0-25. R2 drives rows 26-51.
// To use all 3 blocks (78 physical rows) over BOTH R1 and R2, we need:
// 234 total pixels per scan.
// Divided across R1 and R2 -> 117 pixels per scan line per RGB group.
// So LOGICAL_WIDTH = 117. LOGICAL_HEIGHT = 52.
#define LOGICAL_WIDTH  117
#define LOGICAL_HEIGHT 52

// Segment swap: Physical chain is Seg1 -> Seg3 -> Seg2
#define ENABLE_SEGMENT_SWAP false

// ICN2053 26S panel Y-offset compensation
#define PANEL_Y_OFFSET_COMPENSATION_ENABLED false
#define PANEL_Y_OFFSET_COMPENSATION_VALUE 2

static const size_t SERIAL_LINE_BUFFER_SIZE = 160;
static const uint8_t BINARY_SYNC_1 = 0xAA;
static const uint8_t BINARY_SYNC_2 = 0x55;
static const uint8_t BINARY_FRAME_TYPE_MASK = 0x01;
static const uint8_t BINARY_FRAME_TYPE_RGB332_ROW = 0x02;
static const uint8_t BINARY_FRAME_TYPE_RGB332_FULL = 0x03;
static const uint8_t BRIGHTNESS_MAX_LEVEL = 4;
static const uint16_t MASK_SEGMENTS_PER_ROW = (LOGICAL_WIDTH + 31) / 32;
static const uint16_t BINARY_PAYLOAD_SIZE_MASK = ((PANEL_RES_X + 31) / 32) * 4 * PANEL_RES_Y;
static const uint16_t BINARY_PAYLOAD_SIZE_RGB332_ROW = PANEL_RES_X + 1;
static const uint16_t BINARY_PAYLOAD_SIZE_RGB332_FULL = PANEL_RES_X * PANEL_RES_Y;
static const uint16_t BINARY_PAYLOAD_SIZE_MAX = BINARY_PAYLOAD_SIZE_RGB332_FULL;
static const HUB75_I2S_CFG::clk_speed MATRIX_I2S_SPEED = HUB75_I2S_CFG::HZ_8M;
static const bool MATRIX_CLKPHASE = true;
static const uint8_t MATRIX_LAT_BLANKING = 6;
static const uint8_t MATRIX_MIN_REFRESH_RATE = 120; // 适当降低最小刷新率以换取更好的显示质量
static const uint8_t MATRIX_COLOR_DEPTH_BITS = 6;
static const uint8_t MATRIX_PANEL_BRIGHTNESS = 90;
static const char *BLE_DEVICE_NAME = "ESP32-LED-Panel";
static const char *BLE_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *BLE_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *BLE_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
static const size_t BLE_TX_MAX_NOTIFY_BYTES = 20;
static const bool MAP_SWAP_XY = false;
static const bool MAP_FLIP_X = false;
static const bool MAP_FLIP_Y = false;

class VirtualMatrixPanel78x78 : public MatrixPanel_I2S_DMA {
public:
  VirtualMatrixPanel78x78(const HUB75_I2S_CFG &opts) : MatrixPanel_I2S_DMA(opts) {}
  
  virtual void drawPixel(int16_t x, int16_t y, uint16_t color) override {
    // Map physical coordinates (0-77, 0-77) to logical coordinates
    int mappedX = x;
    int mappedY = y;
    if (!mapPointVirtual(x, y, mappedX, mappedY)) {
      return;
    }
    // Call parent drawPixel which will handle color conversion and DMA update
    MatrixPanel_I2S_DMA::drawPixel(mappedX, mappedY, color);
  }
  
  void drawPixelRGB888(int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b) {
    int mappedX = x;
    int mappedY = y;
    if (!mapPointVirtual(x, y, mappedX, mappedY)) {
      return;
    }
    MatrixPanel_I2S_DMA::drawPixelRGB888(mappedX, mappedY, r, g, b);
  }
  
private:
  bool mapPointVirtual(int x, int y, int &mappedX, int &mappedY) {
    if (x < 0 || x >= PANEL_RES_X || y < 0 || y >= PANEL_RES_Y) {
      return false;
    }

    // Apply mapping transformations (swap XY, flip X, flip Y) on PHYSICAL coordinates
    int px = x;
    int py = y;

    if (MAP_SWAP_XY) {
      px = y;
      py = x;
    }
    if (MAP_FLIP_X) {
      px = (PANEL_RES_X - 1) - px;
    }
    if (MAP_FLIP_Y) {
      py = (PANEL_RES_Y - 1) - py;
    }

    // Physical to logical mapping for 78x78 26S panel
    // 3 blocks of 78x26 pixels.
    // The panel has an R1G1B1 interface AND an R2G2B2 interface.
    // R1 drives the first 117 columns (logical).
    // R2 drives the second 117 columns (logical).
    // Let's reconstruct the 3 physical blocks into 2 logical blocks:
    // Block 1 (Top 26 rows): Driven by R1
    // Block 2 (Middle 26 rows): Driven by R2
    // Block 3 (Bottom 26 rows): Folded in half. Left half to R1, Right half to R2.
    
    int lx = 0;
    int ly = py; // Logical Y is 0-51 (26 * 2)

    if (py < 26) {
      // Top 26 rows -> R1 (ly 0-25)
      // Block 1 (0-77) + Offset 39 (to leave room for Block 3 left half)
      lx = px + 39;
      ly = py;
    } else if (py < 52) {
      // Middle 26 rows -> R2 (ly 26-51)
      // Block 2 (0-77) + Offset 39
      lx = px + 39; 
      ly = py;
    } else {
      // Bottom 26 rows (52-77)
      // Split into two halves
      if (px < 39) {
        // Left half goes to R1 (ly 0-25)
        lx = px;
        ly = py - 52; 
      } else {
        // Right half goes to R2 (ly 26-51)
        lx = px - 39;
        ly = (py - 52) + 26;
      }
    }

    mappedX = lx;
    mappedY = ly;

    // Apply row offset compensation if needed
    // if (PANEL_Y_OFFSET_COMPENSATION_ENABLED) {
    //   mappedY += PANEL_Y_OFFSET_COMPENSATION_VALUE;
    // }

    // Final bounds check on logical coordinates
    if (mappedX < 0 || mappedX >= LOGICAL_WIDTH || mappedY < 0 || mappedY >= LOGICAL_HEIGHT) {
      return false;
    }
    
    return true;
  }
};

VirtualMatrixPanel78x78 *dma_display = nullptr;
BLECharacteristic *bleTxCharacteristic = nullptr;

HUB75_I2S_CFG mxconfig(
  LOGICAL_WIDTH,
  LOGICAL_HEIGHT,
  PANEL_CHAIN
);

enum InputSource {
  SRC_SERIAL = 0,
  SRC_BLE = 1,
  SRC_COUNT = 2
};

enum BinaryParseState : uint8_t {
  BIN_WAIT_SYNC1 = 0,
  BIN_WAIT_SYNC2,
  BIN_READ_TYPE,
  BIN_READ_LEN_L,
  BIN_READ_LEN_H,
  BIN_READ_PAYLOAD,
  BIN_READ_CHK_L,
  BIN_READ_CHK_H,
};

struct BinaryParserContext {
  BinaryParseState state;
  uint8_t type;
  uint16_t len;
  uint16_t index;
  uint16_t chkRead;
  uint8_t payload[BINARY_PAYLOAD_SIZE_MAX];
};

struct LineParserContext {
  char buffer[SERIAL_LINE_BUFFER_SIZE];
  size_t length;
};

static BinaryParserContext binaryContexts[SRC_COUNT];
static LineParserContext lineContexts[SRC_COUNT];
static uint16_t drawColor = 0;
static uint8_t drawBrightness = BRIGHTNESS_MAX_LEVEL;
static uint8_t colorR333 = 7;
static uint8_t colorG333 = 7;
static uint8_t colorB333 = 7;
static uint32_t frameMask[LOGICAL_HEIGHT][MASK_SEGMENTS_PER_ROW];

bool mapPoint(int x, int y, int &mappedX, int &mappedY);
void resetFrameBuffer();
void renderFrameBuffer();
void resetBinaryParser(BinaryParserContext &ctx);
uint16_t computeBinaryChecksum(uint8_t type, uint16_t len, const uint8_t *payload);
uint32_t readU32LE(const uint8_t *src, uint16_t offset);
bool processBinaryByte(BinaryParserContext &ctx, uint8_t byte, InputSource src);
void sendLineToSource(InputSource src, const char *line);
void sendLineToSource(InputSource src, const String &line);
void handleLine(const char *line, InputSource src);
void processIncomingByte(uint8_t byte, InputSource src);
void setupBluetooth();

void clearScreen() {
  dma_display->clearScreen();
}

void updateDrawColor() {
  const uint8_t scaledR = (uint8_t)((colorR333 * drawBrightness + (BRIGHTNESS_MAX_LEVEL / 2)) / BRIGHTNESS_MAX_LEVEL);
  const uint8_t scaledG = (uint8_t)((colorG333 * drawBrightness + (BRIGHTNESS_MAX_LEVEL / 2)) / BRIGHTNESS_MAX_LEVEL);
  const uint8_t scaledB = (uint8_t)((colorB333 * drawBrightness + (BRIGHTNESS_MAX_LEVEL / 2)) / BRIGHTNESS_MAX_LEVEL);
  const uint8_t r8 = (uint8_t)((scaledR * 255 + 3) / 7);
  const uint8_t g8 = (uint8_t)((scaledG * 255 + 3) / 7);
  const uint8_t b8 = (uint8_t)((scaledB * 255 + 3) / 7);
  drawColor = dma_display->color565(r8, g8, b8);
}

void applyDrawBrightness(uint8_t level) {
  if (level > BRIGHTNESS_MAX_LEVEL) {
    level = BRIGHTNESS_MAX_LEVEL;
  }
  drawBrightness = level;
  updateDrawColor();
}

void applyColor333(uint8_t r, uint8_t g, uint8_t b) {
  if (r > 7) r = 7;
  if (g > 7) g = 7;
  if (b > 7) b = 7;
  colorR333 = r;
  colorG333 = g;
  colorB333 = b;
  updateDrawColor();
}

bool mapPoint(int x, int y, int &mappedX, int &mappedY) {
  if (x < 0 || x >= PANEL_RES_X || y < 0 || y >= PANEL_RES_Y) {
    return false;
  }

  int px = x;
  int py = y;

  if (MAP_SWAP_XY) {
    px = y;
    py = x;
  }
  if (MAP_FLIP_X) {
    px = (PANEL_RES_X - 1) - px;
  }
  if (MAP_FLIP_Y) {
    py = (PANEL_RES_Y - 1) - py;
  }

  int lx = 0;
  int ly = py;

  if (py < 26) {
    lx = px + 39;
    ly = py;
  } else if (py < 52) {
    lx = px + 39; 
    ly = py;
  } else {
    if (px < 39) {
      lx = px;
      ly = py - 52; 
    } else {
      lx = px - 39;
      ly = (py - 52) + 26;
    }
  }

  mappedX = lx;
  mappedY = ly;

  if (PANEL_Y_OFFSET_COMPENSATION_ENABLED) {
    mappedY += PANEL_Y_OFFSET_COMPENSATION_VALUE;
  }

  if (mappedX < 0 || mappedX >= LOGICAL_WIDTH || mappedY < 0 || mappedY >= LOGICAL_HEIGHT) {
    return false;
  }

  return true;
}

void drawSinglePoint(int x, int y) {
  // Let VirtualMatrixPanel handle the mapping
  dma_display->drawPixel((uint8_t)x, (uint8_t)y, drawColor);
}

void drawSinglePointColor333Aligned(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
  // Let VirtualMatrixPanel handle the mapping
  const uint8_t r8 = (uint8_t)((r * 255 + 3) / 7);
  const uint8_t g8 = (uint8_t)((g * 255 + 3) / 7);
  const uint8_t b8 = (uint8_t)((b * 255 + 3) / 7);
  dma_display->drawPixel((uint8_t)x, (uint8_t)y, dma_display->color565(r8, g8, b8));
}

void resetFrameBuffer() {
  for (uint8_t row = 0; row < LOGICAL_HEIGHT; ++row) {
    for (uint16_t seg = 0; seg < MASK_SEGMENTS_PER_ROW; ++seg) {
      frameMask[row][seg] = 0;
    }
  }
}

void setFramePoint(int x, int y) {
  int mappedX = x;
  int mappedY = y;
  if (!mapPoint(x, y, mappedX, mappedY)) {
    return;
  }

  const uint16_t segment = mappedX / 32;
  const uint8_t bit = mappedX % 32;
  if (segment < MASK_SEGMENTS_PER_ROW) {
    frameMask[mappedY][segment] |= (1UL << bit);
  }
}

void drawRowMask(uint8_t y, uint32_t *maskSegments) {
  for (uint16_t seg = 0; seg < MASK_SEGMENTS_PER_ROW; ++seg) {
    const uint32_t segment = maskSegments[seg];
    for (uint8_t bit = 0; bit < 32; ++bit) {
      if ((segment & (1UL << bit)) != 0) {
        const uint16_t lx = seg * 32 + bit;
        if (lx < LOGICAL_WIDTH) {
          // In mask mode, we directly update the DMA buffer using logical coordinates
          dma_display->drawPixel((uint8_t)lx, (uint8_t)y, drawColor);
        }
      }
    }
  }
}

void renderFrameBuffer() {
  clearScreen();
  for (uint8_t row = 0; row < LOGICAL_HEIGHT; ++row) {
    bool hasData = false;
    for (uint16_t seg = 0; seg < MASK_SEGMENTS_PER_ROW; ++seg) {
      if (frameMask[row][seg] != 0) {
        hasData = true;
        break;
      }
    }
    if (hasData) {
      drawRowMask(row, frameMask[row]);
    }
  }
}

void resetBinaryParser(BinaryParserContext &ctx) {
  ctx.state = BIN_WAIT_SYNC1;
  ctx.type = 0;
  ctx.len = 0;
  ctx.index = 0;
  ctx.chkRead = 0;
}

uint16_t computeBinaryChecksum(uint8_t type, uint16_t len, const uint8_t *payload) {
  uint16_t sum = 0;
  sum = (sum + type) & 0xFFFF;
  sum = (sum + (len & 0xFF)) & 0xFFFF;
  sum = (sum + ((len >> 8) & 0xFF)) & 0xFFFF;
  for (uint16_t i = 0; i < len; ++i) {
    sum = (sum + payload[i]) & 0xFFFF;
  }
  return sum;
}

uint32_t readU32LE(const uint8_t *src, uint16_t offset) {
  return (uint32_t)src[offset] |
         ((uint32_t)src[offset + 1] << 8) |
         ((uint32_t)src[offset + 2] << 16) |
         ((uint32_t)src[offset + 3] << 24);
}

void applyBinaryFrame(const uint8_t *payload, uint16_t len) {
  if (len != BINARY_PAYLOAD_SIZE_MASK) {
    return;
  }

  resetFrameBuffer();
  for (uint8_t row = 0; row < PANEL_RES_Y; ++row) {
    const uint16_t base = (uint16_t)row * MASK_SEGMENTS_PER_ROW * 4;
    for (uint16_t seg = 0; seg < MASK_SEGMENTS_PER_ROW; ++seg) {
      const uint32_t segmentMask = readU32LE(payload, base + seg * 4);
      for (uint8_t bit = 0; bit < 32; ++bit) {
        if ((segmentMask & (1UL << bit)) != 0) {
          const uint16_t x = seg * 32 + bit;
          if (x < PANEL_RES_X) {
            setFramePoint(x, row);
          }
        }
      }
    }
  }

  renderFrameBuffer();
}

void applyBinaryRgb332RowFrame(const uint8_t *payload, uint16_t len) {
  if (len != BINARY_PAYLOAD_SIZE_RGB332_ROW) {
    return;
  }

  const uint8_t row = payload[0];
  if (row >= PANEL_RES_Y) {
    return;
  }
  
  // Debug: print when processing row 0
  if (row == 0) {
    Serial.print("DEBUG: Processing row 0 frame, ");
    Serial.print(PANEL_RES_X);
    Serial.println(" pixels");
  }

  for (uint8_t x = 0; x < PANEL_RES_X; ++x) {
    const uint8_t packed = payload[x + 1];
    const uint8_t r3 = (packed >> 5) & 0x07;
    const uint8_t g3 = (packed >> 2) & 0x07;
    const uint8_t b2 = packed & 0x03;
    const uint8_t b3 = (uint8_t)((b2 * 7 + 1) / 3);
    const uint8_t scaledR = (uint8_t)((r3 * drawBrightness + (BRIGHTNESS_MAX_LEVEL / 2)) / BRIGHTNESS_MAX_LEVEL);
    const uint8_t scaledG = (uint8_t)((g3 * drawBrightness + (BRIGHTNESS_MAX_LEVEL / 2)) / BRIGHTNESS_MAX_LEVEL);
    const uint8_t scaledB = (uint8_t)((b3 * drawBrightness + (BRIGHTNESS_MAX_LEVEL / 2)) / BRIGHTNESS_MAX_LEVEL);
    drawSinglePointColor333Aligned(x, row, scaledR, scaledG, scaledB);
  }
}

void applyBinaryRgb332FullFrame(const uint8_t *payload, uint16_t len) {
  if (len != BINARY_PAYLOAD_SIZE_RGB332_FULL) {
    return;
  }

  resetFrameBuffer();
  clearScreen();
  for (uint8_t row = 0; row < PANEL_RES_Y; ++row) {
    const uint16_t base = (uint16_t)row * PANEL_RES_X;
    for (uint8_t x = 0; x < PANEL_RES_X; ++x) {
      const uint8_t packed = payload[base + x];
      const uint8_t r3 = (packed >> 5) & 0x07;
      const uint8_t g3 = (packed >> 2) & 0x07;
      const uint8_t b2 = packed & 0x03;
      const uint8_t b3 = (uint8_t)((b2 * 7 + 1) / 3);
      const uint8_t scaledR = (uint8_t)((r3 * drawBrightness + (BRIGHTNESS_MAX_LEVEL / 2)) / BRIGHTNESS_MAX_LEVEL);
      const uint8_t scaledG = (uint8_t)((g3 * drawBrightness + (BRIGHTNESS_MAX_LEVEL / 2)) / BRIGHTNESS_MAX_LEVEL);
      const uint8_t scaledB = (uint8_t)((b3 * drawBrightness + (BRIGHTNESS_MAX_LEVEL / 2)) / BRIGHTNESS_MAX_LEVEL);
      drawSinglePointColor333Aligned(x, row, scaledR, scaledG, scaledB);
    }
  }
}

bool processBinaryByte(BinaryParserContext &ctx, uint8_t byte, InputSource src) {
  switch (ctx.state) {
    case BIN_WAIT_SYNC1:
      if (byte == BINARY_SYNC_1) {
        ctx.state = BIN_WAIT_SYNC2;
        return true;
      }
      return false;
    case BIN_WAIT_SYNC2:
      if (byte == BINARY_SYNC_2) {
        ctx.state = BIN_READ_TYPE;
        return true;
      }
      resetBinaryParser(ctx);
      return false;
    case BIN_READ_TYPE:
      ctx.type = byte;
      ctx.state = BIN_READ_LEN_L;
      return true;
    case BIN_READ_LEN_L:
      ctx.len = byte;
      ctx.state = BIN_READ_LEN_H;
      return true;
    case BIN_READ_LEN_H:
      ctx.len |= ((uint16_t)byte << 8);
      if (ctx.len > BINARY_PAYLOAD_SIZE_MAX) {
        sendLineToSource(src, "BIN ERR len");
        resetBinaryParser(ctx);
      } else {
        ctx.index = 0;
        ctx.state = BIN_READ_PAYLOAD;
      }
      return true;
    case BIN_READ_PAYLOAD:
      if (ctx.index < ctx.len) {
        ctx.payload[ctx.index++] = byte;
      }
      if (ctx.index >= ctx.len) {
        ctx.state = BIN_READ_CHK_L;
      }
      return true;
    case BIN_READ_CHK_L:
      ctx.chkRead = byte;
      ctx.state = BIN_READ_CHK_H;
      return true;
    case BIN_READ_CHK_H: {
      ctx.chkRead |= ((uint16_t)byte << 8);
      const uint16_t checksum = computeBinaryChecksum(ctx.type, ctx.len, ctx.payload);
      if (checksum != ctx.chkRead) {
        sendLineToSource(src, "BIN ERR checksum");
      } else if (ctx.type == BINARY_FRAME_TYPE_MASK && ctx.len == BINARY_PAYLOAD_SIZE_MASK) {
        applyBinaryFrame(ctx.payload, ctx.len);
        // sendLineToSource(src, "BIN OK"); // 减少反馈，避免阻塞
      } else if (ctx.type == BINARY_FRAME_TYPE_RGB332_ROW && ctx.len == BINARY_PAYLOAD_SIZE_RGB332_ROW) {
        applyBinaryRgb332RowFrame(ctx.payload, ctx.len);
        // sendLineToSource(src, "BIN OK"); // 减少反馈
      } else if (ctx.type == BINARY_FRAME_TYPE_RGB332_FULL && ctx.len == BINARY_PAYLOAD_SIZE_RGB332_FULL) {
        applyBinaryRgb332FullFrame(ctx.payload, ctx.len);
        sendLineToSource(src, "BIN OK"); // 仅全帧完成后反馈
      } else {
        sendLineToSource(src, "BIN ERR type");
      }
      resetBinaryParser(ctx);
      return true;
    }
  }

  resetBinaryParser(ctx);
  return false;
}

void sendLineToSource(InputSource src, const String &line) {
  sendLineToSource(src, line.c_str());
}

void sendLineToSource(InputSource src, const char *line) {
  if (line == nullptr) {
    return;
  }

  Serial.println(line);
  if (src != SRC_BLE || bleTxCharacteristic == nullptr) {
    return;
  }

  String payload = String(line) + "\n";
  const char *raw = payload.c_str();
  const size_t length = payload.length();
  size_t offset = 0;
  while (offset < length) {
    const size_t remain = length - offset;
    const size_t chunk = remain > BLE_TX_MAX_NOTIFY_BYTES ? BLE_TX_MAX_NOTIFY_BYTES : remain;
    uint8_t txChunk[BLE_TX_MAX_NOTIFY_BYTES];
    memcpy(txChunk, raw + offset, chunk);
    bleTxCharacteristic->setValue(txChunk, chunk);
    bleTxCharacteristic->notify();
    offset += chunk;
    delay(10); // 增加延迟，给 BLE 栈处理时间
  }
}

void handleLine(const char *line, InputSource src) {
  if (strcmp(line, "PING") == 0) {
    sendLineToSource(src, "PONG");
    return;
  }

  if (strcmp(line, "CLEAR") == 0) {
    resetFrameBuffer();
    clearScreen();
    sendLineToSource(src, "CLEAR OK");
    return;
  }

  int x = 0;
  int y = 0;
  int r = 0;
  int g = 0;
  int b = 0;
  if (sscanf(line, "PIXEL %d %d %d %d %d", &x, &y, &r, &g, &b) == 5) {
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= PANEL_RES_X) x = PANEL_RES_X - 1;
    if (y >= PANEL_RES_Y) y = PANEL_RES_Y - 1;
    if (r < 0) r = 0;
    if (g < 0) g = 0;
    if (b < 0) b = 0;
    if (r > 255) r = 255;
    if (g > 255) g = 255;
    if (b > 255) b = 255;

    drawSinglePointColor333Aligned(
      x,
      y,
      (uint8_t)((r * 7 + 127) / 255),
      (uint8_t)((g * 7 + 127) / 255),
      (uint8_t)((b * 7 + 127) / 255)
    );
    sendLineToSource(src, String("PIXEL OK ") + x + " " + y + " " + r + " " + g + " " + b);
    return;
  }

  // 直接操作逻辑缓冲区的调试命令
  if (sscanf(line, "LPIXEL %d %d %d %d %d", &x, &y, &r, &g, &b) == 5) {
    if (x < 0) x = 0; if (y < 0) y = 0;
    if (x >= LOGICAL_WIDTH) x = LOGICAL_WIDTH - 1;
    if (y >= LOGICAL_HEIGHT) y = LOGICAL_HEIGHT - 1;
    
    const uint8_t r8 = (uint8_t)((r * 7 + 127) / 255);
    const uint8_t g8 = (uint8_t)((g * 7 + 127) / 255);
    const uint8_t b8 = (uint8_t)((b * 7 + 127) / 255);
    dma_display->drawPixel((uint8_t)x, (uint8_t)y, dma_display->color565(
      (uint8_t)((r8 * 255 + 3) / 7), 
      (uint8_t)((g8 * 255 + 3) / 7), 
      (uint8_t)((b8 * 255 + 3) / 7)
    ));
    sendLineToSource(src, String("LPIXEL OK ") + x + " " + y);
    return;
  }

  if (sscanf(line, "COLOR %d %d %d", &r, &g, &b) == 3) {
    if (r < 0) r = 0;
    if (g < 0) g = 0;
    if (b < 0) b = 0;
    if (r > 255) r = 255;
    if (g > 255) g = 255;
    if (b > 255) b = 255;
    applyColor333((uint8_t)((r * 7 + 127) / 255), (uint8_t)((g * 7 + 127) / 255), (uint8_t)((b * 7 + 127) / 255));
    return;
  }

  int level = 0;
  if (sscanf(line, "BRIGHTNESS %d", &level) == 1) {
    if (level < 0) level = 0;
    if (level > BRIGHTNESS_MAX_LEVEL) level = BRIGHTNESS_MAX_LEVEL;
    applyDrawBrightness((uint8_t)level);
    return;
  }

  sendLineToSource(src, String("ERR ") + line);
}

void processIncomingByte(uint8_t byte, InputSource src) {
  if (src >= SRC_COUNT) {
    return;
  }

  BinaryParserContext &binCtx = binaryContexts[src];
  if (processBinaryByte(binCtx, byte, src)) {
    return;
  }

  char c = static_cast<char>(byte);
  if (c == '\r') {
    return;
  }

  LineParserContext &lineCtx = lineContexts[src];
  if (c == '\n' || c == ';') {
    lineCtx.buffer[lineCtx.length] = '\0';
    if (lineCtx.length > 0) {
      handleLine(lineCtx.buffer, src);
    }
    lineCtx.length = 0;
    return;
  }

  if (lineCtx.length < SERIAL_LINE_BUFFER_SIZE - 1) {
    lineCtx.buffer[lineCtx.length++] = c;
  } else {
    lineCtx.length = 0;
    sendLineToSource(src, "ERR LINE_TOO_LONG");
  }
}

void pollSerialCommands() {
  while (Serial.available() > 0) {
    processIncomingByte((uint8_t)Serial.read(), SRC_SERIAL);
  }
}

class BleServerCallbacks : public BLEServerCallbacks {
public:
  void onDisconnect(BLEServer *server) override {
    if (server != nullptr) {
      server->startAdvertising();
    }
  }
};

class BleRxCallbacks : public BLECharacteristicCallbacks {
public:
  void onWrite(BLECharacteristic *characteristic) override {
    if (characteristic == nullptr) {
      return;
    }

    std::string value = characteristic->getValue();
    for (size_t i = 0; i < value.size(); ++i) {
      processIncomingByte((uint8_t)value[i], SRC_BLE);
    }
  }
};

void setupBluetooth() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEServer *server = BLEDevice::createServer();
  if (server == nullptr) {
    Serial.println("BLE INIT FAIL");
    return;
  }

  server->setCallbacks(new BleServerCallbacks());
  BLEService *service = server->createService(BLE_SERVICE_UUID);
  if (service == nullptr) {
    Serial.println("BLE SVC FAIL");
    return;
  }

  BLECharacteristic *rx = service->createCharacteristic(
    BLE_RX_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  bleTxCharacteristic = service->createCharacteristic(
    BLE_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );

  if (rx == nullptr || bleTxCharacteristic == nullptr) {
    Serial.println("BLE CHAR FAIL");
    return;
  }

  rx->setCallbacks(new BleRxCallbacks());
  bleTxCharacteristic->addDescriptor(new BLE2902());
  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  if (advertising != nullptr) {
    advertising->addServiceUUID(BLE_SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->start();
  }
}

void setupMatrix() {
  mxconfig.gpio.r1  = R1_PIN;
  mxconfig.gpio.g1  = G1_PIN;
  mxconfig.gpio.b1  = B1_PIN;
  mxconfig.gpio.r2  = R2_PIN;
  mxconfig.gpio.g2  = G2_PIN;
  mxconfig.gpio.b2  = B2_PIN;
  mxconfig.gpio.a   = A_PIN;
  mxconfig.gpio.b   = B_PIN;
  mxconfig.gpio.c   = C_PIN;
  mxconfig.gpio.d   = D_PIN;
  mxconfig.gpio.e   = E_PIN;
  mxconfig.gpio.lat = LAT_PIN;
  mxconfig.gpio.oe  = OE_PIN;
  mxconfig.gpio.clk = CLK_PIN;
  mxconfig.driver   = HUB75_I2S_CFG::SHIFTREG; 
  mxconfig.line_decoder = HUB75_I2S_CFG::TYPE595;

  mxconfig.i2sspeed = MATRIX_I2S_SPEED;
  mxconfig.clkphase = MATRIX_CLKPHASE;
  mxconfig.min_refresh_rate = MATRIX_MIN_REFRESH_RATE;
  mxconfig.setPixelColorDepthBits(MATRIX_COLOR_DEPTH_BITS);

  dma_display = new VirtualMatrixPanel78x78(mxconfig);
  const bool matrixOk = dma_display->begin();
  if (!matrixOk) {
    Serial.println("MATRIX INIT FAIL");
    return;
  }

  dma_display->setLatBlanking(MATRIX_LAT_BLANKING);
  dma_display->setBrightness8(MATRIX_PANEL_BRIGHTNESS);
  applyColor333(7, 7, 7);
  applyDrawBrightness(BRIGHTNESS_MAX_LEVEL);
  resetFrameBuffer();
  clearScreen();
}

void setup() {
  Serial.begin(57600);
  delay(250);

  for (uint8_t i = 0; i < SRC_COUNT; ++i) {
    resetBinaryParser(binaryContexts[i]);
    lineContexts[i].length = 0;
  }

  setupBluetooth();
  setupMatrix();
}

void loop() {
  pollSerialCommands();
}
