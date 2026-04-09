#include "ESP32-HUB75-MatrixPanel-I2S-DMA.h"
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define R1_PIN 1
#define G1_PIN 2
#define B1_PIN 3
#define R2_PIN 4
#define G2_PIN 5
#define B2_PIN 6
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

// ICN2053 26S panel Y-offset compensation
// Middle 26 columns (x: 26-51) are offset by 2 rows compared to the sides
#define PANEL_Y_OFFSET_COMPENSATION_ENABLED true
#define PANEL_Y_OFFSET_COMPENSATION_VALUE 2
#define PANEL_SEGMENT_WIDTH 26

// Segment swap and zigzag configuration for 78x78 26S panels
// If middle segment shows blue instead of green, segments 2 and 3 are likely swapped in the chain
#define ENABLE_SEGMENT_SWAP true
// If middle segment is split or mirrored at 13 pixels, it may have a zigzag mapping
#define ENABLE_SEGMENT_ZIGZAG true

static const size_t SERIAL_LINE_BUFFER_SIZE = 160;
static const uint8_t BINARY_SYNC_1 = 0xAA;
static const uint8_t BINARY_SYNC_2 = 0x55;
static const uint8_t BINARY_FRAME_TYPE_MASK = 0x01;
static const uint8_t BINARY_FRAME_TYPE_RGB332_ROW = 0x02;
static const uint8_t BINARY_FRAME_TYPE_RGB332_FULL = 0x03;
static const uint8_t BRIGHTNESS_MAX_LEVEL = 4;
static const uint16_t MASK_SEGMENTS_PER_ROW = (PANEL_RES_X + 31) / 32;
static const uint16_t BINARY_PAYLOAD_SIZE_MASK = MASK_SEGMENTS_PER_ROW * 4 * PANEL_RES_Y;
static const uint16_t BINARY_PAYLOAD_SIZE_RGB332_ROW = PANEL_RES_X + 1;
static const uint16_t BINARY_PAYLOAD_SIZE_RGB332_FULL = PANEL_RES_X * PANEL_RES_Y;
static const uint16_t BINARY_PAYLOAD_SIZE_MAX = BINARY_PAYLOAD_SIZE_RGB332_FULL;
static const HUB75_I2S_CFG::clk_speed MATRIX_I2S_SPEED = HUB75_I2S_CFG::HZ_16M;
static const bool MATRIX_CLKPHASE = true;
static const uint8_t MATRIX_LAT_BLANKING = 1;
static const uint8_t MATRIX_MIN_REFRESH_RATE = 180;
static const uint8_t MATRIX_COLOR_DEPTH_BITS = 6;
static const uint8_t MATRIX_PANEL_BRIGHTNESS = 90;
static const char *BLE_DEVICE_NAME = "ESP32-LED-Panel";
static const char *BLE_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *BLE_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *BLE_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
static const size_t BLE_TX_MAX_NOTIFY_BYTES = 20;
static const bool MAP_SWAP_XY = true;
static const bool MAP_FLIP_X = false;
static const bool MAP_FLIP_Y = false;

MatrixPanel_I2S_DMA *dma_display = nullptr;
BLECharacteristic *bleTxCharacteristic = nullptr;

HUB75_I2S_CFG mxconfig(
  PANEL_RES_X,
  PANEL_RES_Y,
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
static uint32_t frameMask[PANEL_RES_Y][MASK_SEGMENTS_PER_ROW];

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

// Get Y offset compensation for a given X coordinate
// For ICN2053 26S panels: middle segment (x: 26-51) displays 2 rows higher
// This compensation shifts the middle segment down to align with the sides
int getYOffsetCompensation(int x) {
#if PANEL_Y_OFFSET_COMPENSATION_ENABLED
  // Determine which segment the X coordinate falls into
  // Panel is divided into 3 segments of 26 columns each
  // Middle segment (segment 1) needs compensation
  int segment = x / PANEL_SEGMENT_WIDTH;
  if (segment == 1) {  // Middle segment (x: 26-51)
    return PANEL_Y_OFFSET_COMPENSATION_VALUE;  // Shift down to align with sides
  }
#endif
  return 0;
}

bool mapPoint(int x, int y, int &mappedX, int &mappedY) {
  if (x < 0 || x >= PANEL_RES_X || y < 0 || y >= PANEL_RES_Y) {
    return false;
  }

  int lx = x;
  int ly = y;

  // 1. 段序交换 (Segment Swap)
  // 逻辑顺序: Seg1(0-25), Seg2(26-51), Seg3(52-77)
  // 物理顺序: Seg1 -> Seg3 -> Seg2
  // 如果中间(26-51)显示了蓝(52-77的数据)，则执行以下交换
  if (ENABLE_SEGMENT_SWAP) {
    if (x >= 26 && x < 52) { 
      lx = x + 26; // 逻辑绿(26-51) -> 映射到物理末尾
    } else if (x >= 52) {
      lx = x - 26; // 逻辑蓝(52-77) -> 映射到物理中间
    }
  }

  // 2. 13像素块内映射 (13-pixel Block Mapping)
  // 解决您提到的“13列蓝色”分裂问题
  // 某些26S面板在每个26列段内，0-12和13-25是对调的，或者有位移
  if (ENABLE_SEGMENT_ZIGZAG) {
    int segBase = (lx / 26) * 26;
    int relX = lx % 26;
    if (relX < 13) {
      lx = segBase + relX + 13; // 左13移到右
    } else {
      lx = segBase + relX - 13; // 右13移到左
    }
  }

  mappedX = lx;
  mappedY = ly;

  // 3. 行偏置补偿 (Y-Offset Compensation)
  // 注意：这里的补偿应该基于物理位置。如果中间段物理上是Seg3，则需调整判断条件
  // 我们暂时保持对逻辑中间位置的补偿，如果对齐不对再微调
  if (PANEL_Y_OFFSET_COMPENSATION_ENABLED) {
    if (x >= 26 && x < 52) { 
      mappedY += PANEL_Y_OFFSET_COMPENSATION_VALUE;
    }
  }

  // Check bounds after compensation
  if (mappedY < 0 || mappedY >= PANEL_RES_Y) {
    return false;
  }

  if (MAP_SWAP_XY) {
    const int t = mappedX;
    mappedX = mappedY;
    mappedY = t;
  }

  if (MAP_FLIP_X) {
    mappedX = (PANEL_RES_X - 1) - mappedX;
  }

  if (MAP_FLIP_Y) {
    mappedY = (PANEL_RES_Y - 1) - mappedY;
  }

  return mappedX >= 0 && mappedX < PANEL_RES_X && mappedY >= 0 && mappedY < PANEL_RES_Y;
}

void drawSinglePoint(int x, int y) {
  int mappedX = x;
  int mappedY = y;
  if (!mapPoint(x, y, mappedX, mappedY)) {
    return;
  }
  dma_display->drawPixel((uint8_t)mappedX, (uint8_t)mappedY, drawColor);
}

void drawSinglePointColor333Aligned(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
  int mappedX = x;
  int mappedY = y;
  if (!mapPoint(x, y, mappedX, mappedY)) {
    return;
  }

  const uint8_t r8 = (uint8_t)((r * 255 + 3) / 7);
  const uint8_t g8 = (uint8_t)((g * 255 + 3) / 7);
  const uint8_t b8 = (uint8_t)((b * 255 + 3) / 7);
  dma_display->drawPixel((uint8_t)mappedX, (uint8_t)mappedY, dma_display->color565(r8, g8, b8));
}

void resetFrameBuffer() {
  for (uint8_t row = 0; row < PANEL_RES_Y; ++row) {
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
        const uint16_t x = seg * 32 + bit;
        if (x < PANEL_RES_X) {
          drawSinglePoint(x, y);
        }
      }
    }
  }
}

void renderFrameBuffer() {
  clearScreen();
  for (uint8_t row = 0; row < PANEL_RES_Y; ++row) {
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

  if (row == 0) {
    resetFrameBuffer();
    clearScreen();
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
        sendLineToSource(src, "BIN OK");
      } else if (ctx.type == BINARY_FRAME_TYPE_RGB332_ROW && ctx.len == BINARY_PAYLOAD_SIZE_RGB332_ROW) {
        applyBinaryRgb332RowFrame(ctx.payload, ctx.len);
        sendLineToSource(src, "BIN OK");
      } else if (ctx.type == BINARY_FRAME_TYPE_RGB332_FULL && ctx.len == BINARY_PAYLOAD_SIZE_RGB332_FULL) {
        applyBinaryRgb332FullFrame(ctx.payload, ctx.len);
        sendLineToSource(src, "BIN OK");
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
    delay(2);
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
  mxconfig.driver   = HUB75_I2S_CFG::FM6126A;
  mxconfig.line_decoder = HUB75_I2S_CFG::SM5368;

  mxconfig.i2sspeed = MATRIX_I2S_SPEED;
  mxconfig.clkphase = MATRIX_CLKPHASE;
  mxconfig.min_refresh_rate = MATRIX_MIN_REFRESH_RATE;
  mxconfig.setPixelColorDepthBits(MATRIX_COLOR_DEPTH_BITS);

  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
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
