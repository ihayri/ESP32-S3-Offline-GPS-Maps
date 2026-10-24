#include <Arduino.h>
#include <FFat.h>
#include <Wire.h>
#include <EEPROM.h>
#define GFX_DEV_DEVICE LILYGO_T_RGB
#include <Arduino_GFX_Library.h>
#include <TinyGPS++.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

// ============================================================
// LILYGO T-RGB 2.1" HALF-CIRCLE
// ============================================================

#define GFX_BL       46
#define TOUCH_SDA     8
#define TOUCH_SCL    48
#define TOUCH_ADDR  0x38
#define GPS_RX_PIN   38
#define GPS_BAUD  38400

#define SHOW_SPEED             0
#define SHOW_SAT_COUNT         0
#define SHOW_NMEA_RX_INDICATOR 0

#define SCR_W 480
#define SCR_H 480
#define BOTTOM_BLACK_STRIPE_H 25
#define MAP_H (SCR_H - BOTTOM_BLACK_STRIPE_H)
#define CX 240
#define CY 240

// ============================================================
// COLORS
// ============================================================

//#define COL_BG       0x3006 // dark purple
#define COL_BG       0x0431 //0x0EBB   // teal too bright
#define COL_RAIL     0xF81F
#define COL_TRUNK    0xFFE0
#define COL_PRIMARY  0xFD20
#define COL_MID      0x9C92
#define COL_LOCAL    0xFFFF
#define COL_DOT      0x07FF
#define COL_GREEN    0x07E0
#define COL_YELLOW   0xFFE0
#define COL_RED      0xF800
#define COL_HUD_BG   0x2104
#define COL_HUD_FG   0xFD20
#define COL_HUD_UNIT 0xFFFF
#define COL_HINT     0x8410

// Dark green forest fill.
#define COL_FOREST   0x1A86

// ============================================================
// ZOOM
// ============================================================

static float PIXELS_PER_DEG = 18000.0f;

static const float ZOOM_MIN = 100.0f;
static const float ZOOM_MAX = 20000.0f;
static const float ZOOM_IN_FACTOR = 1.33f;
static const float ZOOM_OUT_FACTOR = 0.75f;

#define EEPROM_SIZE 8
#define EEPROM_ZOOM_ADDR 0

static float lastSavedZoom = 0.0f;
static unsigned long lastZoomChange = 0;

// ============================================================
// DISPLAY HARDWARE
// ============================================================

Arduino_DataBus *displayBus = new Arduino_XL9535SWSPI(
  8, 48, 2, 3, 5, 4
);

Arduino_ESP32RGBPanel *rgbPanel = new Arduino_ESP32RGBPanel(
  45, 41, 47, 42,
  21, 18, 17, 16, 15,
  14, 13, 12, 11, 10, 9,
  7, 6, 5, 3, 2,
  1, 50, 1, 30,
  1, 20, 1, 30,
  1
);

Arduino_GFX *gfx = new Arduino_RGB_Display(
  480, 480, rgbPanel, 0, true, displayBus, GFX_NOT_DEFINED,
  st7701_type4_init_operations,
  sizeof(st7701_type4_init_operations)
);

TinyGPSPlus gps;
HardwareSerial GPSSerial(1);

static uint16_t *frameBuffer = nullptr;

// ============================================================
// ROAD MAP DATA
// ============================================================

struct RoadSeg {
  uint8_t tier;
  uint16_t nPoints;
  uint8_t* ptsBase;

  int16_t minQLat;
  int16_t maxQLat;
  int16_t minQLon;
  int16_t maxQLon;
};

static uint8_t* mapBlob = nullptr;
static size_t mapBlobSize = 0;

static int32_t bboxMinLat_e7 = 0;
static int32_t bboxMinLon_e7 = 0;
static int32_t bboxMaxLat_e7 = 0;
static int32_t bboxMaxLon_e7 = 0;

static uint32_t latScale = 0;
static uint32_t lonScale = 0;

static uint32_t numSegs = 0;
static RoadSeg* segs = nullptr;

// ============================================================
// FOREST DATA
//
// FRST file format:
//   uint32 magic = 0x54535246
//   int32  minLat, minLon, maxLat, maxLon (e7)
//   uint32 latScale, lonScale
//   uint32 polygon count
//
// Each polygon:
//   uint16 point count
//   repeated int16 qlat, int16 qlon
// ============================================================

struct ForestPoly {
  uint16_t nPoints;
  uint8_t* ptsBase;

  int16_t minQLat;
  int16_t maxQLat;
  int16_t minQLon;
  int16_t maxQLon;
};

static uint8_t* forestBlob = nullptr;
static size_t forestBlobSize = 0;

static uint32_t numForests = 0;
static ForestPoly* forests = nullptr;

// Largest polygon determines the required scratch space.
// One scratch allocation holds X coordinates, Y coordinates,
// and scanline intersections, all as int32_t.
static uint32_t maxForestPoints = 0;
static int32_t* forestScratch = nullptr;

// ============================================================
// VEHICLE POSITION
// ============================================================

static float vehLatF = 43.8059f;
static float vehLonF = 27.1194f;

static bool haveFix = false;
static bool needsRedraw = true;

//static unsigned long lastMapDraw = 0;
//static const unsigned long MAP_REDRAW_INTERVAL_MS = 200;

// ============================================================
// TOUCH STATE
// ============================================================

static bool touchWasDown = false;
static unsigned long lastTouchAction = 0;
static const unsigned long TOUCH_DEBOUNCE_MS = 220;

// ============================================================
// OPTIONAL SPEED
// ============================================================

#if SHOW_SPEED
static int cachedSpeedKmh = 0;
static int lastShownSpeed = -999;

void updateSpeedDirect() {
  int spd = cachedSpeedKmh;
  if (spd == lastShownSpeed) return;

  lastShownSpeed = spd;

  char speedText[8];
  snprintf(speedText, sizeof(speedText), "%3d", spd);

  gfx->setTextSize(4);
  gfx->setTextColor(COL_HUD_FG, COL_BG);
  gfx->setCursor(175, 407);
  gfx->print(speedText);

  gfx->setTextSize(2);
  gfx->setTextColor(COL_HUD_UNIT, COL_BG);
  gfx->setCursor(255, 424);
  gfx->print("km/h");
}
#endif

// ============================================================
// GPS READING
// ============================================================

void readGPS() {
  while (GPSSerial.available() > 0) {
    char c = (char)GPSSerial.read();
    gps.encode(c);

#if SHOW_SPEED
    if (gps.speed.isUpdated() && gps.speed.isValid()) {
      int newSpeed = (int)round(gps.speed.kmph());

      if (newSpeed != cachedSpeedKmh) {
        cachedSpeedKmh = newSpeed;
        updateSpeedDirect();
      }
    }
#endif
  }
}

// ============================================================
// EEPROM ZOOM
// ============================================================

float loadZoomFromEEPROM() {
  float v = 0.0f;
  EEPROM.get(EEPROM_ZOOM_ADDR, v);

  if (isnan(v) || v < ZOOM_MIN || v > ZOOM_MAX) {
    return 18000.0f;
  }

  return v;
}

void saveZoomToEEPROM(float v) {
  EEPROM.put(EEPROM_ZOOM_ADDR, v);
  EEPROM.commit();
}

// ============================================================
// LOAD A FILE FROM FFAT INTO PSRAM
// ============================================================

bool loadFileFromFFat(
  const char* path,
  uint8_t** destination,
  size_t* destinationSize
) {
  *destination = nullptr;
  *destinationSize = 0;

  if (!FFat.begin(false)) {
    return false;
  }

  File f = FFat.open(path, "r");

  if (!f) {
    FFat.end();
    return false;
  }

  size_t fileSize = f.size();

  if (fileSize == 0) {
    f.close();
    FFat.end();
    return false;
  }

  uint8_t* data = (uint8_t*)ps_malloc(fileSize);

  if (!data) {
    f.close();
    FFat.end();
    return false;
  }

  size_t bytesRead = f.read(data, fileSize);

  f.close();
  FFat.end();

  if (bytesRead != fileSize) {
    free(data);
    return false;
  }

  *destination = data;
  *destinationSize = fileSize;

  return true;
}

// ============================================================
// LOAD ROAD MAP
// ============================================================

bool loadVectorMap(const char* path) {
  return loadFileFromFFat(path, &mapBlob, &mapBlobSize);
}

// ============================================================
// PARSE ROAD MAP
// ============================================================

bool parseVectorMap() {
  if (!mapBlob || mapBlobSize < 32) {
    return false;
  }

  uint8_t* p = mapBlob;
  uint32_t magic;

  memcpy(&magic, p, 4);
  p += 4;

  if (magic != 0x56454331) {
    return false;
  }

  memcpy(&bboxMinLat_e7, p, 4);
  p += 4;

  memcpy(&bboxMinLon_e7, p, 4);
  p += 4;

  memcpy(&bboxMaxLat_e7, p, 4);
  p += 4;

  memcpy(&bboxMaxLon_e7, p, 4);
  p += 4;

  memcpy(&latScale, p, 4);
  p += 4;

  memcpy(&lonScale, p, 4);
  p += 4;

  memcpy(&numSegs, p, 4);
  p += 4;

  if (latScale == 0 || lonScale == 0) {
    return false;
  }

  if (numSegs > mapBlobSize / 3) {
    return false;
  }

  segs = (RoadSeg*)ps_malloc(sizeof(RoadSeg) * numSegs);

  if (!segs && numSegs > 0) {
    return false;
  }

  size_t offset = 32;

  for (uint32_t i = 0; i < numSegs; i++) {
    if (offset + 3 > mapBlobSize) {
      return false;
    }

    segs[i].tier = mapBlob[offset++];

    memcpy(&segs[i].nPoints, mapBlob + offset, 2);
    offset += 2;

    segs[i].ptsBase = mapBlob + offset;

    size_t pointsSize = (size_t)segs[i].nPoints * 4;

    if (pointsSize > mapBlobSize - offset) {
      return false;
    }

    segs[i].minQLat = INT16_MAX;
    segs[i].maxQLat = INT16_MIN;
    segs[i].minQLon = INT16_MAX;
    segs[i].maxQLon = INT16_MIN;

    for (uint16_t k = 0; k < segs[i].nPoints; k++) {
      int16_t qlat;
      int16_t qlon;

      memcpy(&qlat, segs[i].ptsBase + k * 4, 2);
      memcpy(&qlon, segs[i].ptsBase + k * 4 + 2, 2);

      if (qlat < segs[i].minQLat) segs[i].minQLat = qlat;
      if (qlat > segs[i].maxQLat) segs[i].maxQLat = qlat;
      if (qlon < segs[i].minQLon) segs[i].minQLon = qlon;
      if (qlon > segs[i].maxQLon) segs[i].maxQLon = qlon;
    }

    offset += pointsSize;
  }

  return true;
}

// ============================================================
// LOAD FOREST FILE
//
// Forests are optional. If loading or parsing fails, the road
// map can still run without forest shading.
// ============================================================

bool loadForestMap(const char* path) {
  return loadFileFromFFat(path, &forestBlob, &forestBlobSize);
}

// ============================================================
// PARSE FOREST FILE
// ============================================================


bool parseForestMap() {
  if (!forestBlob || forestBlobSize < 32) {
    Serial.println("FOREST ERROR: file missing or shorter than header");
    return false;
  }

  uint8_t* p = forestBlob;
  uint32_t magic;

  memcpy(&magic, p, 4);
  p += 4;

  if (magic != 0x54535246) {
    Serial.printf("FOREST ERROR: wrong magic: 0x%08lX\n",
                  (unsigned long)magic);
    return false;
  }

  int32_t forestMinLat, forestMinLon;
  int32_t forestMaxLat, forestMaxLon;
  uint32_t forestLatScale, forestLonScale;

  memcpy(&forestMinLat, p, 4);
  p += 4;

  memcpy(&forestMinLon, p, 4);
  p += 4;

  memcpy(&forestMaxLat, p, 4);
  p += 4;

  memcpy(&forestMaxLon, p, 4);
  p += 4;

  memcpy(&forestLatScale, p, 4);
  p += 4;

  memcpy(&forestLonScale, p, 4);
  p += 4;

  memcpy(&numForests, p, 4);
  p += 4;

  Serial.printf("Forest file size: %u bytes\n",
                (unsigned int)forestBlobSize);
  Serial.printf("Forest polygons in header: %lu\n",
                (unsigned long)numForests);

  // Forest and road data must use the same coordinate system.
  if (forestLatScale != latScale ||
      forestLonScale != lonScale ||
      forestMinLat != bboxMinLat_e7 ||
      forestMinLon != bboxMinLon_e7 ||
      forestMaxLat != bboxMaxLat_e7 ||
      forestMaxLon != bboxMaxLon_e7) {

    Serial.println("FOREST ERROR: header does not match road map");

    Serial.printf(
      "Scales: forest %lu/%lu, roads %lu/%lu\n",
      (unsigned long)forestLatScale,
      (unsigned long)forestLonScale,
      (unsigned long)latScale,
      (unsigned long)lonScale
    );

    Serial.printf(
      "Latitude bounds: forest %ld..%ld, roads %ld..%ld\n",
      (long)forestMinLat,
      (long)forestMaxLat,
      (long)bboxMinLat_e7,
      (long)bboxMaxLat_e7
    );

    Serial.printf(
      "Longitude bounds: forest %ld..%ld, roads %ld..%ld\n",
      (long)forestMinLon,
      (long)forestMaxLon,
      (long)bboxMinLon_e7,
      (long)bboxMaxLon_e7
    );

    return false;
  }

  // Each polygon requires at least a 2-byte point count.
  if (numForests > (forestBlobSize - 32) / 2) {
    Serial.println("FOREST ERROR: polygon count exceeds file capacity");
    return false;
  }

  forests = nullptr;
  forestScratch = nullptr;
  maxForestPoints = 0;

  if (numForests > 0) {
    forests = (ForestPoly*)ps_malloc(
      sizeof(ForestPoly) * (size_t)numForests
    );

    if (!forests) {
      Serial.println("FOREST ERROR: cannot allocate polygon table");
      return false;
    }
  }

  size_t offset = 32;

  for (uint32_t i = 0; i < numForests; i++) {
    if (offset + 2 > forestBlobSize) {
      Serial.printf("FOREST ERROR: missing point count at polygon %lu\n",
                    (unsigned long)i);
      return false;
    }

    uint16_t nPoints;
    memcpy(&nPoints, forestBlob + offset, 2);
    offset += 2;

    size_t pointsSize = (size_t)nPoints * 4;

    if (pointsSize > forestBlobSize - offset) {
      Serial.printf("FOREST ERROR: incomplete polygon %lu\n",
                    (unsigned long)i);
      return false;
    }

    forests[i].nPoints = nPoints;
    forests[i].ptsBase = forestBlob + offset;

    forests[i].minQLat = INT16_MAX;
    forests[i].maxQLat = INT16_MIN;
    forests[i].minQLon = INT16_MAX;
    forests[i].maxQLon = INT16_MIN;

    for (uint16_t k = 0; k < nPoints; k++) {
      int16_t qlat;
      int16_t qlon;

      memcpy(&qlat, forests[i].ptsBase + (size_t)k * 4, 2);
      memcpy(&qlon, forests[i].ptsBase + (size_t)k * 4 + 2, 2);

      if (qlat < forests[i].minQLat) forests[i].minQLat = qlat;
      if (qlat > forests[i].maxQLat) forests[i].maxQLat = qlat;
      if (qlon < forests[i].minQLon) forests[i].minQLon = qlon;
      if (qlon > forests[i].maxQLon) forests[i].maxQLon = qlon;
    }

    if (nPoints > maxForestPoints) {
      maxForestPoints = nPoints;
    }

    offset += pointsSize;
  }

  // The file must end immediately after the final polygon.
  if (offset != forestBlobSize) {
    Serial.printf(
      "FOREST ERROR: parsed %u bytes, but file contains %u bytes\n",
      (unsigned int)offset,
      (unsigned int)forestBlobSize
    );
    return false;
  }

  // Scratch storage: transformed X, transformed Y, intersections.
  if (maxForestPoints > 0) {
    size_t scratchBytes =
      (size_t)maxForestPoints * 3 * sizeof(int32_t);

    forestScratch = (int32_t*)ps_malloc(scratchBytes);

    if (!forestScratch) {
      Serial.printf(
        "FOREST ERROR: cannot allocate %u bytes of scratch memory\n",
        (unsigned int)scratchBytes
      );
      return false;
    }
  }

  Serial.printf(
    "FOREST OK: %lu polygons, maximum %u points per polygon\n",
    (unsigned long)numForests,
    (unsigned int)maxForestPoints
  );

  return true;
}


// ============================================================
// FAST LINE
// ============================================================

static inline void fastLine(
  int16_t x0, int16_t y0,
  int16_t x1, int16_t y1,
  uint16_t color,
  uint16_t* buf
) {
  int16_t dx = abs(x1 - x0);
  int16_t sx = x0 < x1 ? 1 : -1;

  int16_t dy = -abs(y1 - y0);
  int16_t sy = y0 < y1 ? 1 : -1;

  int16_t err = dx + dy;

  while (true) {
    if ((uint16_t)x0 < SCR_W && (uint16_t)y0 < MAP_H) {
      buf[(size_t)y0 * SCR_W + x0] = color;
    }

    if (x0 == x1 && y0 == y1) break;

    int16_t e2 = 2 * err;

    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }

    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

// ============================================================
// FAST DOT
// ============================================================

static inline void fastDot(
  int16_t cx, int16_t cy,
  int16_t r,
  uint16_t color,
  uint16_t* buf
) {
  for (int16_t dy = -r; dy <= r; dy++) {
    for (int16_t dx = -r; dx <= r; dx++) {
      if (dx * dx + dy * dy <= r * r) {
        int16_t px = cx + dx;
        int16_t py = cy + dy;

        if ((uint16_t)px < SCR_W && (uint16_t)py < MAP_H) {
          buf[(size_t)py * SCR_W + px] = color;
        }
      }
    }
  }
}

// ============================================================
// FAST CIRCLE OUTLINE
// ============================================================

void fastCircleOutline(
  int16_t cx, int16_t cy,
  int16_t r,
  uint16_t color,
  uint16_t* buf
) {
  int16_t x = r;
  int16_t y = 0;
  int16_t err = 0;

  while (x >= y) {
    int16_t pts[8][2] = {
      {(int16_t)(cx + x), (int16_t)(cy + y)},
      {(int16_t)(cx + y), (int16_t)(cy + x)},
      {(int16_t)(cx - y), (int16_t)(cy + x)},
      {(int16_t)(cx - x), (int16_t)(cy + y)},
      {(int16_t)(cx - x), (int16_t)(cy - y)},
      {(int16_t)(cx - y), (int16_t)(cy - x)},
      {(int16_t)(cx + y), (int16_t)(cy - x)},
      {(int16_t)(cx + x), (int16_t)(cy - y)}
    };

    for (int i = 0; i < 8; i++) {
      int16_t px = pts[i][0];
      int16_t py = pts[i][1];

      if ((uint16_t)px < SCR_W && (uint16_t)py < MAP_H) {
        buf[(size_t)py * SCR_W + px] = color;
      }
    }

    y++;

    if (err <= 0) {
      err += 2 * y + 1;
    }

    if (err > 0) {
      x--;
      err -= 2 * x + 1;
    }
  }
}

// ============================================================
// ROAD COLORS — PRESERVED FROM YOUR SKETCH
// ============================================================

uint16_t tierColor(uint8_t tier) {
  switch (tier) {
    case 5: return 0x3D7F;       // Waterways
    case 4: return COL_RAIL;     // Railway
    case 3: return COL_PRIMARY;  // Tier 3
    case 2: return COL_PRIMARY;  // Primary
    case 1: return COL_TRUNK;    // Trunk / motorway
    default: return COL_LOCAL;   // Local roads
  }
}

// ============================================================
// DRAW FILLED FOREST POLYGONS
//
// Uses scanline filling. Polygon edges are transformed using the same
// projection as the roads. Scanlines and horizontal spans are clipped
// to the map buffer; the bottom black stripe is never touched.
// ============================================================

void renderForests(
  uint16_t* buf,
  float minVisibleQLat,
  float maxVisibleQLat,
  float minVisibleQLon,
  float maxVisibleQLon,
  int32_t latBasePix,
  int32_t lonBasePix,
  int32_t latStepPix,
  int32_t lonStepPix
) {
  if (!forestBlob || !forests || !forestScratch ||
      numForests == 0 || maxForestPoints == 0) {
    return;
  }

  int32_t* px = forestScratch;
  int32_t* py = forestScratch + maxForestPoints;
  int32_t* intersections = forestScratch + 2 * maxForestPoints;

  for (uint32_t i = 0; i < numForests; i++) {
    if ((i & 0x1F) == 0) {
      readGPS();
    }

    ForestPoly& poly = forests[i];

    if (poly.nPoints < 3) continue;

    // Reject polygons outside the current view.
    if ((float)poly.maxQLat < minVisibleQLat ||
        (float)poly.minQLat > maxVisibleQLat ||
        (float)poly.maxQLon < minVisibleQLon ||
        (float)poly.minQLon > maxVisibleQLon) {
      continue;
    }

    int32_t minY = INT32_MAX;
    int32_t maxY = INT32_MIN;

    // Transform polygon vertices into screen coordinates.
    for (uint16_t k = 0; k < poly.nPoints; k++) {
      int16_t qlat;
      int16_t qlon;

      memcpy(&qlat, poly.ptsBase + k * 4, 2);
      memcpy(&qlon, poly.ptsBase + k * 4 + 2, 2);

      int64_t xFixed =
        (int64_t)lonBasePix + (int64_t)qlon * lonStepPix;

      int64_t yFixed =
        (int64_t)latBasePix + (int64_t)qlat * latStepPix;

      int64_t xValue = (int64_t)CX + (xFixed >> 16);
      int64_t yValue = (int64_t)CY - (yFixed >> 16);

      // Keep values bounded before using them in scanline arithmetic.
      if (xValue < -1000000LL) xValue = -1000000LL;
      if (xValue >  1000000LL) xValue =  1000000LL;
      if (yValue < -1000000LL) yValue = -1000000LL;
      if (yValue >  1000000LL) yValue =  1000000LL;

      px[k] = (int32_t)xValue;
      py[k] = (int32_t)yValue;

      if (py[k] < minY) minY = py[k];
      if (py[k] > maxY) maxY = py[k];
    }

    if (maxY < 0 || minY >= MAP_H) continue;

    if (minY < 0) minY = 0;
    if (maxY >= MAP_H) maxY = MAP_H - 1;

    // Fill each visible scanline using pairs of edge intersections.
    for (int32_t y = minY; y <= maxY; y++) {
      uint32_t count = 0;

      for (uint16_t k = 0, j = poly.nPoints - 1;
           k < poly.nPoints;
           j = k++) {

        int32_t x1 = px[j];
        int32_t y1 = py[j];
        int32_t x2 = px[k];
        int32_t y2 = py[k];

        // Half-open edge rule avoids counting shared vertices twice.
        if ((y1 <= y && y2 > y) ||
            (y2 <= y && y1 > y)) {

          int64_t numerator =
            (int64_t)(y - y1) * (int64_t)(x2 - x1);

          int32_t xCross =
            x1 + (int32_t)(numerator / (int64_t)(y2 - y1));

          if (count < poly.nPoints) {
            intersections[count++] = xCross;
          }
        }
      }

      // Insertion-sort intersections from left to right.
      for (uint32_t a = 1; a < count; a++) {
        int32_t value = intersections[a];
        int32_t b = (int32_t)a - 1;

        while (b >= 0 && intersections[b] > value) {
          intersections[b + 1] = intersections[b];
          b--;
        }

        intersections[b + 1] = value;
      }

      for (uint32_t a = 0; a + 1 < count; a += 2) {
        int32_t xStart = intersections[a];
        int32_t xEnd = intersections[a + 1];

        if (xEnd < 0 || xStart >= SCR_W) continue;

        if (xStart < 0) xStart = 0;
        if (xEnd >= SCR_W) xEnd = SCR_W - 1;

        size_t rowOffset = (size_t)y * SCR_W;

        for (int32_t x = xStart; x <= xEnd; x++) {
          buf[rowOffset + x] = COL_FOREST;
        }
      }
    }
  }
}

// ============================================================
// RENDER ROADS OVER THE FORESTS
// ============================================================

void renderRoads(uint16_t* buf) {
  // Clear only the map area, not the bottom mask.
  for (int i = 0; i < SCR_W * MAP_H; i++) {
    buf[i] = COL_BG;
  }

  float minLatF = (float)(bboxMinLat_e7 / 1e7);
  float minLonF = (float)(bboxMinLon_e7 / 1e7);
  float cosLatF = cosf(vehLatF * (float)PI / 180.0f);

  float safeCos = fabsf(cosLatF) < 0.01f ? 0.01f : fabsf(cosLatF);

  float minVisibleLat =
    vehLatF + ((float)CY - (float)MAP_H) / PIXELS_PER_DEG;

  float maxVisibleLat =
    vehLatF + (float)CY / PIXELS_PER_DEG;

  float minVisibleLon =
    vehLonF - (float)CX / (PIXELS_PER_DEG * safeCos);

  float maxVisibleLon =
    vehLonF + (float)(SCR_W - CX) /
    (PIXELS_PER_DEG * safeCos);

  float minVisibleQLat =
    (minVisibleLat - minLatF) * (float)latScale;

  float maxVisibleQLat =
    (maxVisibleLat - minLatF) * (float)latScale;

  float minVisibleQLon =
    (minVisibleLon - minLonF) * (float)lonScale;

  float maxVisibleQLon =
    (maxVisibleLon - minLonF) * (float)lonScale;

  float latMarginQ =
    2.0f * (float)latScale / PIXELS_PER_DEG;

  float lonMarginQ =
    2.0f * (float)lonScale /
    (PIXELS_PER_DEG * safeCos);

  minVisibleQLat -= latMarginQ;
  maxVisibleQLat += latMarginQ;
  minVisibleQLon -= lonMarginQ;
  maxVisibleQLon += lonMarginQ;

  int32_t latBasePix =
    (int32_t)((minLatF - vehLatF) *
              PIXELS_PER_DEG * 65536.0f);

  int32_t lonBasePix =
    (int32_t)((minLonF - vehLonF) *
              PIXELS_PER_DEG * cosLatF * 65536.0f);

  int32_t latStepPix =
    (int32_t)((PIXELS_PER_DEG / (float)latScale) * 65536.0f);

  int32_t lonStepPix =
    (int32_t)((PIXELS_PER_DEG * cosLatF /
              (float)lonScale) * 65536.0f);

  // IMPORTANT DRAW ORDER:
  // 1. Background
  // 2. Filled forests
  // 3. Road, railway and waterway lines
  renderForests(
    buf,
    minVisibleQLat,
    maxVisibleQLat,
    minVisibleQLon,
    maxVisibleQLon,
    latBasePix,
    lonBasePix,
    latStepPix,
    lonStepPix
  );

  for (uint32_t i = 0; i < numSegs; i++) {
    if ((i & 0x0F) == 0) {
      readGPS();
    }

    RoadSeg& s = segs[i];

    if (s.nPoints < 2) continue;

    if ((float)s.maxQLat < minVisibleQLat ||
        (float)s.minQLat > maxVisibleQLat ||
        (float)s.maxQLon < minVisibleQLon ||
        (float)s.minQLon > maxVisibleQLon) {
      continue;
    }

    uint16_t color = tierColor(s.tier);

    int16_t prevX = 0;
    int16_t prevY = 0;
    bool havePrev = false;

    for (uint16_t k = 0; k < s.nPoints; k++) {
      int16_t qlat;
      int16_t qlon;

      memcpy(&qlat, s.ptsBase + k * 4, 2);
      memcpy(&qlon, s.ptsBase + k * 4 + 2, 2);

      int32_t xPix =
        CX + (int32_t)(((int64_t)lonBasePix +
                        (int64_t)qlon * lonStepPix) >> 16);

      int32_t yPix =
        CY - (int32_t)(((int64_t)latBasePix +
                        (int64_t)qlat * latStepPix) >> 16);

      if (xPix < -1000) xPix = -1000;
      else if (xPix > SCR_W + 1000) xPix = SCR_W + 1000;

      if (yPix < -1000) yPix = -1000;
      else if (yPix > MAP_H + 1000) yPix = MAP_H + 1000;

      int16_t sx = (int16_t)xPix;
      int16_t sy = (int16_t)yPix;

      if (havePrev) {
        bool aIn =
          prevX >= -8 && prevX <= SCR_W + 8 &&
          prevY >= -8 && prevY <= MAP_H + 8;

        bool bIn =
          sx >= -8 && sx <= SCR_W + 8 &&
          sy >= -8 && sy <= MAP_H + 8;

        if (aIn || bIn) {
          fastLine(prevX, prevY, sx, sy, color, buf);
        }
      }

      prevX = sx;
      prevY = sy;
      havePrev = true;
    }
  }
}

// ============================================================
// DRAW MAP
// ============================================================

void drawMap() {
  if (!frameBuffer) return;

  uint16_t* buf = frameBuffer;

  renderRoads(buf);

  // Red without a recent valid fix; cyan when GPS is fixed.
  bool gpsFixNow =
    gps.location.isValid() && gps.location.age() < 3000;

  uint16_t dotColor = gpsFixNow ? COL_DOT : COL_RED;

  fastDot(CX, CY, 6, dotColor, buf);
  fastCircleOutline(CX, CY, 10, dotColor, buf);

  gfx->draw16bitRGBBitmap(0, 0, buf, SCR_W, MAP_H);

#if SHOW_SAT_COUNT
  int n = gps.satellites.isValid()
    ? (int)gps.satellites.value() : 0;

  char satText[8];
  snprintf(satText, sizeof(satText), "%2d", n);

  uint16_t satColor =
    (n >= 6) ? COL_GREEN :
    (n >= 4) ? COL_YELLOW : COL_RED;

  gfx->setTextSize(2);
  gfx->setTextColor(COL_HUD_UNIT, COL_BG);
  gfx->setCursor(185, 8);
  gfx->print("SAT ");
  gfx->setTextColor(satColor, COL_BG);
  gfx->print(satText);
#endif

#if SHOW_SPEED
  updateSpeedDirect();
#endif

  // Keep the bottom 25-pixel stripe black.
  gfx->fillRect(
    0,
    SCR_H - BOTTOM_BLACK_STRIPE_H,
    SCR_W,
    BOTTOM_BLACK_STRIPE_H,
    0x0000
  );

//  lastMapDraw = millis();
}

// ============================================================
// TOUCH
// ============================================================

bool readTouchPoint(uint16_t &x, uint16_t &y) {
  Wire.beginTransmission(TOUCH_ADDR);
  Wire.write((uint8_t)0x02);

  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((uint8_t)TOUCH_ADDR, (uint8_t)5) != 5) {
    return false;
  }

  uint8_t points = Wire.read() & 0x0F;
  uint8_t xh = Wire.read();
  uint8_t xl = Wire.read();
  uint8_t yh = Wire.read();
  uint8_t yl = Wire.read();

  if (points == 0) return false;

  x = ((uint16_t)(xh & 0x0F) << 8) | xl;
  y = ((uint16_t)(yh & 0x0F) << 8) | yl;

  if (x >= SCR_W || y >= SCR_H) return false;

  return true;
}

void handleTouch() {
  uint16_t x = 0, y = 0;
  bool down = readTouchPoint(x, y);

  unsigned long now = millis();

  if (down && !touchWasDown &&
      now - lastTouchAction >= TOUCH_DEBOUNCE_MS) {

    if (y < MAP_H) {
      if (x < 160) {
        PIXELS_PER_DEG *= ZOOM_OUT_FACTOR;
      } else if (x > 320) {
        PIXELS_PER_DEG *= ZOOM_IN_FACTOR;
      } else {
        PIXELS_PER_DEG = 18000.0f;
      }

      if (PIXELS_PER_DEG < ZOOM_MIN) {
        PIXELS_PER_DEG = ZOOM_MIN;
      }

      if (PIXELS_PER_DEG > ZOOM_MAX) {
        PIXELS_PER_DEG = ZOOM_MAX;
      }

      lastZoomChange = now;
      needsRedraw = true;
      lastTouchAction = now;
    }
  }

  touchWasDown = down;
}

// ============================================================
// SETUP
// ============================================================


void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("Starting map...");

  EEPROM.begin(EEPROM_SIZE);

  PIXELS_PER_DEG = loadZoomFromEEPROM();
  lastSavedZoom = PIXELS_PER_DEG;

  Wire.begin(TOUCH_SDA, TOUCH_SCL, 400000);

  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, LOW);

  if (!gfx->begin()) {
    Serial.println("ERROR: display initialization failed");
    while (true) delay(1000);
  }

  digitalWrite(GFX_BL, HIGH);

  gfx->fillScreen(COL_BG);

  gfx->fillRect(
    0,
    SCR_H - BOTTOM_BLACK_STRIPE_H,
    SCR_W,
    BOTTOM_BLACK_STRIPE_H,
    0x0000
  );

  frameBuffer = (uint16_t*)ps_malloc(
    (size_t)SCR_W * MAP_H * sizeof(uint16_t)
  );

  if (!frameBuffer) {
    Serial.println("ERROR: framebuffer allocation failed");
    while (true) delay(1000);
  }

  // The road map is mandatory.
  Serial.println("Loading road map...");

  if (!loadVectorMap("/chernik.vec")) {
    Serial.println("ERROR: cannot load /chernik.vec");
    while (true) delay(1000);
  }

  if (!parseVectorMap()) {
    Serial.println("ERROR: cannot parse /chernik.vec");
    while (true) delay(1000);
  }

  Serial.println("Road map loaded successfully.");

  // Forest data is optional.
  Serial.println("Loading forest map...");

  if (!loadForestMap("/forests.vec")) {
    Serial.println("FOREST ERROR: cannot load /forests.vec");
  } else {
    if (!parseForestMap()) {
      Serial.println("Forest parsing failed; continuing with roads only.");

      if (forestScratch) {
        free(forestScratch);
        forestScratch = nullptr;
      }

      if (forests) {
        free(forests);
        forests = nullptr;
      }

      if (forestBlob) {
        free(forestBlob);
        forestBlob = nullptr;
      }

      forestBlobSize = 0;
      numForests = 0;
      maxForestPoints = 0;
    }
  }

  GPSSerial.setRxBufferSize(4096);
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, -1);

#if SHOW_SPEED
  lastShownSpeed = -999;
  updateSpeedDirect();
#endif

  drawMap();
  needsRedraw = false;

  Serial.println("Map startup complete.");
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  readGPS();

  static bool lastDotFixState = false;

  bool dotFixState =
    gps.location.isValid() && gps.location.age() < 3000;

  if (dotFixState != lastDotFixState) {
    lastDotFixState = dotFixState;
    needsRedraw = true;
  }

  handleTouch();

  if (gps.location.isValid() && gps.location.isUpdated()) {
    float nlat = (float)gps.location.lat();
    float nlon = (float)gps.location.lng();

    bool movedFar =
      !haveFix ||
      fabsf(nlat - vehLatF) > 0.00002f ||
      fabsf(nlon - vehLonF) > 0.00002f;

    if (movedFar) {
      float minLat = bboxMinLat_e7 / 1e7f;
      float maxLat = bboxMaxLat_e7 / 1e7f;
      float minLon = bboxMinLon_e7 / 1e7f;
      float maxLon = bboxMaxLon_e7 / 1e7f;

      if (nlat >= minLat && nlat <= maxLat &&
          nlon >= minLon && nlon <= maxLon) {

        vehLatF = nlat;
        vehLonF = nlon;

        haveFix = true;
        needsRedraw = true;
      }
    }
  }

  if (fabsf(PIXELS_PER_DEG - lastSavedZoom) > 0.5f &&
      millis() - lastZoomChange > 1000) {

    saveZoomToEEPROM(PIXELS_PER_DEG);
    lastSavedZoom = PIXELS_PER_DEG;
  }

  if (needsRedraw) {
   //   millis() - lastMapDraw >= MAP_REDRAW_INTERVAL_MS) {

    needsRedraw = false;
    drawMap();
  }

#if SHOW_SPEED
  updateSpeedDirect();
#endif
}