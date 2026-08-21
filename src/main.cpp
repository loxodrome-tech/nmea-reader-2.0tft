/*
  NMEA 0183 reader with 2.0" 7-pin SPI TFT (ST7789 240x320) on ESP32 WROOM32
  PlatformIO / Arduino framework

  Display module wiring (7 pins):
    VCC -> 3.3 V   (NOT 5 V)
    GND -> GND
    SCL -> SCK
    SDA -> MOSI
    CS, DC, RST -> free GPIOs

  NMEA wiring:
    UART2 RX <- TX of the NMEA device   (GPIO 16)
    UART2 TX -> RX of the NMEA device   (GPIO 17, only if the device needs it)
    GND shared with the NMEA device

  The screen lists each sensor category (GPS, AIS, Heading, Gyro, Velocity,
  Radar, Sounder, Weather, Xducer) currently speaking on the bus, classified
  from the sentence formatter (falling back to the talker ID for unlisted
  formatters). A category drops off the list a few seconds after it stops
  transmitting — only sensors actually being detected are shown.

  A rotary encoder scrolls a highlighted selection up/down the list. A short
  click opens a detail screen for the selected sensor, showing every
  distinct sentence type it's currently sending (one drops off on its own a
  few seconds after it stops arriving); click again to return to the list.
  A long press (from the list) opens a live console of the raw NMEA lines
  as they arrive, on the TFT itself; a short click there pauses/resumes it
  (it keeps buffering underneath while paused), and a long press returns
  to the list.

  Encoder wiring (KY-040 style module):
    CLK -> GPIO32, DT -> GPIO33, SW -> GPIO4, + -> 3.3V, GND -> GND

  The UART baud rate isn't fixed: at boot (and if the bus ever goes fully
  quiet) it cycles through the common NMEA 0183 rates, listening for lines
  whose checksum actually validates, and locks onto the first one that does.

  Pins come from build_flags in platformio.ini.
  The #ifndef fallbacks below only apply if you compile without those flags.
*/

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <string.h>
#include "themys_logo.h"

#ifndef PIN_SCL
#define PIN_SCL   18    // gray   -> SCK
#endif
#ifndef PIN_SDA
#define PIN_SDA   23    // green  -> MOSI
#endif
#ifndef PIN_CS
#define PIN_CS     5    // salmon
#endif
#ifndef PIN_DC
#define PIN_DC    27    // white
#endif
#ifndef PIN_RST
#define PIN_RST   26    // brown
#endif
#ifndef SPI_HZ
#define SPI_HZ  20000000
#endif

#ifndef GPS_RX_PIN
#define GPS_RX_PIN  16
#endif
#ifndef GPS_TX_PIN
#define GPS_TX_PIN  17
#endif

#ifndef ENC_CLK_PIN
#define ENC_CLK_PIN  32
#endif
#ifndef ENC_DT_PIN
#define ENC_DT_PIN   33
#endif
#ifndef ENC_SW_PIN
#define ENC_SW_PIN   4
#endif

#define PIN_MISO  -1    // the display doesn't use it
#define TFT_W    240    // panel's native resolution (portrait)
#define TFT_H    320

// How long without a sentence from a category before it's dropped from the list
#define SENSOR_TIMEOUT_MS  3000
#define MAX_SENSORS  11
#define MAX_TYPES_PER_SENSOR  9  // GPS alone can send GGA/RMC/GLL/VTG/GSA/GSV/ZDA/GNS/DTM
#define ROW_Y0  34
#define ROW_H_MIN  18   // compact rows once many sensors are listed at once
#define ROW_H_MAX  56   // how tall a row is allowed to grow when few sensors are listed

// Encoder button: how long a hold counts as a long press (opens the raw
// console), and the minimum hold time for a release to count as a real
// short click rather than contact bounce.
#define LONG_PRESS_MS  800
#define SW_DEBOUNCE_MS  30

// Baud auto-detect: how long to listen on each candidate rate, how many
// checksum-valid lines are needed to trust it, and how long the bus has to
// stay silent while running before we assume the rate changed and re-scan.
#define BAUD_CANDIDATE_WINDOW_MS  1000
#define BAUD_LOCK_THRESHOLD  2
#define BAUD_SILENCE_RESCAN_MS  10000

Adafruit_ST7789 tft = Adafruit_ST7789(PIN_CS, PIN_DC, PIN_RST);
HardwareSerial &gpsSerial = Serial2;

// Common NMEA 0183 rates, most likely first, so a typical bus locks on fast.
static const uint32_t BAUD_CANDIDATES[] = { 4800, 9600, 38400, 19200, 57600, 115200 };
static const uint8_t NUM_BAUD_CANDIDATES = sizeof(BAUD_CANDIDATES) / sizeof(BAUD_CANDIDATES[0]);

static bool baudLocked = false;
static uint8_t baudCandidateIdx = 0;
static uint32_t baudWindowStartMs = 0;
static uint8_t baudValidCount = 0;
// Updated only when a structurally plausible NMEA line comes in (not on
// every raw byte): if the source's baud rate changes while it keeps
// transmitting, the UART still receives a continuous stream of garbled
// bytes, so byte-level "silence" never happens and a byte-based timeout
// would never re-scan. Going stale on *valid-looking lines* instead catches
// that case too, not just an actually-quiet bus.
static uint32_t lastValidLineMs = 0;
static char lineBuf[96];
static uint8_t lineLen = 0;

static const char CAT_GPS[]      = "GPS";
static const char CAT_AIS[]      = "AIS";
static const char CAT_HEADING[]  = "Heading";
static const char CAT_GYRO[]     = "Gyro";
static const char CAT_VELOCITY[] = "Velocity";
static const char CAT_RADAR[]    = "Radar";
static const char CAT_SOUNDER[]  = "Sounder";
static const char CAT_WEATHER[]  = "Weather";
static const char CAT_XDUCER[]   = "Xducer";
static const char CAT_UNKNOWN[]  = "Unknown";

// Sentence formatters (the 3 letters after the talker ID) mapped to a category.
// Checked first: most instruments on a real bus multiplex through a generic
// talker ID (e.g. "II"), so the formatter is the reliable signal of content.
struct FormatterInfo { const char *type; const char *category; };
static const FormatterInfo TYPE_CATEGORY[] = {
  {"GGA", CAT_GPS},     {"RMC", CAT_GPS},     {"GLL", CAT_GPS},    {"VTG", CAT_GPS},
  {"GSA", CAT_GPS},     {"GSV", CAT_GPS},     {"ZDA", CAT_GPS},    {"GNS", CAT_GPS},
  {"DTM", CAT_GPS},
  {"VDM", CAT_AIS},     {"VDO", CAT_AIS},
  {"HDG", CAT_HEADING}, {"HDM", CAT_HEADING}, {"HDT", CAT_HEADING},
  {"ROT", CAT_GYRO},    {"THS", CAT_GYRO},    {"RSA", CAT_GYRO},
  {"VHW", CAT_VELOCITY},{"VBW", CAT_VELOCITY},{"VLW", CAT_VELOCITY},
  {"TTM", CAT_RADAR},   {"RSD", CAT_RADAR},   {"TLB", CAT_RADAR},  {"TLL", CAT_RADAR},
  {"DBT", CAT_SOUNDER}, {"DPT", CAT_SOUNDER}, {"DBK", CAT_SOUNDER},{"DBS", CAT_SOUNDER},
  {"MWV", CAT_WEATHER}, {"MWD", CAT_WEATHER}, {"MTW", CAT_WEATHER},{"MDA", CAT_WEATHER},
  {"VWR", CAT_WEATHER}, {"VWT", CAT_WEATHER},
  {"XDR", CAT_XDUCER},
};

// Talker ID fallback, used only when the formatter above isn't recognized.
struct TalkerInfo { const char *id; const char *category; };
static const TalkerInfo TALKER_CATEGORY[] = {
  {"GP", CAT_GPS},      {"GN", CAT_GPS},      {"GL", CAT_GPS}, {"GA", CAT_GPS},
  {"GB", CAT_GPS},      {"GQ", CAT_GPS},      {"GI", CAT_GPS},
  {"AI", CAT_AIS},      {"AB", CAT_AIS},
  {"HC", CAT_HEADING},
  {"HE", CAT_GYRO},     {"HN", CAT_GYRO},
  {"VW", CAT_VELOCITY}, {"VD", CAT_VELOCITY}, {"VM", CAT_VELOCITY},
  {"RA", CAT_RADAR},
  {"SD", CAT_SOUNDER},  {"SS", CAT_SOUNDER},
  {"WI", CAT_WEATHER},
  {"YC", CAT_XDUCER},   {"YV", CAT_XDUCER},   {"YX", CAT_XDUCER},
};

struct SensorRow {
  const char *category;
  uint32_t lastSeen;
  char types[MAX_TYPES_PER_SENSOR][4];       // distinct sentence types currently being received (e.g. "RMC")
  uint32_t typeLastSeen[MAX_TYPES_PER_SENSOR]; // when each one last arrived, to expire it individually
  uint8_t typeCount;
};

static SensorRow rows[MAX_SENSORS];
static uint8_t rowCount = 0;

// Rotary encoder: rotating moves the highlighted row in the list; a short
// click opens/closes a detail screen for the selected sensor's sentence
// types; a long press (from the list) opens a live raw-NMEA console.
enum ViewMode { VIEW_LIST, VIEW_DETAIL, VIEW_CONSOLE };
static ViewMode currentView = VIEW_LIST;
static uint8_t selectedRow = 0;
static const char *detailCategory = nullptr; // which category the detail screen is showing

// Full quadrature decode (both pins, on every edge) instead of sampling DT
// only on CLK's falling edge — the single-edge method missed or mis-read
// steps at speed because DT isn't guaranteed to have settled yet when the
// ISR for CLK fires. This table only accumulates at genuine transitions, so
// contact bounce mostly cancels out. Written only from the ISR; read/cleared
// from loop() with interrupts held off.
static volatile int8_t encDelta = 0;
static volatile uint8_t encRawState = 0;
static const int8_t ENC_TABLE[16] = {
  0, -1, 1, 0,
  1, 0, 0, -1,
  -1, 0, 0, 1,
  0, 1, -1, 0,
};

void IRAM_ATTR onEncoderChange() {
  encRawState = (encRawState << 2) | (digitalRead(ENC_CLK_PIN) << 1) | digitalRead(ENC_DT_PIN);
  encDelta += ENC_TABLE[encRawState & 0x0F];
}

// How long to wait, after the last sensor was added/removed, before actually
// resizing the rows to fit the new count. Several sensors often get detected
// within the same second at boot; without this, each one arriving would
// trigger its own resize, visibly growing the rows and then shrinking them
// back down as more show up. Waiting for a quiet moment collapses that into
// a single, final layout instead.
#define LAYOUT_SETTLE_MS  900
static bool layoutPending = false;
static uint32_t lastRowSetChangeMs = 0;

static const char *categoryFor(const char *talkerId, const char *type) {
  for (size_t i = 0; i < sizeof(TYPE_CATEGORY) / sizeof(TYPE_CATEGORY[0]); i++) {
    if (strcmp(type, TYPE_CATEGORY[i].type) == 0) return TYPE_CATEGORY[i].category;
  }
  for (size_t i = 0; i < sizeof(TALKER_CATEGORY) / sizeof(TALKER_CATEGORY[0]); i++) {
    if (talkerId[0] == TALKER_CATEGORY[i].id[0] && talkerId[1] == TALKER_CATEGORY[i].id[1]) {
      return TALKER_CATEGORY[i].category;
    }
  }
  return CAT_UNKNOWN;
}

#define BAUD_BADGE_X0  180  // left edge of the top-right baud-rate badge area

#define SPLASH_DURATION_MS  2800
#define SPLASH_BAR_W  200
#define SPLASH_BAR_H  8

// Logo on a white card, with a fake progress bar filling underneath — purely
// cosmetic (not tied to any real init step), just to give the splash more
// time on screen without it feeling like a stall.
static void showSplashScreen() {
  tft.fillScreen(ST77XX_WHITE); // matches the logo's white background, no visible seam

  int16_t groupH = THEMYS_LOGO_H + 24 + SPLASH_BAR_H;
  int16_t logoX = (tft.width() - THEMYS_LOGO_W) / 2;
  int16_t logoY = (tft.height() - groupH) / 2;
  tft.drawRGBBitmap(logoX, logoY, themysLogo, THEMYS_LOGO_W, THEMYS_LOGO_H);

  int16_t barX = (tft.width() - SPLASH_BAR_W) / 2;
  int16_t barY = logoY + THEMYS_LOGO_H + 24;
  uint16_t track = tft.color565(224, 224, 224);
  uint16_t fill = tft.color565(232, 65, 44); // matches the logo's arrow

  tft.fillRoundRect(barX, barY, SPLASH_BAR_W, SPLASH_BAR_H, SPLASH_BAR_H / 2, track);

  uint32_t start = millis();
  int16_t lastFilled = 0;
  while (true) {
    uint32_t elapsed = millis() - start;
    if (elapsed > SPLASH_DURATION_MS) elapsed = SPLASH_DURATION_MS;
    int16_t filled = (int32_t)SPLASH_BAR_W * elapsed / SPLASH_DURATION_MS;
    if (filled > lastFilled) {
      tft.fillRoundRect(barX, barY, filled, SPLASH_BAR_H, SPLASH_BAR_H / 2, fill);
      lastFilled = filled;
    }
    if (elapsed >= SPLASH_DURATION_MS) break;
    delay(20);
  }
  delay(200); // brief hold once the bar reads full
}

static void drawHeader() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(8, 8);
  tft.print("NMEA MONITOR");
  tft.drawFastHLine(0, 30, tft.width(), ST77XX_WHITE);
}

static void clearBaudBadge() {
  tft.fillRect(BAUD_BADGE_X0, 0, tft.width() - BAUD_BADGE_X0, 30, ST77XX_BLACK);
}

// Shows the currently-locked baud rate top-right in the header, e.g. "9600 bps"
// — same text size as the title.
static void drawBaudBadge(uint32_t baud) {
  clearBaudBadge();
  char buf[16];
  snprintf(buf, sizeof(buf), "%lu bps", (unsigned long)baud);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(tft.width() - (int16_t)strlen(buf) * 12 - 8, 8);
  tft.print(buf);
}

static void clearContentArea() {
  tft.fillRect(0, 32, tft.width(), tft.height() - 32, ST77XX_BLACK);
}

// Rows split the available vertical space evenly among the sensors currently
// listed, and grow their text with it — so a handful of sensors fill the
// screen instead of clumping in a few small lines at the top.
static int16_t currentRowH() {
  if (rowCount == 0) return ROW_H_MIN;
  int16_t h = (tft.height() - ROW_Y0) / rowCount;
  if (h < ROW_H_MIN) h = ROW_H_MIN;
  if (h > ROW_H_MAX) h = ROW_H_MAX;
  return h;
}

// dotRadiusFor[size] — the dot grows with the font size (index 0 unused)
static const int16_t dotRadiusFor[4] = { 0, 4, 7, 10 };

// Longest category name that could ever appear, used to size for the worst
// case up front instead of the current live content — see worstCaseChars().
static const char *const ALL_CATEGORIES[] = {
  CAT_GPS, CAT_AIS, CAT_HEADING, CAT_GYRO, CAT_VELOCITY,
  CAT_RADAR, CAT_SOUNDER, CAT_WEATHER, CAT_XDUCER, CAT_UNKNOWN,
};

// The character budget a row needs: just the longest possible category name.
// Fixed for the life of the program, so every row shares one size and it
// never has to change just because a row's own name happens to be shorter.
static int16_t worstCaseChars() {
  static int16_t cached = -1;
  if (cached >= 0) return cached;
  int16_t maxNameLen = 0;
  for (size_t i = 0; i < sizeof(ALL_CATEGORIES) / sizeof(ALL_CATEGORIES[0]); i++) {
    int16_t len = strlen(ALL_CATEGORIES[i]);
    if (len > maxNameLen) maxNameLen = len;
  }
  cached = maxNameLen;
  return cached;
}

// One shared layout for every row: same font size, same dot size, same row
// height. Recomputed only when the row set changes (a sensor appears or
// disappears) — never when an existing row's sentence list simply grows —
// so the screen stays put instead of continuously "settling."  The size is
// the largest that fits the worst-case row on a single line, and it grows
// as fewer sensors are listed, since gRowH grows too.
static int16_t gRowH = ROW_H_MIN, gDotR = 4;
static uint8_t gFontSize = 1;

static void refreshLayoutMetrics() {
  gRowH = currentRowH();
  int16_t worstChars = worstCaseChars();

  for (uint8_t size = 3; size >= 1; size--) {
    int16_t dotR = dotRadiusFor[size];
    int16_t textX = dotR * 2 + 12;
    int16_t availableW = tft.width() - textX - 4;
    bool fits = (size * 8 + 4 <= gRowH) && (worstChars * size * 6 <= availableW);
    if (fits) {
      gFontSize = size;
      gDotR = dotR;
      return;
    }
    if (size == 1) break; // nothing fits comfortably: fall back to the smallest size
  }
  gFontSize = 1;
  gDotR = dotRadiusFor[1];
}

// Row layout, one line per sensor: [dot] Category, with a border around the
// row the encoder currently has highlighted.
static void drawRow(uint8_t i) {
  int16_t y = ROW_Y0 + i * gRowH;
  int16_t textX = gDotR * 2 + 12;
  int16_t textY = y + (gRowH - gFontSize * 8) / 2;

  tft.fillRect(0, y, tft.width(), gRowH, ST77XX_BLACK);
  tft.fillCircle(gDotR + 4, y + gRowH / 2, gDotR, ST77XX_GREEN);

  tft.setTextSize(gFontSize);
  tft.setCursor(textX, textY);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(rows[i].category);

  if (i == selectedRow) {
    tft.drawRect(1, y + 1, tft.width() - 2, gRowH - 2, ST77XX_CYAN);
    tft.drawRect(0, y, tft.width(), gRowH, ST77XX_CYAN);
  }
}

static int8_t findRow(const char *category) {
  for (uint8_t i = 0; i < rowCount; i++) {
    if (strcmp(rows[i].category, category) == 0) return i;
  }
  return -1;
}

// How much vertical space the list actually occupied last time it was drawn.
// Used to blank only the leftover strip when the list shrinks, instead of
// blacking out the whole content area up front — that upfront clear is what
// caused the visible flash/flicker on every update.
static int16_t prevOccupiedH = 0;

// Redraws the whole list from the current rows[] contents, sized as one
// uniform layout. Needed whenever a row is added, removed, or the shared
// size needs to change. Each row erases only its own rect as it redraws
// itself, so there's no full-screen blank frame — any area left over from a
// taller previous layout is blanked separately, after the new rows are painted.
static void redrawAllRows() {
  if (selectedRow >= rowCount) selectedRow = rowCount > 0 ? rowCount - 1 : 0;

  if (rowCount == 0) {
    clearContentArea();
    prevOccupiedH = 0;
    return;
  }
  refreshLayoutMetrics();
  for (uint8_t i = 0; i < rowCount; i++) {
    drawRow(i);
  }
  int16_t occupiedH = rowCount * gRowH;
  if (occupiedH < prevOccupiedH) {
    tft.fillRect(0, ROW_Y0 + occupiedH, tft.width(), prevOccupiedH - occupiedH, ST77XX_BLACK);
  }
  prevOccupiedH = occupiedH;
}

// Records a sentence type as currently received: refreshes its timestamp if
// already known, or adds it if new. Doesn't draw anything — the list view
// doesn't show it; only the detail screen does, and only when it's the one
// currently open. Returns whether the list actually grew (a new entry).
static bool addTypeSilently(uint8_t i, const char *type) {
  for (uint8_t t = 0; t < rows[i].typeCount; t++) {
    if (strcmp(rows[i].types[t], type) == 0) {
      rows[i].typeLastSeen[t] = millis();
      return false; // already known, just refreshed
    }
  }
  if (rows[i].typeCount >= MAX_TYPES_PER_SENSOR) return false; // list full, keep what we have
  strcpy(rows[i].types[rows[i].typeCount], type);
  rows[i].typeLastSeen[rows[i].typeCount] = millis();
  rows[i].typeCount++;
  return true;
}

// Drops any sentence type on row i that hasn't arrived in a while — a
// sensor can keep transmitting overall while one of its formatters stops
// (e.g. GSA turned off in a simulator), and that shouldn't linger forever
// in its detail view. Returns whether anything was dropped.
static bool pruneStaleTypes(uint8_t i) {
  uint32_t now = millis();
  bool removedAny = false;
  for (uint8_t t = 0; t < rows[i].typeCount; ) {
    if (now - rows[i].typeLastSeen[t] >= SENSOR_TIMEOUT_MS) {
      for (uint8_t k = t; k < rows[i].typeCount - 1; k++) {
        strcpy(rows[i].types[k], rows[i].types[k + 1]);
        rows[i].typeLastSeen[k] = rows[i].typeLastSeen[k + 1];
      }
      rows[i].typeCount--;
      removedAny = true;
    } else {
      t++;
    }
  }
  return removedAny;
}

// Full-screen detail for one sensor: its name and every distinct sentence
// type seen from it so far. Redrawn from scratch each time it's shown or
// its list grows — there's only one row on screen here, so no need for the
// partial-redraw tricks the list view uses.
static void drawDetailView(uint8_t idx) {
  clearContentArea();

  tft.fillCircle(300, 48, 8, ST77XX_GREEN);
  tft.setTextSize(3);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(8, 40);
  tft.print(rows[idx].category);

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(8, 76);
  tft.print("Sentences received:");

  int16_t x = 8, y = 96;
  const int16_t maxX = tft.width() - 8;
  const int16_t lineH = 22;
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_YELLOW);
  for (uint8_t t = 0; t < rows[idx].typeCount; t++) {
    String token = (t > 0 ? String(", ") : String("")) + rows[idx].types[t];
    int16_t w = token.length() * 12;
    if (x != 8 && x + w > maxX) {
      x = 8;
      y += lineH;
    }
    tft.setCursor(x, y);
    tft.print(token);
    x += w;
  }
  if (rows[idx].typeCount == 0) {
    tft.setCursor(8, y);
    tft.print("(none yet)");
  }

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(8, tft.height() - 14);
  tft.print("Click the encoder to go back");

  // The detail screen uses far more of the content area than the list
  // usually does. Recording that as the "previously occupied" height means
  // the next redrawAllRows() (when going back to the list) will blank all
  // the way down to here, instead of leaving leftover detail text below
  // wherever the list's rows happen to end this time.
  prevOccupiedH = tft.height() - 32;
}

// Rolling history of the last raw NMEA lines, for the on-screen console.
// Truncated to what fits the screen width — the full untruncated line is
// still available over USB serial for anyone who needs it complete.
#define CONSOLE_LINES     12
#define CONSOLE_LINE_LEN  50
static char consoleBuf[CONSOLE_LINES][CONSOLE_LINE_LEN + 1];
static uint8_t consoleCount = 0;
static bool consoleDirty = false;  // set when a new line arrives; drives a throttled redraw
static bool consolePaused = false; // frozen for reading; lines keep buffering underneath

static void pushConsoleLine(const char *line, uint8_t len) {
  if (consoleCount < CONSOLE_LINES) {
    consoleCount++;
  } else {
    for (uint8_t i = 0; i < CONSOLE_LINES - 1; i++) {
      memcpy(consoleBuf[i], consoleBuf[i + 1], sizeof(consoleBuf[i]));
    }
  }
  uint8_t copyLen = len < CONSOLE_LINE_LEN ? len : CONSOLE_LINE_LEN;
  memcpy(consoleBuf[consoleCount - 1], line, copyLen);
  consoleBuf[consoleCount - 1][copyLen] = '\0';
  consoleDirty = true;
}

// Live view of the raw NMEA stream, most recent line at the bottom.
static void drawConsoleView() {
  clearContentArea();

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN);
  int16_t y = 36;
  for (uint8_t i = 0; i < consoleCount; i++) {
    tft.setCursor(8, y);
    tft.print(consoleBuf[i]);
    y += 15;
  }

  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(8, tft.height() - 14);
  tft.print(consolePaused ? "PAUSED - click: resume, hold: back"
                           : "Click: pause, hold: back");

  prevOccupiedH = tft.height() - 32; // same leftover-clear reasoning as the detail view
}

static void removeRowAt(uint8_t idx) {
  for (uint8_t i = idx; i < rowCount - 1; i++) rows[i] = rows[i + 1];
  rowCount--;
}

static int8_t hexDigit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// True only for a well-formed "$...,...*hh" / "!...,...*hh" line whose
// checksum actually matches. At the wrong baud rate, bytes are garbled
// enough that this essentially never happens by chance — which is what
// makes it a reliable signal during baud auto-detection.
static bool validChecksum(const char *line, uint8_t len) {
  if (len < 4) return false;
  if (line[0] != '$' && line[0] != '!') return false;
  if (line[len - 3] != '*') return false;
  int8_t hi = hexDigit(line[len - 2]);
  int8_t lo = hexDigit(line[len - 1]);
  if (hi < 0 || lo < 0) return false;
  uint8_t expected = (hi << 4) | lo;
  uint8_t sum = 0;
  for (uint8_t i = 1; i < len - 3; i++) sum ^= line[i];
  return sum == expected;
}

static void drawBaudStatus(uint32_t candidate) {
  clearContentArea();
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(8, ROW_Y0);
  tft.print("Detecting baud rate...");
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(8, ROW_Y0 + 24);
  tft.print("trying ");
  tft.print(candidate);
  tft.print(" bps");
}

static void startBaudCandidate(uint8_t idx) {
  baudCandidateIdx = idx;
  gpsSerial.end();
  gpsSerial.begin(BAUD_CANDIDATES[idx], SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  while (gpsSerial.available()) gpsSerial.read(); // discard bytes from the old rate
  lineLen = 0;
  baudValidCount = 0;
  baudWindowStartMs = millis();
  if (idx == 0) clearBaudBadge(); // starting a fresh scan: the old rate no longer applies
  Serial.printf("Trying %lu baud...\n", (unsigned long)BAUD_CANDIDATES[idx]);
  drawBaudStatus(BAUD_CANDIDATES[idx]);
}

static void lockBaud() {
  baudLocked = true;
  lastValidLineMs = millis();
  uint32_t baud = BAUD_CANDIDATES[baudCandidateIdx];
  Serial.printf("Locked at %lu baud\n", (unsigned long)baud);
  rowCount = 0;
  currentView = VIEW_LIST;
  selectedRow = 0;
  consoleCount = 0; // drop any lines buffered under the previous (wrong) rate
  consolePaused = false;
  redrawAllRows();
  drawBaudBadge(baud);
}

// Feeds one candidate-phase line through the checksum check; locks the baud
// once enough valid lines are seen, or moves to the next candidate once the
// listening window for this one runs out.
static void tickBaudDetection() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (c == '\n') {
      if (validChecksum(lineBuf, lineLen)) baudValidCount++;
      lineLen = 0;
      if (baudValidCount >= BAUD_LOCK_THRESHOLD) {
        lockBaud();
        return;
      }
    } else if (c != '\r') {
      if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = c;
    }
  }
  if (millis() - baudWindowStartMs >= BAUD_CANDIDATE_WINDOW_MS) {
    startBaudCandidate((baudCandidateIdx + 1) % NUM_BAUD_CANDIDATES);
  }
}

// Handles one full NMEA line: "$GPRMC,...*hh" or "!AIVDM,...*hh"
static void processLine(const char *line, uint8_t len) {
  if (len < 6) return;
  if (line[0] != '$' && line[0] != '!') return;
  lastValidLineMs = millis(); // structurally plausible: counts as "still receiving real NMEA"

  char id[3] = { line[1], line[2], '\0' };
  char type[4] = { line[3], line[4], line[5], '\0' };
  const char *category = categoryFor(id, type);

  int8_t idx = findRow(category);
  if (idx < 0) {
    if (rowCount >= MAX_SENSORS) return; // list full, ignore new categories for now
    idx = rowCount++;
    rows[idx].category = category;
    rows[idx].typeCount = 0;
    // Don't resize/redraw right now: several sensors often show up within
    // the same second, and resizing after each one would visibly grow the
    // rows and shrink them back down repeatedly. Wait for a quiet moment.
    layoutPending = true;
    lastRowSetChangeMs = millis();
  }

  rows[idx].lastSeen = millis();

  bool grew = addTypeSilently(idx, type);
  if (grew && currentView == VIEW_DETAIL && detailCategory == category) {
    drawDetailView(idx);
  }
}

// Drops any category that's gone quiet, so only currently-detected sensors stay listed.
// Also prunes any individual sentence type that's gone quiet on a category
// that's otherwise still active, and refreshes the detail screen if that's
// the one currently open.
static void refreshTimeouts() {
  uint32_t now = millis();
  bool removedAny = false;
  bool detailClosed = false;
  for (uint8_t i = 0; i < rowCount; ) {
    if (now - rows[i].lastSeen >= SENSOR_TIMEOUT_MS) {
      if (currentView == VIEW_DETAIL && detailCategory == rows[i].category) {
        currentView = VIEW_LIST; // the sensor being viewed just disappeared
        detailClosed = true;
      }
      removeRowAt(i);
      removedAny = true;
    } else {
      if (pruneStaleTypes(i) && currentView == VIEW_DETAIL && detailCategory == rows[i].category) {
        drawDetailView(i);
      }
      i++;
    }
  }
  if (removedAny) {
    layoutPending = true;
    lastRowSetChangeMs = now;
    if (detailClosed) {
      // The screen is currently showing the now-stale detail view we just
      // backed out of — replace it right away instead of waiting out the
      // usual settle delay, which would otherwise leave it frozen on screen.
      layoutPending = false;
      redrawAllRows();
    }
  }
}

// Performs the deferred resize once the sensor set has been stable for a
// moment (see LAYOUT_SETTLE_MS) — the one point where rows actually change
// height/position, instead of doing it on every single add/remove.
static void checkLayoutSettle() {
  if (layoutPending && millis() - lastRowSetChangeMs >= LAYOUT_SETTLE_MS) {
    layoutPending = false;
    redrawAllRows();
  }
}

// Short click: from the list, opens the detail screen for the highlighted
// row; from the console, toggles pause (frozen for reading, but still
// buffering underneath) instead of leaving; from the detail screen, goes
// back to the list.
static void onEncoderClick() {
  if (currentView == VIEW_LIST) {
    if (rowCount == 0) return;
    if (selectedRow >= rowCount) selectedRow = rowCount - 1;
    detailCategory = rows[selectedRow].category;
    currentView = VIEW_DETAIL;
    drawDetailView(selectedRow);
  } else if (currentView == VIEW_CONSOLE) {
    consolePaused = !consolePaused;
    drawConsoleView();
  } else {
    currentView = VIEW_LIST;
    redrawAllRows();
  }
}

// Long press toggles the console from the list (open) or from the console
// itself (close, back to the list). Does nothing from the detail screen.
static void onEncoderLongPress() {
  if (currentView == VIEW_LIST) {
    currentView = VIEW_CONSOLE;
    consolePaused = false;
    drawConsoleView();
  } else if (currentView == VIEW_CONSOLE) {
    currentView = VIEW_LIST;
    redrawAllRows();
  }
}

// Applies pending encoder rotation to the list selection, and tracks the
// click button's hold time to tell a short click from a long press.
static void handleEncoder() {
  noInterrupts();
  int8_t delta = encDelta;
  encDelta = 0;
  interrupts();

  // Typical KY-040 modules emit 4 quadrature edges per detent; accumulate
  // and only move the selection once a full detent's worth has come in, so
  // one physical click always means exactly one row of movement.
  static int16_t encAccum = 0;
  encAccum += delta;
  const int16_t STEPS_PER_DETENT = 4;
  int8_t steps = 0;
  while (encAccum >= STEPS_PER_DETENT) { encAccum -= STEPS_PER_DETENT; steps++; }
  while (encAccum <= -STEPS_PER_DETENT) { encAccum += STEPS_PER_DETENT; steps--; }

  if (steps != 0 && currentView == VIEW_LIST && rowCount > 0) {
    int16_t newSel = (int16_t)selectedRow + steps;
    if (newSel < 0) newSel = 0;
    if (newSel >= rowCount) newSel = rowCount - 1;
    if (newSel != selectedRow) {
      uint8_t old = selectedRow;
      selectedRow = (uint8_t)newSel;
      drawRow(old);
      drawRow(selectedRow);
    }
  }

  static bool swPressed = false;
  static bool longPressFired = false;
  static uint32_t swPressStartMs = 0;
  bool pressed = (digitalRead(ENC_SW_PIN) == LOW);

  if (pressed && !swPressed) {
    swPressed = true;
    longPressFired = false;
    swPressStartMs = millis();
  } else if (pressed && swPressed && !longPressFired && millis() - swPressStartMs >= LONG_PRESS_MS) {
    longPressFired = true;
    onEncoderLongPress();
  } else if (!pressed && swPressed) {
    swPressed = false;
    if (!longPressFired && millis() - swPressStartMs > SW_DEBOUNCE_MS) {
      onEncoderClick();
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println(F("=== NMEA Reader ==="));
  Serial.printf("SCK=%d MOSI=%d CS=%d DC=%d RST=%d @ %lu Hz\n",
                PIN_SCL, PIN_SDA, PIN_CS, PIN_DC, PIN_RST, (unsigned long)SPI_HZ);
  Serial.printf("GPS RX=%d TX=%d\n", GPS_RX_PIN, GPS_TX_PIN);
  Serial.printf("Encoder CLK=%d DT=%d SW=%d\n", ENC_CLK_PIN, ENC_DT_PIN, ENC_SW_PIN);

  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_DT_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), onEncoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT_PIN), onEncoderChange, CHANGE);

  // Remaps SPI through the GPIO matrix: works even if you're not on 18/23
  SPI.begin(PIN_SCL, PIN_MISO, PIN_SDA, PIN_CS);

  tft.init(TFT_W, TFT_H);
  tft.setSPISpeed(SPI_HZ);
  tft.setRotation(1);          // landscape — try 3 if the image is upside down

  showSplashScreen();

  drawHeader();
  startBaudCandidate(0);
}

void loop() {
  if (!baudLocked) {
    tickBaudDetection();
    return;
  }

  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (c == '\n') {
      Serial.write((const uint8_t *)lineBuf, lineLen); // raw passthrough, for viewing in a serial monitor
      Serial.println();
      pushConsoleLine(lineBuf, lineLen);
      processLine(lineBuf, lineLen);
      lineLen = 0;
    } else if (c != '\r') {
      if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = c;
    }
  }

  // Only redraw the console once per batch of lines, not once per line —
  // several can arrive in the same pass through this loop. While paused,
  // leave consoleDirty set so the view catches up in one redraw on resume.
  if (consoleDirty && currentView == VIEW_CONSOLE && !consolePaused) {
    consoleDirty = false;
    drawConsoleView();
  }

  static uint32_t lastRefresh = 0;
  if (millis() - lastRefresh >= 300) {
    lastRefresh = millis();
    refreshTimeouts();
  }

  checkLayoutSettle();
  handleEncoder();

  // No valid-looking line in a while — either the bus went quiet or the
  // source's baud rate changed and it's all garbled now. Either way, re-scan.
  if (millis() - lastValidLineMs >= BAUD_SILENCE_RESCAN_MS) {
    baudLocked = false;
    startBaudCandidate(0);
  }
}
