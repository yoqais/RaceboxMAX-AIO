#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_MAX1704X.h>
#include <NimBLEDevice.h>
#include <string.h>
#include <math.h>

#define PIN_SDA        D4
#define PIN_SCL        D5
#define PIN_OLED_RES   D3
#define PIN_BTN_UP     D0
#define PIN_BTN_DOWN   D1
#define PIN_BTN_SELECT D2

// -------------------------------------------------------------- settings --
// Paste your RaceBox address here once you have it, e.g. "d1:9a:44:0f:22:31"
static const char* TARGET_ADDRESS = "";

static const uint32_t ADDRESS_HOLD_MS  = 5000;
static const uint32_t DRAW_INTERVAL_MS = 100;
static const uint32_t GAUGE_INTERVAL_MS = 1000;
static const uint32_t GAUGE_SETTLE_MS  = 2500;
static const uint32_t DEBOUNCE_MS      = 25;
static const uint32_t LONGPRESS_MS     = 600;
static const uint32_t MIN_LAP_MS       = 10000;   // ignore re-triggers
static const float    GATE_HALF_WIDTH_M = 15.0f;  // captured gates: 30 m wide

// ----------------------------------------------------------- BLE objects --
static NimBLEUUID UART_SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static NimBLEUUID TX_CHAR_UUID     ("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

U8G2_SSD1309_128X64_NONAME0_F_HW_I2C u8g2(U8G2_R0, PIN_OLED_RES, PIN_SCL, PIN_SDA);
Adafruit_MAX17048 gauge;

// ================================================================ screens ==
enum Screen {
  SC_SCAN, SC_ADDR, SC_LOST,
  SC_MAIN, SC_LIVE,
  SC_TRACKLIST, SC_TRACKRUN,
  SC_DRAGLIST, SC_DRAGRUN,
  SC_OPTIONS, SC_BATTERY
};
volatile Screen screen = SC_SCAN;

static const char* MAIN_ITEMS[]  = {"Live", "Track", "Drag", "Options"};
static const char* TRACK_ITEMS[] = {"< Back", "T-Hill East 3mi",
                                    "T-Hill West 2mi", "T-Hill 5 Mile"};
static const char* DRAG_ITEMS[]  = {"< Back", "Standing start",
                                    "60-0 braking", "Clear bests"};
static const char* OPT_ITEMS[]   = {"< Back", "Battery"};

uint8_t selMain = 0,  topMain = 0;
uint8_t selTrack = 0, topTrack = 0;
uint8_t selDrag = 0,  topDrag = 0;
uint8_t selOpt = 0,   topOpt = 0;

// ================================================================= tracks ==
struct Gate { double lat1, lon1, lat2, lon2; bool valid; };
struct TrackDef {
  const char* name;
  double cLat, cLon;      // approximate facility centre, proximity check only
  float  radiusKm;
  Gate   gate;
};

// Thunderhill Raceway Park, Willows CA. Centre is approximate and used ONLY
// to decide "are we at the track". Gates must be captured or hard-coded.
TrackDef tracks[] = {
  { "T-Hill East 3mi", 39.5372, -122.3310, 3.0f, {0,0,0,0,false} },
  { "T-Hill West 2mi", 39.5372, -122.3310, 3.0f, {0,0,0,0,false} },
  { "T-Hill 5 Mile",   39.5372, -122.3310, 3.0f, {0,0,0,0,false} },
};
const uint8_t TRACK_COUNT = sizeof(tracks) / sizeof(tracks[0]);
int8_t activeTrack = -1;

// ============================================================ live values ==
bool gaugeOK = false, displayOK = false;
float batPct = 0, batVolt = 0, batRate = 0;
uint16_t batIC = 0; uint8_t batChip = 0, batAlert = 0;

volatile bool connected = false, doConnect = false;
static NimBLEAddress targetAddr;
static bool haveTarget = false;
static NimBLERemoteCharacteristic* pTxChar = nullptr;
static NimBLEClient* pClient = nullptr;
char rbAddress[24] = "--:--:--:--:--:--";
char rbName[28]    = "unknown";
uint32_t addressShownAt = 0;

uint32_t rb_iTOW;
uint16_t rb_year; uint8_t rb_month, rb_day, rb_hour, rb_min, rb_sec;
uint8_t  rb_fixStatus, rb_numSVs, rb_battRaw;
int32_t  rb_lon, rb_lat, rb_mslAlt;
uint32_t rb_hAcc, rb_speed, rb_heading;
uint16_t rb_pdop;
int16_t  rb_gx, rb_gy, rb_gz, rb_rx, rb_ry, rb_rz;
uint32_t rbPackets = 0;

// ============================================================ lap timing ===
struct LapState {
  bool     havePrev;
  double   prevX, prevY;
  uint32_t prevITOW;
  bool     running;
  uint32_t startITOW;
  uint32_t lastLapMs, bestLapMs;
  uint16_t lapCount;
  bool     newBest;
  uint32_t newBestAt;
} lap;

// ============================================================ drag timing ==
enum DragMode  { DM_STANDING, DM_BRAKE };
enum DragState { DS_WAIT, DS_ARMED, DS_RUN, DS_DONE };
DragMode  dragMode  = DM_STANDING;
volatile DragState dragState = DS_WAIT;

struct DragRun {
  uint32_t t0;            // iTOW at launch / brake start
  double   dist;          // metres travelled
  uint32_t prevITOW;
  float    t60ft, t0_60, t1_8, t1_4;
  float    v1_8, v1_4;
  float    brakeDistFt, brakeFromMph;
  uint32_t stillSince;
} run;

float bestT0_60 = 0, bestT1_4 = 0, bestBrakeFt = 0;

// =========================================================== button types ==
// These MUST stay above the first function definition in the file. The
// Arduino IDE injects auto-generated prototypes just before the first
// function, so any type used in a signature has to already exist by then.
enum BtnEvent { EV_NONE, EV_UP, EV_DOWN, EV_SELECT, EV_BACK };

struct Btn {
  uint8_t pin; bool stable; bool lastRaw;
  uint32_t changedAt, pressedAt; bool longSent;
};
Btn bUp {PIN_BTN_UP,     true, true, 0, 0, false};
Btn bDn {PIN_BTN_DOWN,   true, true, 0, 0, false};
Btn bSel{PIN_BTN_SELECT, true, true, 0, 0, false};

// ================================================== safe field extraction ==
static inline uint16_t rdU16(const uint8_t* p){ uint16_t v; memcpy(&v,p,2); return v; }
static inline int16_t  rdI16(const uint8_t* p){ int16_t  v; memcpy(&v,p,2); return v; }
static inline uint32_t rdU32(const uint8_t* p){ uint32_t v; memcpy(&v,p,4); return v; }
static inline int32_t  rdI32(const uint8_t* p){ int32_t  v; memcpy(&v,p,4); return v; }

// ==================================================== unit helpers ========
float speedMps()  { return rb_speed / 1000.0f; }
float speedMph()  { return rb_speed * 0.00223694f; }
float headingDeg(){ return rb_heading / 100000.0f; }
float latG()      { return rb_gy / 1000.0f; }
float lonG()      { return rb_gx / 1000.0f; }
float rbInputV()  { return rb_battRaw / 10.0f; }
bool  haveFix()   { return rb_fixStatus == 3 || rb_fixStatus == 2; }

const char* compass(float d) {
  static const char* p[] = {"N","NE","E","SE","S","SW","W","NW"};
  return p[((int)((d + 22.5f) / 45.0f)) & 7];
}
const char* fixText(uint8_t f) {
  switch (f) { case 0: return "nofix"; case 2: return "2D"; case 3: return "3D"; }
  return "?";
}

// local flat projection, good to millimetres over a few km
static void projectTo(double lat, double lon, double lat0, double lon0,
                      double &x, double &y) {
  const double M_PER_DEG = 111320.0;
  x = (lon - lon0) * M_PER_DEG * cos(lat0 * M_PI / 180.0);
  y = (lat - lat0) * M_PER_DEG;
}

static double distanceKm(double la1, double lo1, double la2, double lo2) {
  double x, y;
  projectTo(la1, lo1, la2, lo2, x, y);
  return sqrt(x * x + y * y) / 1000.0;
}

// ==================================================== packet reassembly ====
#define PKT_MAX 160
static uint8_t pkt[PKT_MAX];
static size_t  pktLen = 0;

static bool checksumOK(const uint8_t* d, size_t len) {
  uint8_t a = 0, b = 0;
  for (size_t i = 2; i < len - 2; i++) { a += d[i]; b += a; }
  return (a == d[len - 2] && b == d[len - 1]);
}

void updateLapTiming();
void updateDrag();

static void parseDataMessage(const uint8_t* d) {
  rb_iTOW   = rdU32(d + 6);
  rb_year   = rdU16(d + 10);
  rb_month  = d[12]; rb_day = d[13];
  rb_hour   = d[14]; rb_min = d[15]; rb_sec = d[16];
  rb_fixStatus = d[26];
  rb_numSVs    = d[29];
  rb_lon    = rdI32(d + 30);
  rb_lat    = rdI32(d + 34);
  rb_mslAlt = rdI32(d + 42);
  rb_hAcc   = rdU32(d + 46);
  rb_speed  = rdU32(d + 54);
  rb_heading= rdU32(d + 58);
  rb_pdop   = rdU16(d + 70);
  rb_battRaw= d[73];
  rb_gx = rdI16(d + 74); rb_gy = rdI16(d + 76); rb_gz = rdI16(d + 78);
  rb_rx = rdI16(d + 80); rb_ry = rdI16(d + 82); rb_rz = rdI16(d + 84);
  rbPackets++;

  // run the timers here so no 25 Hz sample is ever missed
  if (screen == SC_TRACKRUN) updateLapTiming();
  if (screen == SC_DRAGRUN)  updateDrag();
}

static void handlePacket(const uint8_t* d, size_t len) {
  if (d[2] == 0xFF && d[3] == 0x01 && rdU16(d + 4) == 80) parseDataMessage(d);
}

static void feedByte(uint8_t b) {
  if (pktLen >= PKT_MAX) pktLen = 0;
  if (pktLen == 0) { if (b == 0xB5) pkt[pktLen++] = b; return; }
  if (pktLen == 1) {
    if (b == 0x62) pkt[pktLen++] = b;
    else if (b != 0xB5) pktLen = 0;
    return;
  }
  pkt[pktLen++] = b;
  if (pktLen >= 6) {
    size_t total = 6 + rdU16(pkt + 4) + 2;
    if (total > PKT_MAX) { pktLen = 0; return; }
    if (pktLen == total) {
      if (checksumOK(pkt, total)) handlePacket(pkt, total);
      pktLen = 0;
    }
  }
}

// ============================================== gate crossing detection ====
// Segment/segment intersection. t = fraction along the path segment.
static bool segIntersect(double ax,double ay,double bx,double by,
                         double cx,double cy,double dx,double dy, double &t) {
  double rx = bx-ax, ry = by-ay, sx = dx-cx, sy = dy-cy;
  double den = rx*sy - ry*sx;
  if (fabs(den) < 1e-12) return false;
  double tt = ((cx-ax)*sy - (cy-ay)*sx) / den;
  double uu = ((cx-ax)*ry - (cy-ay)*rx) / den;
  if (tt < 0 || tt > 1 || uu < 0 || uu > 1) return false;
  t = tt;
  return true;
}

void resetLap() {
  memset(&lap, 0, sizeof(lap));
}

void updateLapTiming() {
  if (activeTrack < 0) return;
  Gate &g = tracks[activeTrack].gate;
  if (!g.valid || !haveFix()) { lap.havePrev = false; return; }

  double lat = rb_lat / 1e7, lon = rb_lon / 1e7;
  double lat0 = (g.lat1 + g.lat2) * 0.5, lon0 = (g.lon1 + g.lon2) * 0.5;

  double px, py;  projectTo(lat, lon, lat0, lon0, px, py);
  double g1x, g1y, g2x, g2y;
  projectTo(g.lat1, g.lon1, lat0, lon0, g1x, g1y);
  projectTo(g.lat2, g.lon2, lat0, lon0, g2x, g2y);

  if (!lap.havePrev) {
    lap.havePrev = true;
    lap.prevX = px; lap.prevY = py; lap.prevITOW = rb_iTOW;
    return;
  }

  double t;
  if (segIntersect(lap.prevX, lap.prevY, px, py, g1x, g1y, g2x, g2y, t)) {
    // interpolate the exact crossing instant between the two 25 Hz samples
    int32_t dt = (int32_t)(rb_iTOW - lap.prevITOW);
    uint32_t cross = lap.prevITOW + (uint32_t)(t * dt);

    if (lap.running) {
      uint32_t lapMs = cross - lap.startITOW;
      if (lapMs >= MIN_LAP_MS) {
        lap.lastLapMs = lapMs;
        lap.lapCount++;
        if (lap.bestLapMs == 0 || lapMs < lap.bestLapMs) {
          lap.bestLapMs = lapMs;
          lap.newBest = true;
          lap.newBestAt = millis();
        }
        lap.startITOW = cross;
      }
    } else {
      lap.running = true;
      lap.startITOW = cross;
      lap.lapCount = 0;
    }
  }

  lap.prevX = px; lap.prevY = py; lap.prevITOW = rb_iTOW;
}

// build a gate perpendicular to the current heading, centred here
void captureGate() {
  if (activeTrack < 0 || !haveFix()) return;
  double lat = rb_lat / 1e7, lon = rb_lon / 1e7;
  double hdg = headingDeg() * M_PI / 180.0;

  // perpendicular unit vector in metres
  double px = cos(hdg), py = -sin(hdg);          // right-hand normal
  double mLat = GATE_HALF_WIDTH_M / 111320.0;
  double mLon = GATE_HALF_WIDTH_M / (111320.0 * cos(lat * M_PI / 180.0));

  Gate &g = tracks[activeTrack].gate;
  g.lat1 = lat + py * mLat;  g.lon1 = lon + px * mLon;
  g.lat2 = lat - py * mLat;  g.lon2 = lon - px * mLon;
  g.valid = true;

  resetLap();
  Serial.printf("GATE %s\n  {%.7f, %.7f, %.7f, %.7f, true}\n",
                tracks[activeTrack].name, g.lat1, g.lon1, g.lat2, g.lon2);
}

// ===================================================== drag mode timing ====
void resetDrag() {
  memset(&run, 0, sizeof(run));
  dragState = DS_WAIT;
}

void updateDrag() {
  float mph = speedMph();
  uint32_t now = rb_iTOW;

  if (dragMode == DM_STANDING) {
    switch (dragState) {
      case DS_WAIT:
        if (mph < 1.0f) {
          if (run.stillSince == 0) run.stillSince = now;
          if (now - run.stillSince > 1000) { dragState = DS_ARMED; }
        } else run.stillSince = 0;
        break;

      case DS_ARMED:
        if (mph > 1.0f) {
          memset(&run, 0, sizeof(run));
          run.t0 = now; run.prevITOW = now;
          dragState = DS_RUN;
        }
        break;

      case DS_RUN: {
        int32_t dt = (int32_t)(now - run.prevITOW);
        if (dt > 0 && dt < 500) run.dist += speedMps() * (dt / 1000.0);
        run.prevITOW = now;
        float el = (now - run.t0) / 1000.0f;

        if (run.t60ft == 0 && run.dist >= 18.288)  run.t60ft = el;
        if (run.t0_60 == 0 && mph >= 60.0f)        run.t0_60 = el;
        if (run.t1_8  == 0 && run.dist >= 201.168) { run.t1_8 = el; run.v1_8 = mph; }
        if (run.t1_4  == 0 && run.dist >= 402.336) {
          run.t1_4 = el; run.v1_4 = mph;
          dragState = DS_DONE;
          if (bestT1_4 == 0 || run.t1_4 < bestT1_4) bestT1_4 = run.t1_4;
          if (run.t0_60 > 0 && (bestT0_60 == 0 || run.t0_60 < bestT0_60))
            bestT0_60 = run.t0_60;
        }
        if (mph < 1.0f && el > 3.0f) dragState = DS_DONE;   // aborted
        break;
      }
      case DS_DONE: break;
    }
  } else {  // DM_BRAKE
    switch (dragState) {
      case DS_WAIT:
        if (mph >= 62.0f) dragState = DS_ARMED;
        break;
      case DS_ARMED:
        if (mph <= 60.0f) {
          memset(&run, 0, sizeof(run));
          run.t0 = now; run.prevITOW = now; run.brakeFromMph = mph;
          dragState = DS_RUN;
        }
        break;
      case DS_RUN: {
        int32_t dt = (int32_t)(now - run.prevITOW);
        if (dt > 0 && dt < 500) run.dist += speedMps() * (dt / 1000.0);
        run.prevITOW = now;
        if (mph < 1.0f) {
          run.brakeDistFt = run.dist * 3.28084;
          dragState = DS_DONE;
          if (bestBrakeFt == 0 || run.brakeDistFt < bestBrakeFt)
            bestBrakeFt = run.brakeDistFt;
        }
        if (mph > 70.0f) dragState = DS_WAIT;   // gave up braking
        break;
      }
      case DS_DONE: break;
    }
  }
}

// ================================================================ buttons ==
// returns 1 on release-after-short, 2 on long-press reached
uint8_t pollBtn(Btn &b) {
  uint8_t out = 0;
  bool raw = digitalRead(b.pin);          // LOW = pressed (INPUT_PULLUP)
  uint32_t now = millis();

  if (raw != b.lastRaw) { b.lastRaw = raw; b.changedAt = now; }
  if ((now - b.changedAt) > DEBOUNCE_MS && raw != b.stable) {
    b.stable = raw;
    if (!b.stable) { b.pressedAt = now; b.longSent = false; }   // pressed
    else if (!b.longSent) out = 1;                              // short release
  }
  if (!b.stable && !b.longSent && (now - b.pressedAt) > LONGPRESS_MS) {
    b.longSent = true;
    out = 2;
  }
  return out;
}

BtnEvent readButtons() {
  uint8_t u = pollBtn(bUp), d = pollBtn(bDn), s = pollBtn(bSel);
  if (s == 2) return EV_BACK;
  if (s == 1) return EV_SELECT;
  if (u)      return EV_UP;
  if (d)      return EV_DOWN;
  return EV_NONE;
}

// ================================================================ BLE glue ==
static void notifyCallback(NimBLERemoteCharacteristic*, uint8_t* data,
                           size_t len, bool) {
  for (size_t i = 0; i < len; i++) feedByte(data[i]);
}

class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient*) override { connected = true; }
  void onDisconnect(NimBLEClient*, int reason) override {
    connected = false; pTxChar = nullptr; screen = SC_LOST;
    Serial.printf("disconnected (%d)\n", reason);
  }
};

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    std::string name = dev->getName();
    if (name.rfind("RaceBox", 0) != 0) return;
    std::string addr = dev->getAddress().toString();
    if (strlen(TARGET_ADDRESS) > 0 && addr != std::string(TARGET_ADDRESS)) return;

    snprintf(rbName, sizeof(rbName), "%s", name.c_str());
    snprintf(rbAddress, sizeof(rbAddress), "%s", addr.c_str());
    Serial.printf("Found %s @ %s\n", rbName, rbAddress);

    targetAddr = dev->getAddress();
    haveTarget = true;
    NimBLEDevice::getScan()->stop();
    doConnect = true;
  }
};
static ScanCallbacks   scanCbs;
static ClientCallbacks clientCbs;

void startScan() {
  NimBLEScan* s = NimBLEDevice::getScan();
  s->setScanCallbacks(&scanCbs, false);
  s->setInterval(45); s->setWindow(15); s->setActiveScan(true);
  s->start(0, false, false);
  Serial.println("Scanning...");
}

bool connectToRaceBox() {
  if (!haveTarget) return false;
  pClient = NimBLEDevice::createClient();
  pClient->setClientCallbacks(&clientCbs, false);
  if (!pClient->connect(targetAddr)) {
    NimBLEDevice::deleteClient(pClient); pClient = nullptr; return false;
  }
  NimBLERemoteService* svc = pClient->getService(UART_SERVICE_UUID);
  if (!svc) return false;
  pTxChar = svc->getCharacteristic(TX_CHAR_UUID);
  if (!pTxChar) return false;
  return pTxChar->subscribe(true, notifyCallback);
}

// ================================================================ display ==
void drawStatusBar() {
  char right[24], left[20];
  u8g2.setFont(u8g2_font_5x8_tf);

  if (gaugeOK && millis() > GAUGE_SETTLE_MS)
    snprintf(right, sizeof(right), "%d%% %+.1f/h", (int)(batPct + 0.5f), batRate);
  else
    snprintf(right, sizeof(right), "--%% --/h");
  int rw = u8g2.getStrWidth(right);
  u8g2.drawStr(128 - rw, 7, right);

  if (connected) snprintf(left, sizeof(left), "%s %dsv", fixText(rb_fixStatus), rb_numSVs);
  else           snprintf(left, sizeof(left), "no link");

  if (u8g2.getStrWidth(left) <= 128 - rw - 4) u8g2.drawStr(0, 7, left);
  u8g2.drawHLine(0, 10, 128);
}

void drawList(const char* const* items, uint8_t count, uint8_t sel, uint8_t top) {
  u8g2.setFont(u8g2_font_7x14_tf);
  for (uint8_t i = 0; i < 3 && (top + i) < count; i++) {
    uint8_t idx = top + i;
    int y0 = 12 + i * 17;
    if (idx == sel) { u8g2.drawBox(0, y0, 122, 17); u8g2.setDrawColor(0); }
    u8g2.drawStr(3, y0 + 13, items[idx]);
    u8g2.setDrawColor(1);
  }
  if (count > 3) {
    int barH = 51 * 3 / count; if (barH < 8) barH = 8;
    int barY = 12 + (int)((51 - barH) * (float)top / (float)(count - 3));
    u8g2.drawFrame(124, 12, 4, 51);
    u8g2.drawBox(125, barY + 1, 2, barH - 2);
  }
}

void ensureVisible(uint8_t sel, uint8_t &top, uint8_t count) {
  if (count <= 3) { top = 0; return; }
  if (sel < top) top = sel;
  if (sel >= top + 3) top = sel - 2;
}

void fmtLap(char* out, size_t n, uint32_t ms) {
  if (ms == 0) { snprintf(out, n, "--:--.--"); return; }
  uint32_t m = ms / 60000, s = (ms % 60000) / 1000, c = (ms % 1000) / 10;
  snprintf(out, n, "%lu:%02lu.%02lu",
           (unsigned long)m, (unsigned long)s, (unsigned long)c);
}

void drawBigLeft(const char* s, int baseline) {
  u8g2.setFont(u8g2_font_logisoso24_tr);
  u8g2.drawStr(2, baseline, s);
}

// ---- screens ----
void scrScan() {
  u8g2.setFont(u8g2_font_7x14_tf);
  u8g2.drawStr(0, 32, "Searching for");
  u8g2.drawStr(0, 50, "RaceBox...");
}

void scrAddr() {
  char b[32];
  uint32_t left = (ADDRESS_HOLD_MS - (millis() - addressShownAt)) / 1000 + 1;
  u8g2.setFont(u8g2_font_7x14_tf);
  u8g2.drawStr(0, 26, "CONNECTED");
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 42, rbAddress);
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 54, rbName);
  snprintf(b, sizeof(b), "menu in %lus", (unsigned long)left);
  u8g2.drawStr(0, 63, b);
}

void scrLost() {
  u8g2.setFont(u8g2_font_7x14_tf);
  u8g2.drawStr(0, 32, "Link lost");
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 50, "rescanning...");
}

void scrLive() {
  char b[32];
  snprintf(b, sizeof(b), "%d", (int)(speedMph() + 0.5f));
  drawBigLeft(b, 44);
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(2 + u8g2.getStrWidth(b) + 34, 44, "MPH");

  u8g2.setFont(u8g2_font_5x8_tf);
  snprintf(b, sizeof(b), "lat%+.2fg  lon%+.2fg", latG(), lonG());
  u8g2.drawStr(0, 54, b);
  snprintf(b, sizeof(b), "%s %03d  alt %dm  %.1fm",
           compass(headingDeg()), (int)headingDeg(),
           (int)(rb_mslAlt / 1000), rb_hAcc / 1000.0);
  u8g2.drawStr(0, 63, b);
}

void drawCheckerFlag(int x, int y) {
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 4; c++)
      if ((r + c) & 1) u8g2.drawBox(x + c * 5, y + r * 5, 5, 5);
  u8g2.drawFrame(x, y, 20, 15);
}

void scrTrackRun() {
  char b[32];
  u8g2.setFont(u8g2_font_5x8_tf);

  if (activeTrack < 0) return;
  TrackDef &T = tracks[activeTrack];

  if (!connected || !haveFix()) {
    u8g2.drawStr(0, 20, T.name);
    u8g2.setFont(u8g2_font_7x14_tf);
    u8g2.drawStr(0, 42, "NO FIX");
    return;
  }

  double d = distanceKm(rb_lat / 1e7, rb_lon / 1e7, T.cLat, T.cLon);
  if (d > T.radiusKm) {
    u8g2.drawStr(0, 20, T.name);
    u8g2.setFont(u8g2_font_7x14_tf);
    u8g2.drawStr(0, 40, "NOT AT TRACK");
    u8g2.setFont(u8g2_font_5x8_tf);
    snprintf(b, sizeof(b), "%.1f km away", d);
    u8g2.drawStr(0, 55, b);
    return;
  }

  if (!T.gate.valid) {
    u8g2.drawStr(0, 20, T.name);
    u8g2.setFont(u8g2_font_7x14_tf);
    u8g2.drawStr(0, 40, "NO GATE SET");
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(0, 55, "press UP to capture here");
    return;
  }

  snprintf(b, sizeof(b), "%s  lap %d", T.name, lap.lapCount);
  u8g2.drawStr(0, 20, b);

  if (lap.running) {
    uint32_t el = rb_iTOW - lap.startITOW;
    fmtLap(b, sizeof(b), el);
  } else {
    snprintf(b, sizeof(b), "ARMED");
  }
  if (lap.running) drawBigLeft(b, 48);
  else { u8g2.setFont(u8g2_font_7x14_tf); u8g2.drawStr(2, 44, b); }

  u8g2.setFont(u8g2_font_5x8_tf);
  char lb[16], bb[16];
  fmtLap(lb, sizeof(lb), lap.lastLapMs);
  fmtLap(bb, sizeof(bb), lap.bestLapMs);
  snprintf(b, sizeof(b), "L %s  B %s", lb, bb);
  u8g2.drawStr(0, 63, b);

  if (lap.newBest && millis() - lap.newBestAt < 4000) drawCheckerFlag(104, 14);
  else if (lap.newBest && millis() - lap.newBestAt >= 4000) lap.newBest = false;
}

void scrDragRun() {
  char b[36];
  u8g2.setFont(u8g2_font_5x8_tf);

  const char* st = dragState == DS_WAIT  ? "waiting"
                 : dragState == DS_ARMED ? "ARMED - go!"
                 : dragState == DS_RUN   ? "RUNNING"
                                         : "DONE";
  snprintf(b, sizeof(b), "%s  %s",
           dragMode == DM_STANDING ? "STANDING" : "60-0", st);
  u8g2.drawStr(0, 20, b);

  if (dragMode == DM_STANDING) {
    float el = (dragState == DS_RUN || dragState == DS_DONE)
             ? (run.prevITOW - run.t0) / 1000.0f : 0.0f;
    snprintf(b, sizeof(b), "%.2f", dragState == DS_DONE && run.t1_4 > 0
                                   ? run.t1_4 : el);
    drawBigLeft(b, 46);

    u8g2.setFont(u8g2_font_5x8_tf);
    snprintf(b, sizeof(b), "60ft %.2f  0-60 %.2f", run.t60ft, run.t0_60);
    u8g2.drawStr(0, 55, b);
    snprintf(b, sizeof(b), "1/8 %.2f@%.0f 1/4 %.2f@%.0f",
             run.t1_8, run.v1_8, run.t1_4, run.v1_4);
    u8g2.drawStr(0, 63, b);
  } else {
    snprintf(b, sizeof(b), "%.0f", dragState == DS_DONE ? run.brakeDistFt
                                                        : run.dist * 3.28084);
    drawBigLeft(b, 46);
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(2 + u8g2.getStrWidth(b) + 34, 46, "ft");
    u8g2.setFont(u8g2_font_5x8_tf);
    snprintf(b, sizeof(b), "best %.0f ft   now %.0f mph",
             bestBrakeFt, speedMph());
    u8g2.drawStr(0, 63, b);
  }
}

void scrBattery() {
  char b[28];
  u8g2.setFont(u8g2_font_5x8_tf);
  snprintf(b, sizeof(b), "SOC   %.1f %%", batPct);      u8g2.drawStr(0, 20, b);
  snprintf(b, sizeof(b), "Cell  %.3f V", batVolt);      u8g2.drawStr(0, 28, b);
  snprintf(b, sizeof(b), "Rate  %+.2f %%/hr", batRate); u8g2.drawStr(0, 36, b);
  if (batRate < -0.1f)      snprintf(b, sizeof(b), "Left  %.1f hr", batPct / -batRate);
  else if (batRate > 0.1f)  snprintf(b, sizeof(b), "Full  %.1f hr", (100 - batPct) / batRate);
  else                      snprintf(b, sizeof(b), "Left  --");
  u8g2.drawStr(0, 44, b);
  snprintf(b, sizeof(b), "IC    0x%04X", batIC);        u8g2.drawStr(0, 52, b);
  snprintf(b, sizeof(b), "Chip 0x%02X  Alrt 0x%02X", batChip, batAlert);
  u8g2.drawStr(0, 60, b);
}

void draw() {
  u8g2.clearBuffer();
  drawStatusBar();
  switch (screen) {
    case SC_SCAN:      scrScan(); break;
    case SC_ADDR:      scrAddr(); break;
    case SC_LOST:      scrLost(); break;
    case SC_MAIN:      drawList(MAIN_ITEMS, 4, selMain, topMain); break;
    case SC_LIVE:      scrLive(); break;
    case SC_TRACKLIST: drawList(TRACK_ITEMS, 4, selTrack, topTrack); break;
    case SC_TRACKRUN:  scrTrackRun(); break;
    case SC_DRAGLIST:  drawList(DRAG_ITEMS, 4, selDrag, topDrag); break;
    case SC_DRAGRUN:   scrDragRun(); break;
    case SC_OPTIONS:   drawList(OPT_ITEMS, 2, selOpt, topOpt); break;
    case SC_BATTERY:   scrBattery(); break;
  }
  u8g2.sendBuffer();
}

// ================================================================== input ==
void handleEvent(BtnEvent e) {
  if (e == EV_NONE) return;

  switch (screen) {
    case SC_MAIN:
      if (e == EV_UP   && selMain > 0) selMain--;
      if (e == EV_DOWN && selMain < 3) selMain++;
      ensureVisible(selMain, topMain, 4);
      if (e == EV_SELECT) {
        switch (selMain) {
          case 0: screen = SC_LIVE; break;
          case 1: screen = SC_TRACKLIST; selTrack = 0; topTrack = 0; break;
          case 2: screen = SC_DRAGLIST;  selDrag = 0;  topDrag = 0;  break;
          case 3: screen = SC_OPTIONS;   selOpt = 0;   topOpt = 0;   break;
        }
      }
      break;

    case SC_LIVE:
      if (e == EV_BACK || e == EV_SELECT) screen = SC_MAIN;
      break;

    case SC_TRACKLIST:
      if (e == EV_UP   && selTrack > 0) selTrack--;
      if (e == EV_DOWN && selTrack < 3) selTrack++;
      ensureVisible(selTrack, topTrack, 4);
      if (e == EV_SELECT) {
        if (selTrack == 0) screen = SC_MAIN;
        else { activeTrack = selTrack - 1; resetLap(); screen = SC_TRACKRUN; }
      }
      if (e == EV_BACK) screen = SC_MAIN;
      break;

    case SC_TRACKRUN:
      if (e == EV_UP && activeTrack >= 0 && !tracks[activeTrack].gate.valid)
        captureGate();
      if (e == EV_BACK) screen = SC_TRACKLIST;
      break;

    case SC_DRAGLIST:
      if (e == EV_UP   && selDrag > 0) selDrag--;
      if (e == EV_DOWN && selDrag < 3) selDrag++;
      ensureVisible(selDrag, topDrag, 4);
      if (e == EV_SELECT) {
        if (selDrag == 0) screen = SC_MAIN;
        else if (selDrag == 1) { dragMode = DM_STANDING; resetDrag(); screen = SC_DRAGRUN; }
        else if (selDrag == 2) { dragMode = DM_BRAKE;    resetDrag(); screen = SC_DRAGRUN; }
        else { bestT0_60 = bestT1_4 = bestBrakeFt = 0; }
      }
      if (e == EV_BACK) screen = SC_MAIN;
      break;

    case SC_DRAGRUN:
      if (e == EV_UP || e == EV_SELECT) resetDrag();   // re-arm
      if (e == EV_BACK) screen = SC_DRAGLIST;
      break;

    case SC_OPTIONS:
      if (e == EV_UP   && selOpt > 0) selOpt--;
      if (e == EV_DOWN && selOpt < 1) selOpt++;
      if (e == EV_SELECT) screen = (selOpt == 0) ? SC_MAIN : SC_BATTERY;
      if (e == EV_BACK) screen = SC_MAIN;
      break;

    case SC_BATTERY:
      if (e == EV_BACK || e == EV_SELECT) screen = SC_OPTIONS;
      break;

    default: break;
  }
}

// ================================================================== setup ==
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== RaceBox Dash ===");

  pinMode(PIN_BTN_UP, INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
  pinMode(PIN_BTN_SELECT, INPUT_PULLUP);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  displayOK = u8g2.begin();
  if (displayOK) u8g2.setBusClock(400000);

  for (uint8_t i = 0; i < 5 && !gaugeOK; i++) {
    gaugeOK = gauge.begin(&Wire);
    if (!gaugeOK) delay(400);
  }
  if (gaugeOK) { batIC = gauge.getICversion(); batChip = gauge.getChipID(); }
  Serial.println(gaugeOK ? "MAX17048 OK" : "MAX17048 missing");

  NimBLEDevice::init("");
  NimBLEDevice::setPower(9);
  NimBLEDevice::setMTU(517);
  startScan();

  resetLap();
  resetDrag();
}

// =================================================================== loop ==
void loop() {
  uint32_t now = millis();
  static uint32_t lastDraw = 0, lastGauge = 0;

  handleEvent(readButtons());

  if (doConnect) {
    doConnect = false;
    if (connectToRaceBox()) {
      addressShownAt = now;
      screen = SC_ADDR;
      Serial.printf("*** RaceBox address: %s\n", rbAddress);
    } else { screen = SC_SCAN; startScan(); }
  }

  if (screen == SC_ADDR && (now - addressShownAt) >= ADDRESS_HOLD_MS) {
    screen = SC_MAIN; selMain = 0; topMain = 0;
  }
  if (screen == SC_LOST && !connected) { screen = SC_SCAN; startScan(); }

  if (gaugeOK && now - lastGauge >= GAUGE_INTERVAL_MS) {
    lastGauge = now;
    batVolt  = gauge.cellVoltage();
    batPct   = gauge.cellPercent();
    batRate  = gauge.chargeRate();
    batAlert = gauge.getAlertStatus();
    if (batPct < 0) batPct = 0;
    if (batPct > 100) batPct = 100;
  }

  if (displayOK && now - lastDraw >= DRAW_INTERVAL_MS) { lastDraw = now; draw(); }

  delay(2);
}
