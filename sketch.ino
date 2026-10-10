// Event-Driven Telemetry + Live Blynk Dashboard (ESP32, Wokwi)
// - Sends only when something meaningful happens (report-by-exception).
// - Every event goes into a fixed-size queue and is sent in order; while offline the
//   queue simply fills and is replayed after the connection returns.
// - Wi-Fi reconnection is a non-blocking state machine with backoff.
// - RSSI is smoothed, classified (with hysteresis) and used to slow telemetry on poor links.
// No delay() anywhere.

// ---- Blynk credentials: copy from Blynk Console (Device Info) ----
#define BLYNK_TEMPLATE_ID "TMPL3vBvZVzrI"
#define BLYNK_TEMPLATE_NAME "eventdrivenelemetry"
#define BLYNK_AUTH_TOKEN    "xxxxxxxxx"
#define BLYNK_PRINT Serial   // uncomment to see Blynk's own connection messages (for troubleshooting)

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <stdarg.h>

// ---------- Types (declared first so Arduino auto-prototypes can see them) ----------
enum NetState : uint8_t { NET_OFFLINE_WAIT, NET_WIFI_CONNECTING, NET_BLYNK_CONNECTING, NET_ONLINE, NET_OUTAGE_SIM, NUM_NET };
enum EvType   : uint8_t { EV_BOOT, EV_CELLS, EV_EXTREMES, EV_RELAY, EV_STATE, EV_LINK, EV_NET_DOWN, EV_NET_UP, EV_HEARTBEAT, NUM_EV };
enum SysState : uint8_t { SYS_NORMAL, SYS_WARNING, SYS_FAULT };
enum FaultCode : uint8_t { FC_NONE, FC_LOW_CELL, FC_CELL_UV, FC_CELL_OV, FC_IMBAL, FC_RELAY, NUM_FC };
enum Link     : uint8_t { LQ_EXCELLENT, LQ_GOOD, LQ_FAIR, LQ_POOR, LQ_BAD, LQ_DOWN };
enum Inject   : uint8_t { INJ_NONE, INJ_CELL_DROP, INJ_IMBALANCE, INJ_RELAY_FAULT, NUM_INJ };
struct Rec {                       // one telemetry event, 24 bytes
  uint32_t seq, ts;                // sequence number and creation time (ms)
  uint16_t mv[3];                  // cell voltages (mV)
  uint16_t aux;                    // extra value (outage seconds, link class)
  uint8_t type, weak, strong, relay, state, fcode;
  int8_t rssi;
  uint8_t flags;                   // FLAG_GAP: older records were dropped before this one
};

const char* NET_NAME[]  = {"OFFLINE_WAIT", "WIFI_CONNECTING", "BLYNK_CONNECTING", "ONLINE", "OUTAGE_SIM"};
const char* EV_NAME[]   = {"BOOT", "CELLS", "EXTREMES", "RELAY", "STATE", "LINK", "NET_DOWN", "NET_UP", "HEARTBEAT"};
const bool  EV_CRIT[]   = {true, false, false, true, true, false, true, true, false};   // kept longest when full
const char* SYS_NAME[]  = {"NORMAL", "WARNING", "FAULT"};
const char* FC_NAME[]   = {"NONE", "LOW_CELL", "CELL_UV", "CELL_OV", "IMBALANCE", "RELAY"};
const char* LQ_NAME[]   = {"EXCELLENT", "GOOD", "FAIR", "POOR", "BAD", "DOWN"};
const char* INJ_NAME[]  = {"NONE", "CELL_DROP", "IMBALANCE", "RELAY_FAULT"};

// ---------- Pins ----------
const uint8_t CELL_PIN[3] = {34, 35, 32};   // pots = 3 battery cells
#define ATTEN_PIN 33                         // pot = signal attenuation (simulated RSSI drop)
#define RELAY_PIN 19
#define RELAY_ACTIVE_LOW true
#define RED_LED 2
#define GREEN_LED 4
#define YELLOW_LED 5
#define BUZZER 18
#define BTN_INJECT 25                        // cycle injected fault
#define BTN_OUTAGE 26                        // toggle simulated network outage
#define BTN_BURST 27                         // flood the queue to show overflow handling

// ---------- Blynk datastreams (virtual pins) ----------
const uint8_t VP_CELL0 = 0;                  // V0,V1,V2 cell voltages
const uint8_t VP_EXTREMES = 3, VP_RELAY = 4, VP_STATE = 5, VP_WIFI = 6, VP_DEPTH = 7, VP_MODE = 8, VP_JOURNAL = 9;
const uint8_t NSTREAMS = 9;                  // streams V0..V8 are mirrors; V9 is the event journal

// ---------- Tuning ----------
const uint8_t  QCAP = 32;                              // offline queue size (records)
const uint32_t SAMPLE_MS = 100, LCD_MS = 250, STAT_MS = 5000;
const uint32_t TX_FAST_MS = 125, TX_SLOW_MS = 500;     // 8 writes/s normally, 2/s on poor link (Blynk limit: 10/s)
const uint16_t CELL_DELTA_MV = 50;                     // report a cell only if it moved this much
const uint32_t COALESCE_MS = 2000;                     // merge cell updates inside this window
const uint32_t HEARTBEAT_MS = 300000;                  // liveness record if nothing else happened (5 min)
const uint32_t LIVE_AGE_MS = 1500;                     // older than this when sent = QUEUED
const uint32_t LQ_HOLD_MS = 1500;                      // a new link class must hold this long before it counts
const uint32_t WIFI_TIMEOUT_MS = 10000, BACKOFF_MIN = 1000, BACKOFF_MAX = 30000;
const uint16_t BLYNK_CONNECT_CAP_MS = 5000;            // the library's connect() call is bounded by this (Wokwi's gateway can be slow)
const uint8_t  FLAG_GAP = 1;
const char* WIFI_SSID = "Wokwi-GUEST";
#define RSSI_SIMULATED true                            // Wokwi's RSSI is fixed, so emulate it. Set false on real hardware
const int   SIM_RSSI_BASE = -50;
const float MAX_ATTEN_DB = 45.0;

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------- Data ----------
uint16_t mv[3] = {3600, 3600, 3600}, repMv[3] = {3600, 3600, 3600};   // current / last reported
uint8_t  weak = 0, strong = 0, sysState = SYS_NORMAL, fcode = FC_NONE, inject = INJ_NONE;
bool     relayClosed = true, fUv = false, fOv = false, fLow = false, fImb = false;
bool     clearTimerOn = false; uint32_t fltClearSince = 0;
int      rssi = SIM_RSSI_BASE; float rssiF = SIM_RSSI_BASE, attenDb = 0; uint8_t lq = LQ_EXCELLENT, lqCand = LQ_EXCELLENT; uint32_t lqCandT = 0;

// Queue
Rec      q[QCAP];
uint8_t  qHead = 0, qCount = 0;
uint32_t seqNo = 0, inflightSeq = 0, lastEventMs = 0;
uint32_t enqueued = 0, sentCount = 0, dropped = 0, coalesced = 0, msgs = 0;
bool     gapPending = false;

// Network
uint8_t  net = NET_OFFLINE_WAIT;
uint32_t retryAt = 0, backoffMs = BACKOFF_MIN, wifiT0 = 0, offlineSince = 0;
bool     outageSim = false;

// What the dashboard currently shows (so we write only what changed)
uint16_t shownMv[3]; uint8_t shownWeak, shownStrong, shownSys, shownFc, shownLq; int8_t shownRelay; int shownRssi, shownDepth;
uint16_t shownWeakMv, shownStrongMv; uint32_t tDepth = 0; char shownMode[12];
uint32_t maxLoopUs = 0, stalls = 0;

// ---------- Structured log (JSON lines) ----------
void logj(const char* ev, const char* fmt, ...) {
  char body[190];
  va_list ap; va_start(ap, fmt); vsnprintf(body, sizeof body, fmt, ap); va_end(ap);
  Serial.printf("{\"t_ms\":%lu,\"ev\":\"%s\",%s}\n", (unsigned long)millis(), ev, body);
}

// ---------- Fixed-size queue ----------
Rec& qAt(uint8_t i) { return q[(qHead + i) % QCAP]; }
void qPop() { qHead = (qHead + 1) % QCAP; qCount--; }
void qRemoveAt(uint8_t i) {
  if (i == 0) { qPop(); return; }
  for (uint8_t k = i; k + 1 < qCount; k++) qAt(k) = qAt(k + 1);
  qCount--;
}
void qPush(const Rec& r) {
  if (qCount == QCAP && inflightSeq && qAt(0).seq == inflightSeq && Blynk.connected()) {
    qPop(); sentCount++; inflightSeq = 0;               // head was already transmitted: commit it, don't call it a drop
  }
  if (qCount == QCAP) {                                 // full: drop the oldest non-critical record,
    uint8_t victim = 0;                                 // or the oldest of all if every record is critical
    for (uint8_t i = 0; i < qCount; i++) if (!EV_CRIT[qAt(i).type]) { victim = i; break; }
    Rec& v = qAt(victim);
    if (v.seq == inflightSeq) inflightSeq = 0;
    dropped++;
    if (victim + 1 < qCount) qAt(victim + 1).flags |= FLAG_GAP;   // mark where data is missing
    else gapPending = true;                                      // victim was the newest: mark the next record
    logj("TELEM", "\"action\":\"DROP\",\"seq\":%lu,\"type\":\"%s\",\"critical\":%s,\"dropped_total\":%lu",
         (unsigned long)v.seq, EV_NAME[v.type], EV_CRIT[v.type] ? "true" : "false", (unsigned long)dropped);
    qRemoveAt(victim);
  }
  qAt(qCount) = r; qCount++;
}

// ---------- Creating events ----------
void fillSnapshot(Rec& r) {
  for (uint8_t i = 0; i < 3; i++) r.mv[i] = mv[i];
  r.weak = weak; r.strong = strong; r.relay = relayClosed; r.state = sysState; r.fcode = fcode; r.rssi = (int8_t)rssi;
}
void enqueue(uint8_t type, uint16_t aux) {
  uint32_t now = millis();
  lastEventMs = now;
  for (uint8_t i = 0; i < 3; i++) repMv[i] = mv[i];     // these values are now "reported"
  if (type == EV_CELLS && qCount) {                     // merge rapid cell changes into the newest record
    Rec& t = qAt(qCount - 1);
    uint32_t win = COALESCE_MS * (lq >= LQ_POOR ? 5 : 1);
    if (t.type == EV_CELLS && t.seq != inflightSeq && now - t.ts < win) {
      fillSnapshot(t); coalesced++;
      logj("TELEM", "\"action\":\"COALESCE\",\"seq\":%lu,\"type\":\"CELLS\",\"depth\":%u", (unsigned long)t.seq, qCount);
      return;
    }
  }
  Rec r; memset(&r, 0, sizeof r);
  r.seq = ++seqNo; r.ts = now; r.type = type; r.aux = aux;
  fillSnapshot(r);
  if (gapPending) { r.flags |= FLAG_GAP; gapPending = false; }
  qPush(r); enqueued++;
  logj("TELEM", "\"action\":\"ENQUEUE\",\"seq\":%lu,\"type\":\"%s\",\"depth\":%u,\"net\":\"%s\"",
       (unsigned long)r.seq, EV_NAME[type], qCount, NET_NAME[net]);
}

// ---------- Cells, faults and events (100 ms task) ----------
bool wifiUpNow() { return WiFi.status() == WL_CONNECTED; }

void readCells() {
  for (uint8_t i = 0; i < 3; i++) {
    float v = 2800 + analogRead(CELL_PIN[i]) / 4095.0 * 1600 + random(-1, 2);
    if (inject == INJ_CELL_DROP && i == 1) v -= 800;
    if (inject == INJ_IMBALANCE && i == 2) v += 400;
    mv[i] = (uint16_t)constrain(v, 2000.0, 4500.0);
  }
}

void relayDrive(bool closed) {
  relayClosed = closed;
  digitalWrite(RELAY_PIN, (closed != RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

void monitorTask() {
  uint32_t now = millis();
  readCells();
  uint8_t cw = 0, cs = 0;
  for (uint8_t i = 1; i < 3; i++) { if (mv[i] < mv[cw]) cw = i; if (mv[i] > mv[cs]) cs = i; }
  uint16_t lo = mv[cw], hi = mv[cs];
  bool extChanged = false;
  if (cw != weak && lo + 20 < mv[weak]) { weak = cw; extChanged = true; }        // 20 mV hysteresis so
  if (cs != strong && hi > mv[strong] + 20) { strong = cs; extChanged = true; }  // near-equal cells don't flap

  if (lo < 3000) fUv = true; else if (lo > 3050) fUv = false;
  if (hi > 4250) fOv = true; else if (hi < 4200) fOv = false;
  if (lo < 3300) fLow = true; else if (lo > 3400) fLow = false;
  if (hi - lo > 250) fImb = true; else if (hi - lo < 180) fImb = false;
  bool fRly = inject == INJ_RELAY_FAULT;
  uint8_t newFc = fRly ? FC_RELAY : fUv ? FC_CELL_UV : fOv ? FC_CELL_OV : fImb ? FC_IMBAL : fLow ? FC_LOW_CELL : FC_NONE;
  uint8_t newSys = (fRly || fUv || fOv) ? SYS_FAULT : (fImb || fLow) ? SYS_WARNING : SYS_NORMAL;
  bool stateChanged = (newSys != sysState || newFc != fcode);
  sysState = newSys; fcode = newFc;

  bool relayChanged = false;                              // open on FAULT, re-close 3 s after it clears
  if (sysState == SYS_FAULT) { clearTimerOn = false; if (relayClosed) { relayDrive(false); relayChanged = true; } }
  else if (!relayClosed) {
    if (!clearTimerOn) { clearTimerOn = true; fltClearSince = now; }
    else if (now - fltClearSince >= 3000) { relayDrive(true); relayChanged = true; }
  }

  if (stateChanged) enqueue(EV_STATE, 0);                 // events, in cause-and-effect order
  if (relayChanged) enqueue(EV_RELAY, 0);
  if (extChanged) enqueue(EV_EXTREMES, 0);
  bool delta = false;
  for (uint8_t i = 0; i < 3; i++) if (abs((int)mv[i] - (int)repMv[i]) >= CELL_DELTA_MV) delta = true;
  if (delta) enqueue(EV_CELLS, 0);
  if (now - lastEventMs >= HEARTBEAT_MS) enqueue(EV_HEARTBEAT, 0);
}

// ---------- RSSI: smoothing, classification with hysteresis ----------
uint8_t classify(float r) { return r >= -55 ? LQ_EXCELLENT : r >= -67 ? LQ_GOOD : r >= -75 ? LQ_FAIR : r >= -85 ? LQ_POOR : LQ_BAD; }

void linkTask() {
  attenDb = analogRead(ATTEN_PIN) / 4095.0 * MAX_ATTEN_DB;
  float raw = RSSI_SIMULATED ? (SIM_RSSI_BASE - attenDb) : (wifiUpNow() ? WiFi.RSSI() : -100);
  rssiF += 0.2 * (raw - rssiF);
  rssi = (int)lroundf(rssiF);
  uint8_t better = classify(rssiF - 3), same = classify(rssiF), old = lq, target = lq;
  if (better < lq) target = better;                       // move up only with a 3 dB margin
  else if (same > lq) target = same;                      // move down when the signal falls
  if (target != lq) {                                     // ...but only after it has held for LQ_HOLD_MS
    if (target != lqCand) { lqCand = target; lqCandT = millis(); }
    else if (millis() - lqCandT >= LQ_HOLD_MS) lq = target;
  } else lqCand = lq;
  if (lq != old && wifiUpNow()) {
    logj("LINK", "\"rssi_dbm\":%d,\"from\":\"%s\",\"to\":\"%s\"", rssi, LQ_NAME[old], LQ_NAME[lq]);
    enqueue(EV_LINK, lq);
  }
}

// ---------- Non-blocking reconnection state machine ----------
void netGo(uint8_t to, const char* why) {
  logj("NET", "\"from\":\"%s\",\"to\":\"%s\",\"reason\":\"%s\",\"backoff_ms\":%lu,\"depth\":%u",
       NET_NAME[net], NET_NAME[to], why, (unsigned long)backoffMs, qCount);
  net = to;
}
void scheduleRetry() { retryAt = millis() + backoffMs; backoffMs = min(backoffMs * 2, BACKOFF_MAX); }

void shownReset() {                                      // force a full dashboard refresh
  for (uint8_t i = 0; i < 3; i++) shownMv[i] = 0xFFFF;
  shownWeak = shownStrong = shownSys = shownFc = shownLq = 255; shownRelay = -1; shownRssi = -999; shownDepth = -1;
  shownWeakMv = shownStrongMv = 0xFFFF; shownMode[0] = 0;
}
void onOffline() { offlineSince = millis(); enqueue(EV_NET_DOWN, 0); }
void onOnline() {
  uint32_t sec = offlineSince ? (millis() - offlineSince) / 1000 : 0;
  offlineSince = 0; backoffMs = BACKOFF_MIN;
  shownReset();
  enqueue(EV_NET_UP, (uint16_t)min(sec, (uint32_t)65535));
}

void netTask() {
  uint32_t now = millis();
  bool wifiUp = wifiUpNow();
  if (outageSim && net != NET_OUTAGE_SIM) {              // operator started an outage
    if (net == NET_ONLINE) onOffline();
    Blynk.disconnect(); WiFi.disconnect(true);
    netGo(NET_OUTAGE_SIM, "simulated outage"); return;
  }
  switch (net) {
    case NET_OFFLINE_WAIT:
      if ((int32_t)(now - retryAt) >= 0) {
        if (wifiUp) netGo(NET_BLYNK_CONNECTING, "wifi already up");
        else { WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, "", 6); wifiT0 = now; netGo(NET_WIFI_CONNECTING, "retry"); }
      }
      break;
    case NET_WIFI_CONNECTING:
      if (wifiUp) netGo(NET_BLYNK_CONNECTING, "wifi connected");
      else if (now - wifiT0 >= WIFI_TIMEOUT_MS) { WiFi.disconnect(); scheduleRetry(); netGo(NET_OFFLINE_WAIT, "wifi timeout"); }
      break;
    case NET_BLYNK_CONNECTING:
      if (!wifiUp) { scheduleRetry(); netGo(NET_OFFLINE_WAIT, "wifi lost"); break; }
      if (Blynk.connect(BLYNK_CONNECT_CAP_MS)) { onOnline(); netGo(NET_ONLINE, "blynk connected"); }
      else { scheduleRetry(); netGo(NET_OFFLINE_WAIT, "blynk connect failed"); }
      break;
    case NET_ONLINE:
      Blynk.run();
      if (!wifiUp || !Blynk.connected()) {
        onOffline(); backoffMs = BACKOFF_MIN; scheduleRetry();
        netGo(NET_OFFLINE_WAIT, !wifiUp ? "wifi lost" : "blynk lost");
      }
      break;
    case NET_OUTAGE_SIM:
      if (!outageSim) { backoffMs = BACKOFF_MIN; retryAt = now; netGo(NET_OFFLINE_WAIT, "outage ended"); }
      break;
  }
}

// ---------- Dashboard: mirrors (what changed) and the ordered journal ----------
uint8_t lqShown() { return net == NET_ONLINE ? lq : LQ_DOWN; }
int  depthShown() {                                       // backlog = records already older than "live"
  int n = 0; uint32_t now = millis();
  for (uint8_t i = 0; i < qCount; i++) if (now - qAt(i).ts > LIVE_AGE_MS) n++;
  return n;
}
bool replaying() { return depthShown() > 0; }
const char* modeNow() { return replaying() ? "REPLAYING" : "LIVE"; }

bool streamDiffers(uint8_t i) {
  uint32_t now = millis();
  switch (i) {
    case 0: case 1: case 2: return abs((int)mv[i] - (int)shownMv[i]) >= CELL_DELTA_MV;
    case 3: return shownWeak != weak || shownStrong != strong ||
                   abs((int)mv[weak] - (int)shownWeakMv) >= CELL_DELTA_MV || abs((int)mv[strong] - (int)shownStrongMv) >= CELL_DELTA_MV;
    case 4: return shownRelay != (int8_t)relayClosed;
    case 5: return shownSys != sysState || shownFc != fcode;
    case 6: return shownLq != lqShown() || abs(rssi - shownRssi) >= 4;
    case 7: { int d = depthShown(); return d != shownDepth && (d == 0 || shownDepth <= 0 || now - tDepth >= 1000); }
    case 8: return strcmp(shownMode, modeNow()) != 0;
  }
  return false;
}
void streamSend(uint8_t i) {
  char b[48];
  switch (i) {
    case 0: case 1: case 2: Blynk.virtualWrite(VP_CELL0 + i, mv[i] / 1000.0f); shownMv[i] = mv[i]; break;
    case 3:
      snprintf(b, sizeof b, "Weak C%u %u.%02u | Strong C%u %u.%02u", weak + 1, mv[weak] / 1000, (mv[weak] % 1000) / 10,
               strong + 1, mv[strong] / 1000, (mv[strong] % 1000) / 10);
      Blynk.virtualWrite(VP_EXTREMES, b);
      shownWeak = weak; shownStrong = strong; shownWeakMv = mv[weak]; shownStrongMv = mv[strong]; break;
    case 4: Blynk.virtualWrite(VP_RELAY, relayClosed ? 1 : 0); shownRelay = relayClosed; break;
    case 5:
      if (sysState == SYS_NORMAL) snprintf(b, sizeof b, "NORMAL");
      else snprintf(b, sizeof b, "%s %s", SYS_NAME[sysState], FC_NAME[fcode]);
      Blynk.virtualWrite(VP_STATE, b); shownSys = sysState; shownFc = fcode; break;
    case 6:
      snprintf(b, sizeof b, "%s %ddBm", LQ_NAME[lqShown()], rssi);
      Blynk.virtualWrite(VP_WIFI, b); shownLq = lqShown(); shownRssi = rssi; break;
    case 7: Blynk.virtualWrite(VP_DEPTH, depthShown()); shownDepth = depthShown(); tDepth = millis(); break;
    case 8: snprintf(shownMode, sizeof shownMode, "%s", modeNow()); Blynk.virtualWrite(VP_MODE, shownMode); break;
  }
  msgs++;
}

void fmtRec(const Rec& r, char* out, size_t n, uint32_t age) {
  char tag[16], body[72];
  if (age <= LIVE_AGE_MS) snprintf(tag, sizeof tag, "[LIVE]"); else snprintf(tag, sizeof tag, "[Q -%lus]", (unsigned long)(age / 1000));
  switch (r.type) {
    case EV_CELLS:    snprintf(body, sizeof body, "CELLS %u.%02u/%u.%02u/%u.%02uV", r.mv[0] / 1000, (r.mv[0] % 1000) / 10, r.mv[1] / 1000, (r.mv[1] % 1000) / 10, r.mv[2] / 1000, (r.mv[2] % 1000) / 10); break;
    case EV_EXTREMES: snprintf(body, sizeof body, "EXTREMES weak C%u strong C%u", r.weak + 1, r.strong + 1); break;
    case EV_RELAY:    snprintf(body, sizeof body, "RELAY %s", r.relay ? "CLOSED" : "OPEN"); break;
    case EV_STATE:    snprintf(body, sizeof body, "STATE %s %s", SYS_NAME[r.state], FC_NAME[r.fcode]); break;
    case EV_LINK:     snprintf(body, sizeof body, "LINK %s %ddBm", LQ_NAME[r.aux < 6 ? r.aux : 5], r.rssi); break;
    case EV_NET_UP:   snprintf(body, sizeof body, "NET UP after %us", r.aux); break;
    default:          snprintf(body, sizeof body, "%s", EV_NAME[r.type]); break;
  }
  snprintf(out, n, "%s #%lu %s%s\n", tag, (unsigned long)r.seq, body, (r.flags & FLAG_GAP) ? " GAP" : "");
}
void sendRecord(const Rec& r) {
  uint32_t age = millis() - r.ts;
  char line[120]; fmtRec(r, line, sizeof line, age);
  Blynk.virtualWrite(VP_JOURNAL, line); msgs++;
  inflightSeq = r.seq;
  logj("TELEM", "\"action\":\"SENT\",\"seq\":%lu,\"type\":\"%s\",\"data\":\"%s\",\"age_ms\":%lu,\"depth\":%u",
       (unsigned long)r.seq, EV_NAME[r.type], age <= LIVE_AGE_MS ? "LIVE" : "QUEUED", (unsigned long)age, qCount);
}

// One Blynk write per tick, so the rate stays under the 10 messages/s limit.
void txTask() {
  static uint32_t t = 0;
  uint32_t now = millis();
  if (now - t < (lq >= LQ_POOR ? TX_SLOW_MS : TX_FAST_MS)) return;
  t = now;
  if (net != NET_ONLINE) { inflightSeq = 0; return; }
  if (inflightSeq) {                                      // commit the previous record only if still connected
    if (Blynk.connected() && qCount && qAt(0).seq == inflightSeq) { qPop(); sentCount++; }
    inflightSeq = 0;
  }
  for (uint8_t i = 0; i < NSTREAMS; i++) if (streamDiffers(i)) { streamSend(i); return; }
  if (qCount) sendRecord(qAt(0));
}

// ---------- Buttons, LEDs, LCD ----------
const uint8_t BTN_PIN[3] = {BTN_INJECT, BTN_OUTAGE, BTN_BURST};
bool bStable[3] = {HIGH, HIGH, HIGH}, bLast[3] = {HIGH, HIGH, HIGH};
uint32_t bT[3] = {0, 0, 0};
bool btnPressed(uint8_t k) {
  bool r = digitalRead(BTN_PIN[k]); uint32_t now = millis();
  if (r != bLast[k]) { bLast[k] = r; bT[k] = now; }
  if (r != bStable[k] && now - bT[k] >= 30) { bStable[k] = r; return r == LOW; }
  return false;
}
void burstTest() {                                       // 40 cell events + 4 critical markers
  logj("BURST", "\"msg\":\"flooding queue to show overflow handling\",\"depth_before\":%u", qCount);
  for (uint8_t i = 0; i < 40; i++) {
    mv[0] = 3300 + i * 10;
    Rec r; memset(&r, 0, sizeof r);
    r.seq = ++seqNo; r.ts = millis(); r.type = EV_CELLS; fillSnapshot(r);
    if (gapPending) { r.flags |= FLAG_GAP; gapPending = false; }
    qPush(r); enqueued++;
    logj("TELEM", "\"action\":\"ENQUEUE\",\"seq\":%lu,\"type\":\"CELLS\",\"depth\":%u,\"net\":\"%s\"", (unsigned long)r.seq, qCount, NET_NAME[net]);
    if (i % 13 == 0) enqueue(EV_RELAY, i);
  }
}
void buttonsTask() {
  if (btnPressed(0)) { inject = (inject + 1) % NUM_INJ; logj("INJECT", "\"mode\":\"%s\"", INJ_NAME[inject]); }
  if (btnPressed(1)) { outageSim = !outageSim; logj("OUTAGE", "\"simulated\":%s", outageSim ? "true" : "false"); }
  if (btnPressed(2)) burstTest();
}
void outputsTask() {
  uint32_t now = millis();
  bool on = net == NET_ONLINE, rep = on && replaying();
  digitalWrite(GREEN_LED, on && !rep && lq < LQ_POOR);
  digitalWrite(YELLOW_LED, rep || (on && lq >= LQ_POOR));
  digitalWrite(RED_LED, !on);
  digitalWrite(BUZZER, sysState == SYS_FAULT && (now % 1000) < 150);
}
void lcdLine(uint8_t row, const char* s) {               // rewrite a line only if it changed
  static char last[2][17];
  char b[17]; snprintf(b, sizeof b, "%-16s", s);
  if (strcmp(b, last[row]) == 0) return;
  strcpy(last[row], b); lcd.setCursor(0, row); lcd.print(b);
}
void lcdTask() {
  char a[24], b[24];
  const char* mode = net != NET_ONLINE ? "OFFL" : replaying() ? "REPL" : "LIVE";
  snprintf(a, sizeof a, "%-7s Q:%02d %s", SYS_NAME[sysState], qCount > 99 ? 99 : qCount, mode);
  if (net == NET_ONLINE) snprintf(b, sizeof b, "WiFi:%-4s %4ddB", LQ_NAME[lq], rssi);
  else if (net == NET_OUTAGE_SIM) snprintf(b, sizeof b, "SIM OUTAGE ON");
  else if (net == NET_WIFI_CONNECTING) snprintf(b, sizeof b, "Connecting WiFi");
  else if (net == NET_BLYNK_CONNECTING) snprintf(b, sizeof b, "Connecting Blynk");
  else snprintf(b, sizeof b, "OFFL retry in %2lus", (unsigned long)(((int32_t)(retryAt - millis()) > 0 ? retryAt - millis() : 0) / 1000));
  lcdLine(0, a); lcdLine(1, b);
}

void setup() {
  Serial.begin(115200);
  pinMode(RED_LED, OUTPUT); pinMode(GREEN_LED, OUTPUT); pinMode(YELLOW_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT); pinMode(RELAY_PIN, OUTPUT);
  for (uint8_t i = 0; i < 3; i++) pinMode(BTN_PIN[i], INPUT_PULLUP);
  lcd.init(); lcd.backlight();
  WiFi.persistent(false); WiFi.setAutoReconnect(false);
  Blynk.config(BLYNK_AUTH_TOKEN);                        // config only: no blocking begin()
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? LOW : HIGH);
  shownReset();
  readCells();
  for (uint8_t i = 0; i < 3; i++) repMv[i] = mv[i];
  for (uint8_t i = 1; i < 3; i++) { if (mv[i] < mv[weak]) weak = i; if (mv[i] > mv[strong]) strong = i; }
  offlineSince = millis(); retryAt = millis(); lastEventMs = millis();
  enqueue(EV_BOOT, 0);
  logj("BOOT", "\"msg\":\"event-driven telemetry started\",\"queue_capacity\":%u,\"record_bytes\":%u", QCAP, (unsigned)sizeof(Rec));
}

void loop() {
  static uint32_t tSample = 0, tLcd = 0, tStat = 0, lastUs = 0;
  uint32_t now = millis(), us = micros();
  if (lastUs) { uint32_t dt = us - lastUs; if (dt > maxLoopUs) maxLoopUs = dt; if (dt > 50000) stalls++; }
  lastUs = us;

  netTask();
  if (now - tSample >= SAMPLE_MS) {
    tSample += SAMPLE_MS; if (now - tSample > 5 * SAMPLE_MS) tSample = now;
    monitorTask();
    linkTask();
  }
  txTask();
  buttonsTask();
  outputsTask();
  if (now - tLcd >= LCD_MS) { tLcd = now; lcdTask(); }
  if (now - tStat >= STAT_MS) {
    tStat = now;
    logj("STAT", "\"net\":\"%s\",\"depth\":%u,\"enqueued\":%lu,\"sent\":%lu,\"dropped\":%lu,\"coalesced\":%lu,\"blynk_msgs\":%lu,\"rssi_dbm\":%d,\"link\":\"%s\",\"max_loop_us\":%lu,\"stalls\":%lu",
         NET_NAME[net], qCount, (unsigned long)enqueued, (unsigned long)sentCount, (unsigned long)dropped, (unsigned long)coalesced,
         (unsigned long)msgs, rssi, LQ_NAME[lq], (unsigned long)maxLoopUs, (unsigned long)stalls);
  }
}
