#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Preferences.h>
#include <vl53l7cx_class.h>

// ---------------- Direction helpers ----------------
enum Direction { DIR_LEFT, DIR_SLIGHTLY_LEFT, DIR_CENTER, DIR_SLIGHTLY_RIGHT, DIR_RIGHT, DIR_UNKNOWN };

static inline Direction dirFromColFW(int col){
  float x = col / 7.0f;
  if (x < 0.20f) return DIR_LEFT;
  if (x < 0.40f) return DIR_SLIGHTLY_LEFT;
  if (x < 0.60f) return DIR_CENTER;
  if (x < 0.80f) return DIR_SLIGHTLY_RIGHT;
  return DIR_RIGHT;
}

static inline uint8_t directionToTrackIndex(Direction d){
  // 1=center, 2=left, 3=right, 4=slightly
  switch (d){
    case DIR_CENTER: return 1;
    case DIR_LEFT:   return 2;
    case DIR_RIGHT:  return 3;
    case DIR_SLIGHTLY_LEFT:
    case DIR_SLIGHTLY_RIGHT: return 4;
    default: return 0;
  }
}

static inline const char* dirToText(Direction d){
  switch(d){
    case DIR_CENTER:          return "center";
    case DIR_LEFT:            return "left";
    case DIR_RIGHT:           return "right";
    case DIR_SLIGHTLY_LEFT:   return "slightly left";
    case DIR_SLIGHTLY_RIGHT:  return "slightly right";
    default:                  return "—";
  }
}

// ---------------- Pins & config ----------------
static const int SDA_PIN = 21;
static const int SCL_PIN = 22;
static const int LPN_PIN = -1;      // tie to 3V3 if not used

// JQ6500 on UART2
static const int JQ_RX = 16;        // ESP32 RX2  <- JQ6500 TX
static const int JQ_TX = 17;        // ESP32 TX2  -> JQ6500 RX

// Wi-Fi AP
const char* AP_SSID = "SmartCane_AP";
const char* AP_PASS = "12345678";
WebServer server(80);

// ---------------- User settings (persisted) ----------------
Preferences prefs;
struct Settings {
  uint16_t range_cm;      // speak/visualize up to this distance (cm)
  uint8_t  freq_hz;       // ranging frequency (Hz)
  // Calibration
  int16_t  cal_offset_cm; // add after scaling (can be negative)
  uint8_t  cal_scale_pct; // 100 = no scale; applies to mm before offset
  // Web speech toggle
  uint8_t  speak_web;     // 0/1
};
Settings g_set;

// derived runtime values
static uint16_t g_maxRangeMm = 200;            // default 20cm
static uint32_t g_frameIntervalMs = 500;       // default 2 Hz

// bounds
static const uint16_t RANGE_CM_MIN = 5;        // 5 cm
static const uint16_t RANGE_CM_MAX = 200;      // 2 m
static const uint8_t  FREQ_HZ_MIN  = 1;        // 1 Hz
static const uint8_t  FREQ_HZ_MAX  = 10;       // 10 Hz
static const int16_t  CAL_OFF_MIN  = -200;     // -200 cm
static const int16_t  CAL_OFF_MAX  =  200;     // +200 cm
static const uint8_t  CAL_SCL_MIN  = 50;       // 50%
static const uint8_t  CAL_SCL_MAX  = 150;      // 150%

// Audio debounce
static const uint32_t COOLDOWN_MS = 1500;
static const uint16_t DELTA_CM_FOR_REPEAT = 5;

// ---------------- Sensor & MP3 ----------------
static const uint16_t VL53L7CX_I2C_ADDR = 0x52;
VL53L7CX tof(&Wire, LPN_PIN, VL53L7CX_I2C_ADDR);

HardwareSerial MP3(2);
static inline void jqSend(uint8_t cmd, int nArgs=0, uint8_t a1=0, uint8_t a2=0){
  uint8_t len = 1 + nArgs + 1;
  uint8_t buf[6]; int i=0;
  buf[i++]=0x7E; buf[i++]=len; buf[i++]=cmd;
  if(nArgs>=1) buf[i++]=a1;
  if(nArgs>=2) buf[i++]=a2;
  buf[i++]=0xEF;
  MP3.write(buf,i);
  MP3.flush();
}
static inline void jqSetVolume(uint8_t v){ if(v>30) v=30; jqSend(0x06,1,v); }
static inline void jqPlayIndex(uint16_t idx){ jqSend(0x03,2,uint8_t(idx>>8),uint8_t(idx&0xFF)); }

// ---------------- Shared (cached) frame ----------------
struct Frame {
  uint16_t grid[64];      // clipped to 0..range_mm (0=invalid/far) AFTER calibration
  uint16_t nearest_mm;    // 0 if none in range (AFTER calibration)
  int      nearest_idx;   // 0..63 or -1
  Direction dir;
  uint32_t  timestamp_ms; // millis() of last update
  bool      valid;
} g_frame;

// ---------------- UI (heatmap + editable settings + calibration + web speech) ----------------
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="en"><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Smart Cane — 8×8 Heatmap</title>
<style>
  :root{font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif}
  body{margin:0;padding:20px;background:#0b0d14;color:#e8ecf1}
  .wrap{max-width:920px;margin:0 auto}
  h1{margin:0 0 6px;font-size:1.25rem}
  .row{display:flex;gap:12px;align-items:center;flex-wrap:wrap}
  .badge{display:inline-block;padding:4px 10px;border-radius:999px;background:#1f2a44;color:#a9c1ff;font-size:.85rem}
  .grid{margin-top:14px;display:grid;grid-template-columns:repeat(8,1fr);gap:6px}
  .cell{aspect-ratio:1/1;background:#0e1422;border-radius:6px;display:flex;align-items:center;justify-content:center;font-size:11px;color:#c8d6f0}
  .metric{font-size:2.2rem;font-weight:700}
  .muted{color:#8ea2c4}
  .footer{margin-top:12px;font-size:.85rem;color:#7588a8}
  input,select{background:#0e1422;color:#e8ecf1;border:1px solid #2b3653;border-radius:8px;padding:8px}
  button{background:#2c7be5;color:#fff;border:none;padding:8px 12px;border-radius:8px;font-weight:600;cursor:pointer}
  .card{background:#121826;border-radius:14px;padding:12px;margin-top:12px}
  label{display:flex;align-items:center;gap:8px}
  .col{display:flex;gap:10px;flex-wrap:wrap}
  small{color:#8ea2c4}
</style>
</head><body>
<div class="wrap">
  <h1>Smart Cane — 8×8 Heatmap</h1>
  <div class="row"><span class="badge" id="stat">Sensor: …</span><span class="badge">AP: SmartCane_AP</span></div>

  <div class="card">
    <div class="col">
      <label>Detection range (cm) <input id="range" type="number" min="5" max="200" step="1" value="20" style="width:100px"/></label>
      <label>Frequency (Hz) <input id="freq" type="number" min="1" max="10" step="1" value="2" style="width:90px"/></label>
      <label>Scale (%) <input id="scale" type="number" min="50" max="150" step="1" value="100" style="width:90px"/></label>
      <label>Offset (cm) <input id="offset" type="number" min="-200" max="200" step="1" value="0" style="width:90px"/></label>
      <label><input id="speak" type="checkbox"/> Speak in browser</label>
      <button id="save">Save</button><span class="muted" id="saveMsg"></span>
    </div>
    <div class="muted" style="margin-top:6px">
      <small>Calibration formula: <em>mm_cal = raw_mm × (scale/100) + offset_cm×10</em></small>
    </div>
  </div>

  <div class="card">
    <b>Calibrate at known distance</b>
    <div class="col" style="margin-top:6px">
      <label>True distance (cm) <input id="truecm" type="number" min="5" max="400" step="1" value="40" style="width:100px"/></label>
      <button id="calNow">Calibrate</button>
      <span class="muted" id="calMsg"></span>
    </div>
    <div class="muted"><small>Hold the cane steady facing a flat target (wall/box) at the typed distance, then press Calibrate.</small></div>
  </div>

  <div class="row" style="margin-top:10px">
    <div class="metric"><span id="near">—</span> <small>cm</small></div>
    <div class="muted" id="dir">—</div>
    <div class="muted" id="when">Waiting for first frame…</div>
  </div>

  <div class="grid" id="grid"></div>
  <div class="footer">JQ6500 plays on-device; optional browser speech mirrors it.</div>
</div>
<script>
const gridEl=document.getElementById('grid'), statEl=document.getElementById('stat');
const nearEl=document.getElementById('near'), dirEl=document.getElementById('dir'), whenEl=document.getElementById('when');
const rangeEl=document.getElementById('range'), freqEl=document.getElementById('freq'),
      scaleEl=document.getElementById('scale'), offsetEl=document.getElementById('offset'),
      speakEl=document.getElementById('speak'), saveBtn=document.getElementById('save'), saveMsg=document.getElementById('saveMsg');
const trueEl=document.getElementById('truecm'), calBtn=document.getElementById('calNow'), calMsg=document.getElementById('calMsg');

for(let i=0;i<64;i++){const c=document.createElement('div');c.className='cell';gridEl.appendChild(c)}
const cells=[...document.querySelectorAll('.cell')];

let pollMs = 500;
let lastSpeakTs = 0, lastSpeakDir = '', lastSpeakCm = 9999;

function loadSettings(){
  fetch('/settings').then(r=>r.json()).then(s=>{
    if(!s.ok) return;
    rangeEl.value = s.range_cm;
    freqEl.value  = s.freq_hz;
    scaleEl.value = s.cal_scale_pct;
    offsetEl.value= s.cal_offset_cm;
    speakEl.checked = !!s.speak_web;

    const f = Math.max(1, Math.min(10, parseInt(freqEl.value||'2',10)));
    pollMs = Math.max(150, Math.min(1000, Math.round(1000/f)));
  });
}
loadSettings();

saveBtn.onclick = ()=>{
  const qs = new URLSearchParams({
    range_cm: String(rangeEl.value||'20'),
    freq_hz:  String(freqEl.value||'2'),
    cal_scale_pct: String(scaleEl.value||'100'),
    cal_offset_cm: String(offsetEl.value||'0'),
    speak_web: (document.getElementById('speak').checked ? '1' : '0')
  });
  fetch('/set?'+qs.toString()).then(r=>r.json()).then(s=>{
    if(s.ok){
      saveMsg.textContent = 'Saved ✓';
      const f = Math.max(1, Math.min(10, s.freq_hz));
      pollMs = Math.max(150, Math.min(1000, Math.round(1000/f)));
      setTimeout(()=>saveMsg.textContent='', 1200);
    } else { saveMsg.textContent = 'Save failed'; }
  }).catch(()=>saveMsg.textContent='Save failed');
};

calBtn.onclick = ()=>{
  const tcm = parseInt(trueEl.value||'40',10);
  if(!(tcm>0)){ calMsg.textContent='Enter a valid distance'; return; }
  calMsg.textContent = 'Calibrating…';
  fetch('/cal_now?true_cm='+tcm).then(r=>r.json()).then(s=>{
    if(s.ok){
      offsetEl.value = s.new_offset_cm;
      scaleEl.value  = s.scale_pct;
      saveMsg.textContent = 'Saved ✓';
      calMsg.textContent = `Measured ~${s.measured_cm} cm → Offset set to ${s.new_offset_cm} cm`;
      setTimeout(()=>{saveMsg.textContent='';},1200);
    } else {
      calMsg.textContent = s.err || 'Calibration failed';
    }
  }).catch(()=>calMsg.textContent='Calibration failed');
};

function colorize(vals){
  const valid = vals.filter(v=>v>0);
  if(!valid.length){ cells.forEach(c=>{c.style.background='#0e1422'; c.textContent='';}); return; }
  const mn=Math.min(...valid), mx=Math.max(...valid), rg=Math.max(1,mx-mn);
  vals.forEach((v,i)=>{
    if(v>0){
      const t=(v-mn)/rg, shade=Math.round(20+(1-t)*220);
      cells[i].style.background=`rgb(${shade*0.4},${shade*0.55},${Math.min(255,shade+30)})`;
      cells[i].textContent=v;
    } else { cells[i].style.background='#0e1422'; cells[i].textContent=''; }
  });
}

function maybeSpeak(direction, cm, range, speakEnabled){
  if(!speakEnabled) return;
  if(!('speechSynthesis' in window)) return;
  const now = performance.now();
  const crossed = (lastSpeakCm > range && cm <= range);
  const dirChanged = (lastSpeakDir !== direction);
  const distChanged = (Math.abs(cm - lastSpeakCm) >= 5);
  const cooled = (now - lastSpeakTs >= 1500);
  if (cm>0 && cm <= range && (crossed || dirChanged || distChanged) && cooled){
    let phrase = '';
    if(direction==='center') phrase = 'center';
    else if(direction==='left') phrase = 'left';
    else if(direction==='right') phrase = 'right';
    else if(direction==='slightly left' || direction==='slightly right') phrase = 'slightly ' + direction.split(' ')[1];
    else phrase = 'obstacle';
    const u = new SpeechSynthesisUtterance(phrase);
    try { speechSynthesis.cancel(); } catch(e){}
    speechSynthesis.speak(u);
    lastSpeakTs = now; lastSpeakDir = direction; lastSpeakCm = cm;
  }
}

function poll(){
  fetch('/frame',{cache:'no-store'}).then(r=>r.json()).then(d=>{
    if(!d.ok){ statEl.textContent='Sensor: not ready'; return; }
    statEl.textContent='Sensor: online';
    colorize(d.grid_mm);
    const cm = d.nearest_cm;
    nearEl.textContent = cm>0? cm : '—';
    dirEl.textContent = d.direction || '—';
    whenEl.textContent = 'Last: '+new Date(d.ts).toLocaleTimeString();
    maybeSpeak(d.direction, cm, Number(rangeEl.value||'20'), d.speak_web===1 || d.speak_web===true || document.getElementById('speak').checked);
  }).catch(()=>{ statEl.textContent='Sensor: error'; })
    .finally(()=>setTimeout(poll, pollMs));
}
poll();
</script>
</body></html>
)HTML";

// ---------------- HTTP handlers ----------------
void sendSettingsJSON(){
  String j;
  j.reserve(192);
  j += F("{\"ok\":true,\"range_cm\":");      j += g_set.range_cm;
  j += F(",\"freq_hz\":");                   j += g_set.freq_hz;
  j += F(",\"cal_offset_cm\":");             j += g_set.cal_offset_cm;
  j += F(",\"cal_scale_pct\":");             j += g_set.cal_scale_pct;
  j += F(",\"speak_web\":");                 j += (int)g_set.speak_web;
  j += F("}");
  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json", j);
}

void handleSettings(){ sendSettingsJSON(); }

void applySettingsToSensor(uint8_t freq_hz){
  tof.vl53l7cx_stop_ranging();
  delay(5);
  tof.vl53l7cx_set_ranging_frequency_hz(freq_hz);
  tof.vl53l7cx_start_ranging();
  Serial.printf("[ToF] Frequency set to %u Hz\n", freq_hz);
}

static inline int clampi(int v, int lo, int hi){ return v<lo?lo : (v>hi?hi:v); }

void handleSet(){
  // parse params
  uint16_t range_cm = g_set.range_cm;
  uint8_t  freq_hz  = g_set.freq_hz;
  int16_t  cal_off  = g_set.cal_offset_cm;
  uint8_t  cal_scl  = g_set.cal_scale_pct;
  uint8_t  speakweb = g_set.speak_web;

  if (server.hasArg("range_cm")) {
    int v = server.arg("range_cm").toInt();
    range_cm = clampi(v, RANGE_CM_MIN, RANGE_CM_MAX);
  }
  if (server.hasArg("freq_hz")) {
    int v = server.arg("freq_hz").toInt();
    freq_hz = clampi(v, FREQ_HZ_MIN, FREQ_HZ_MAX);
  }
  if (server.hasArg("cal_offset_cm")) {
    int v = server.arg("cal_offset_cm").toInt();
    cal_off = clampi(v, CAL_OFF_MIN, CAL_OFF_MAX);
  }
  if (server.hasArg("cal_scale_pct")) {
    int v = server.arg("cal_scale_pct").toInt();
    cal_scl = clampi(v, CAL_SCL_MIN, CAL_SCL_MAX);
  }
  if (server.hasArg("speak_web")) {
    int v = server.arg("speak_web").toInt();
    speakweb = (v != 0) ? 1 : 0;
  }

  bool freqChanged = (freq_hz != g_set.freq_hz);

  // Apply
  g_set.range_cm      = range_cm;
  g_set.freq_hz       = freq_hz;
  g_set.cal_offset_cm = cal_off;
  g_set.cal_scale_pct = cal_scl;
  g_set.speak_web     = speakweb;

  g_maxRangeMm        = (uint16_t)(g_set.range_cm * 10);
  g_frameIntervalMs   = max<uint32_t>(100, (1000 / max<uint8_t>(1, g_set.freq_hz)));

  // Persist
  prefs.putUShort("range_cm",      g_set.range_cm);
  prefs.putUChar ("freq_hz",       g_set.freq_hz);
  prefs.putShort ("cal_off_cm",    g_set.cal_offset_cm);
  prefs.putUChar ("cal_scl_pct",   g_set.cal_scale_pct);
  prefs.putUChar ("speak_web",     g_set.speak_web);

  if (freqChanged) applySettingsToSensor(g_set.freq_hz);

  // respond
  sendSettingsJSON();
}

// ---- Utility: take a quick raw measurement (no clipping, before calibration) ----
// Returns nearest distance in *mm* from N short samples, or 0 on failure.
uint16_t sampleNearestRawMm(uint8_t samples = 4, uint32_t perSampleTimeoutMs = 250){
  uint16_t best = 0;
  for (uint8_t s=0; s<samples; s++){
    uint32_t start = millis();
    while (true){
      uint8_t ready = 0;
      if (tof.vl53l7cx_check_data_ready(&ready) == 0 && ready){
        VL53L7CX_ResultsData r;
        if (tof.vl53l7cx_get_ranging_data(&r) == 0){
          uint16_t min_mm = 0;
          for (int i=0;i<64;i++){
            uint16_t d = r.distance_mm[i];
            if (d>0 && (min_mm==0 || d<min_mm)) min_mm = d;
          }
          if (min_mm>0){
            if (best==0 || min_mm<best) best = min_mm;
          }
          break;
        }
      }
      if (millis() - start > perSampleTimeoutMs) break;
      delay(5);
    }
    delay(20);
  }
  return best; // 0 if none
}

// ---- HTTP: /cal_now?true_cm=XX  (compute new offset so calibrated == true) ----
void handleCalNow(){
  if (!server.hasArg("true_cm")) {
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"missing true_cm\"}");
    return;
  }
  int true_cm = server.arg("true_cm").toInt();
  if (true_cm <= 0 || true_cm > 400){
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"true_cm out of range\"}");
    return;
  }

  // Take a short burst of raw readings
  uint16_t raw_mm = sampleNearestRawMm(4, 250);
  if (raw_mm == 0){
    server.send(200, "application/json", "{\"ok\":false,\"err\":\"no reading; aim at a flat target\"}");
    return;
  }

  // Use current scale% (keep), compute an offset that makes: cal_mm = true_cm*10
  // cal_mm = raw_mm * (scale/100) + offset_cm*10  =>  offset_cm = (true_mm - raw_mm*scale/100)/10
  int32_t true_mm = (int32_t)true_cm * 10;
  int32_t scaled  = ((int32_t)raw_mm * (int32_t)g_set.cal_scale_pct) / 100;
  int32_t new_off_cm = (true_mm - scaled) / 10; // integer cm

  // Clamp to safe bounds
  if (new_off_cm < CAL_OFF_MIN) new_off_cm = CAL_OFF_MIN;
  if (new_off_cm > CAL_OFF_MAX) new_off_cm = CAL_OFF_MAX;

  g_set.cal_offset_cm = (int16_t)new_off_cm;
  prefs.putShort("cal_off_cm", g_set.cal_offset_cm);

  // Respond with details
  String j;
  j.reserve(128);
  j += F("{\"ok\":true,\"measured_cm\":");
  j += (raw_mm/10);
  j += F(",\"scale_pct\":");
  j += (int)g_set.cal_scale_pct;
  j += F(",\"new_offset_cm\":");
  j += g_set.cal_offset_cm;
  j += F("}");
  server.sendHeader("Cache-Control","no-store");
  server.send(200, "application/json", j);
}

void handleRoot(){ server.send_P(200, "text/html; charset=utf-8", INDEX_HTML); }

void handleFrame(){
  Frame snap = g_frame; // simple copy
  String json;
  if(snap.valid){
    json.reserve(1300);
    json += F("{\"ok\":true,\"nearest_cm\":");
    json += (snap.nearest_mm>0? (snap.nearest_mm/10) : 0);
    json += F(",\"direction\":\""); json += dirToText(snap.dir);
    json += F("\",\"ts\":"); json += (uint32_t) (snap.timestamp_ms);
    json += F(",\"speak_web\":"); json += (int)g_set.speak_web;
    json += F(",\"grid_mm\":[");
    for(int i=0;i<64;i++){ if(i) json+=','; json+= snap.grid[i]; }
    json += F("]}");
  } else {
    json = F("{\"ok\":false}");
  }
  server.sendHeader("Cache-Control","no-store");
  server.send(200, "application/json", json);
}

void notFound(){ server.send(404, "text/plain", "Not found"); }

// ---------------- Audio trigger (≤ range) ----------------
uint32_t lastPlayMs = 0;
uint16_t lastPlayCm = 9999;
Direction lastPlayDir = DIR_UNKNOWN;

void maybePlayVoice(uint16_t nearest_mm, int nearest_idx){
  if (nearest_idx < 0) return;
  if (nearest_mm == 0 || nearest_mm > g_maxRangeMm) return;

  uint16_t cm = nearest_mm / 10;
  Direction dir = dirFromColFW(nearest_idx % 8);

  uint32_t now = millis();
  bool crossed     = (lastPlayCm > g_set.range_cm && cm <= g_set.range_cm);
  bool dirChanged  = (dir != lastPlayDir);
  bool distChanged = (abs((int)cm - (int)lastPlayCm) >= (int)DELTA_CM_FOR_REPEAT);
  bool cooled      = (now - lastPlayMs >= COOLDOWN_MS);

  if (cm <= g_set.range_cm && (crossed || dirChanged || distChanged) && cooled) {
    uint8_t track = directionToTrackIndex(dir);
    if (track >= 1) {
      jqPlayIndex(track);
      lastPlayMs  = now;
      lastPlayCm  = cm;
      lastPlayDir = dir;
    }
  }
}

// ---------------- Init helpers ----------------
void tofInit(){
  if(LPN_PIN>=0){ pinMode(LPN_PIN,OUTPUT); digitalWrite(LPN_PIN,LOW); delay(5); digitalWrite(LPN_PIN,HIGH); delay(5); }
  Wire.begin(SDA_PIN,SCL_PIN);
  Wire.setClock(400000);

  if(tof.begin()!=0){
    Serial.println("[ToF] begin() failed");
    return;
  }
  tof.vl53l7cx_off(); delay(5);
  tof.vl53l7cx_init();
  tof.vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_8X8);
  tof.vl53l7cx_set_ranging_frequency_hz(max<uint8_t>(1, g_set.freq_hz));
  tof.vl53l7cx_start_ranging();
  Serial.printf("[ToF] Ranging @ %u Hz\n", g_set.freq_hz);
}

void mp3Init(){
  MP3.begin(9600, SERIAL_8N1, JQ_RX, JQ_TX);
  delay(200);
  jqSetVolume(22); // 0..30
  Serial.println("[JQ6500] UART ready");
}

void wifiInit(){
  WiFi.mode(WIFI_AP);
  WiFi.softAPsetHostname("smartcane");
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("[WiFi] AP %s, IP: %s\n", ok?"started":"FAILED", WiFi.softAPIP().toString().c_str());
  server.on("/", HTTP_GET, handleRoot);
  server.on("/frame", HTTP_GET, handleFrame);
  server.on("/settings", HTTP_GET, handleSettings);
  server.on("/set", HTTP_GET, handleSet);
  server.on("/cal_now", HTTP_GET, handleCalNow);   // calibrate endpoint
  server.onNotFound(notFound);
  server.begin();
  Serial.println("[HTTP] Listening on :80");
}

void loadSettings(){
  prefs.begin("smartcane", false);
  g_set.range_cm      = prefs.getUShort("range_cm",   20); // default 20 cm
  g_set.freq_hz       = prefs.getUChar ("freq_hz",     2); // default 2 Hz
  g_set.cal_offset_cm = prefs.getShort ("cal_off_cm",  0); // default 0 cm
  g_set.cal_scale_pct = prefs.getUChar ("cal_scl_pct",100); // default 100%
  g_set.speak_web     = prefs.getUChar ("speak_web",   0); // default off

  // clamp
  if (g_set.range_cm < RANGE_CM_MIN) g_set.range_cm = RANGE_CM_MIN;
  if (g_set.range_cm > RANGE_CM_MAX) g_set.range_cm = RANGE_CM_MAX;
  if (g_set.freq_hz  < FREQ_HZ_MIN ) g_set.freq_hz  = FREQ_HZ_MIN;
  if (g_set.freq_hz  > FREQ_HZ_MAX ) g_set.freq_hz  = FREQ_HZ_MAX;
  if (g_set.cal_offset_cm < CAL_OFF_MIN) g_set.cal_offset_cm = CAL_OFF_MIN;
  if (g_set.cal_offset_cm > CAL_OFF_MAX) g_set.cal_offset_cm = CAL_OFF_MAX;
  if (g_set.cal_scale_pct < CAL_SCL_MIN) g_set.cal_scale_pct = CAL_SCL_MIN;
  if (g_set.cal_scale_pct > CAL_SCL_MAX) g_set.cal_scale_pct = CAL_SCL_MAX;
  g_set.speak_web = g_set.speak_web ? 1 : 0;

  g_maxRangeMm = (uint16_t)(g_set.range_cm * 10);
  g_frameIntervalMs = max<uint32_t>(100, (1000 / max<uint8_t>(1, g_set.freq_hz)));
  Serial.printf("[Settings] range=%ucm, freq=%uHz, scale=%u%%, offset=%dcm, speak_web=%u\n",
                g_set.range_cm, g_set.freq_hz, g_set.cal_scale_pct, (int)g_set.cal_offset_cm, (unsigned)g_set.speak_web);
}

// ---------------- Setup / Loop ----------------
void setup(){
  Serial.begin(115200); delay(200);
  Serial.println("\n[SmartCane — Calibration + Web Speech] Boot");

  // Init cache
  memset((void*)g_frame.grid, 0, sizeof(g_frame.grid));
  g_frame.nearest_mm = 0; g_frame.nearest_idx = -1; g_frame.dir = DIR_UNKNOWN; g_frame.timestamp_ms = 0; g_frame.valid=false;

  loadSettings();
  tofInit();
  mp3Init();
  wifiInit();
}

void loop(){
  // Always prioritize serving clients
  server.handleClient();

  // Sensor cadence (based on current settings)
  static uint32_t lastFrameTick = 0;
  if (millis() - lastFrameTick >= g_frameIntervalMs){
    lastFrameTick = millis();

    static VL53L7CX_ResultsData res;
    uint8_t ready = 0;
    if (tof.vl53l7cx_check_data_ready(&ready) == 0 && ready){
      if (tof.vl53l7cx_get_ranging_data(&res) == 0){
        // Build new snapshot
        Frame tmp; tmp.valid = true; tmp.nearest_mm = 0; tmp.nearest_idx = -1; tmp.dir = DIR_UNKNOWN;
        uint16_t min_mm = 0xFFFF; int min_idx=-1;

        for (int i=0;i<64;i++){
          uint16_t d_raw = res.distance_mm[i]; // mm (raw)
          // --- Apply calibration: scale then offset (mm_cal = raw * s% + off_cm*10)
          int32_t mm_cal = (int32_t)((int32_t)d_raw * (int32_t)g_set.cal_scale_pct) / 100;
          mm_cal += ((int32_t)g_set.cal_offset_cm) * 10;
          if (mm_cal < 0) mm_cal = 0;
          uint16_t d = (uint16_t)mm_cal;

          // Clip to selected range for visualization/logic
          uint16_t clipped = (d>0 && d<=g_maxRangeMm) ? d : 0;
          tmp.grid[i] = clipped;

          if (clipped>0 && clipped < min_mm){ min_mm = clipped; min_idx = i; }
        }

        if (min_idx >= 0){
          tmp.nearest_mm = min_mm;
          tmp.nearest_idx = min_idx;
          tmp.dir = dirFromColFW(min_idx % 8);
        }
        tmp.timestamp_ms = millis();

        // Publish snapshot
        g_frame = tmp;

        // Drive on-device audio from fresh frame
        if (min_idx >= 0) maybePlayVoice(min_mm, min_idx);
      }
    }
  }

  // Let Wi-Fi run
  delay(1);
}
