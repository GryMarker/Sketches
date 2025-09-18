

// AP: SSID "Glove-AP", PASS "glove1234", URL http://192.168.4.1/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>

// ---------- SoftAP ----------
const char* AP_SSID = "Glove-AP";
const char* AP_PASS = "glove1234";
IPAddress apIP(192,168,4,1), apGW(192,168,4,1), apMask(255,255,255,0);
DNSServer dns;

// ---------- Sensors ----------
const int SENS_PINS[4] = {32, 33, 34, 35};       // Index, Middle, Ring, Pinky (device order)
const char* NAME[4]    = {"Index","Middle","Ring","Pinky"};


const int UI_ORDER[4]  = {0, 3, 1, 2};
const int LED_PIN      = 2;

// Spoken phrases per finger (device order). Can be edited in UI and saved in NVS.
String PHRASE[4] = {"Yes", "Help", "Water please", "Pain"};

// --- Stable detection tuning ---
const uint32_t CALIB_MS         = 2000;  // baseline & noise (hand open)
const uint32_t READ_PERIOD_MS   = 1000;  // decisions once per second
const int      SAMPLES_PER_TICK = 40;    // 40 reads (~200 ms burst)
const int      SAMPLE_DELAY_MS  = 5;
const float    EMA_ALPHA        = 0.25f; 
const uint32_t HOLD_MS          = 500;   // must hold 0.5 s
const uint32_t RETRIGGER_GAP_MS = 1200;  // extra spacing

// Auto-capture tuning: when capturing a bent finger, we take this fraction of max|Δ| as new trigger floor.
const float CAPTURE_FRACTION    = 0.35f; 

// Runtime state
float baseVal[4], filtVal[4], sigma[4];
int   thrOn[4], thrOff[4];      
int   thrFloor[4] = {40,40,40,40}; 
bool  bent[4] = {0};
unsigned long bentSince[4] = {0,0,0,0};
unsigned long lastTrigAt[4] = {0,0,0,0};
bool  needRelease[4] = {false,false,false,false};  // must open before retrigger

String   lastPhrase = "—";
uint32_t lastEventId = 0;

// ---------- NVS ----------
Preferences prefs;

// ---------- Web ----------
WebServer server(80);

const char* HTML = R"HTML(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Assist Glove</title>
<style>
:root{--ok:#14b814;}
html,body{margin:0;background:#000;color:#fff;font-family:system-ui,Segoe UI,Roboto,Arial}
.wrap{max-width:980px;margin:0 auto;padding:20px}
h1{font-size:1.6rem;margin:0 0 12px}
h2{font-size:1.2rem;margin:12px 0 6px}
.row{display:flex;gap:10px;flex-wrap:wrap;margin:10px 0}
.btn{padding:10px 12px;border:1px solid #444;background:#1d1d1d;border-radius:10px;color:#fff;font-weight:600;cursor:pointer}
.badge{flex:1 1 220px;display:flex;justify-content:space-between;align-items:center;
  padding:14px 16px;border:1px solid #333;border-radius:14px;background:#111;font-size:1.05rem}
.dot{width:12px;height:12px;border-radius:50%;background:#444;margin-left:10px}
.on .dot{background:var(--ok)}
.small{color:#bbb;font-size:0.95rem}
.alert{position:sticky;top:0;z-index:5;background:#1f2937;border:1px solid #334155;border-left:8px solid var(--ok);
  padding:12px 14px;border-radius:10px;margin:8px 0;display:none}
.alert.show{display:block;animation:pop .2s ease}
@keyframes pop{from{transform:scale(.98);opacity:.7}to{transform:scale(1);opacity:1}}
.card{background:#0d0d0d;border:1px solid #222;border-radius:14px;padding:12px}
ul{list-style:none;padding:0;margin:0}
li{padding:10px 8px;border-bottom:1px solid #222;display:flex;justify-content:space-between;gap:10px}
.time{color:#9aa;font-variant-numeric:tabular-nums}
.grid{display:grid;grid-template-columns:repeat(2,minmax(260px,1fr));gap:12px}
.input{display:flex;flex-direction:column;gap:6px;background:#101010;border:1px solid #222;border-radius:10px;padding:10px}
.input label{font-size:0.95rem;color:#ddd}
.input input[type=text]{padding:8px;border-radius:8px;border:1px solid #333;background:#0a0a0a;color:#fff}
.slider-row{display:flex;align-items:center;gap:10px}
.slider-row input[type=range]{flex:1}
.kv{display:flex;justify-content:space-between}
@media (max-width:720px){.grid{grid-template-columns:1fr}}
</style></head><body>
<div class="wrap">
  <h1>Assist Glove</h1>
  <div class="row">
    <button class="btn" id="voiceBtn">🔊 Enable Voice</button>
    <span class="small" id="voiceStatus"></span>
    <button class="btn" id="recalBtn">🧭 Recalibrate Baseline</button>
    <span class="small" id="recalStatus"></span>
  </div>

  <div id="alert" class="alert"><b id="alertText"></b></div>

  <div class="row" id="badges"></div>

  <div class="card" style="margin-top:12px">
    <div class="row" style="justify-content:space-between;align-items:center">
      <h2>History</h2>
      <button class="btn" id="clearBtn">Clear</button>
    </div>
    <ul id="history"></ul>
  </div>

  <div class="card" style="margin-top:12px">
    <h2>Settings</h2>
    <div class="small">Edit phrases, set trigger level, or auto-capture while bending a finger.</div>
    <div class="grid" id="cfgGrid"></div>
    <div class="row" style="justify-content:flex-end">
      <button class="btn" id="saveBtn">💾 Save</button>
      <button class="btn" id="defaultBtn">↺ Defaults</button>
    </div>
  </div>
</div>

<script>
/* ======== UI CONSTANTS ======== */
const uiNames=['Index','Pinky','Middle','Ring'];   // badge order (server uses same)
const REPEAT_COUNT = 3;
const REPEAT_GAP_MS = 250;

/* ======== STATE ======== */
let voiceOK=false, lastEvent=0, audioCtx=null;
let cfg=null; // {phr:[...], thr:[...]} in device order [Index,Middle,Ring,Pinky]
const UI_ORDER = [0,3,1,2]; // server's mapping for badges (Index,Pinky,Middle,Ring)
const INV_UI = (()=>{
  // map from UI order idx -> device index
  const inv = []; for(let k=0;k<4;k++){ inv[k]=UI_ORDER[k]; } return inv;
})();

/* ======== VOICE ======== */
document.getElementById('voiceBtn').addEventListener('click', async ()=>{
  try{
    voiceOK=true;
    if(!audioCtx){ audioCtx = new (window.AudioContext||window.webkitAudioContext)(); await audioCtx.resume(); }
    speak('Voice enabled'); document.getElementById('voiceStatus').textContent=' Voice ON';
    setTimeout(()=>document.getElementById('voiceStatus').textContent='',1500);
  }catch(e){}
});
function speak(text){
  if(!voiceOK) return;
  try{ const u=new SpeechSynthesisUtterance(text); u.lang='en-US'; speechSynthesis.cancel(); speechSynthesis.speak(u); }catch(e){}
}
function speakRepeat(text, times=REPEAT_COUNT){
  if(!voiceOK) return;
  try{ speechSynthesis.cancel(); }catch(e){}
  let count=0;
  const speakOnce=()=>{
    const u=new SpeechSynthesisUtterance(text); u.lang='en-US';
    u.onend=()=>{ count++; if(count<times) setTimeout(speakOnce, REPEAT_GAP_MS); };
    try{ speechSynthesis.speak(u); }catch(e){}
    beep(); vibrate();
  }; speakOnce();
}
function beep(ms=160){
  if(!voiceOK||!audioCtx) return;
  const o=audioCtx.createOscillator(), g=audioCtx.createGain();
  o.type='sine'; o.frequency.value=880;
  g.gain.setValueAtTime(0.001,audioCtx.currentTime);
  g.gain.exponentialRampToValueAtTime(0.3,audioCtx.currentTime+0.01);
  o.connect(g); g.connect(audioCtx.destination);
  o.start(); setTimeout(()=>{g.gain.exponentialRampToValueAtTime(0.001,audioCtx.currentTime+0.01); o.stop();}, ms);
}
function vibrate(){ if(navigator.vibrate) navigator.vibrate([120,80,120]); }

/* ======== BADGES, ALERT, HISTORY ======== */
function renderBadges(bent){
  let html='';
  for(let i=0;i<4;i++){
    html+=`<div class="badge ${bent[i]?'on':''}">
      <div>${uiNames[i]}</div>
      <div style="display:flex;align-items:center;gap:10px">
        <div>${bent[i]?'BENT':'OPEN'}</div><div class="dot"></div>
      </div></div>`;
  }
  document.getElementById('badges').innerHTML=html;
}
function showAlert(text){
  const a=document.getElementById('alert');
  document.getElementById('alertText').textContent=text;
  a.classList.add('show'); setTimeout(()=>a.classList.remove('show'), 2500);
}
function addHistory(text){
  const ul=document.getElementById('history');
  const li=document.createElement('li');
  const t=new Date();
  li.innerHTML=`<span>${text}</span><span class="time">${t.toLocaleTimeString([], {hour:'2-digit',minute:'2-digit',second:'2-digit'})}</span>`;
  ul.prepend(li); while(ul.children.length>20) ul.removeChild(ul.lastChild);
}
document.getElementById('clearBtn').addEventListener('click', ()=>{ document.getElementById('history').innerHTML=''; });

/* ======== SETTINGS UI ======== */
function renderCfg(){
  if(!cfg) return;
  const namesDev=['Index','Middle','Ring','Pinky']; // device order for settings
  let html='';
  for(let i=0;i<4;i++){
    const phr = cfg.phr[i] ?? '';
    const thr = cfg.thr[i] ?? 40;
    html+=`<div class="input">
      <div class="kv"><label><b>${namesDev[i]}</b> phrase</label>
        <button class="btn" onclick="testSpeak(${i})">Say</button></div>
      <input type="text" id="phr${i}" value="${phr.replaceAll('"','&quot;')}" maxlength="64" />
      <label>Trigger level (counts)</label>
      <div class="slider-row">
        <input type="range" id="thr${i}" min="20" max="400" step="2" value="${thr}" oninput="document.getElementById('thrv${i}').textContent=this.value">
        <span id="thrv${i}">${thr}</span>
      </div>
      <div class="row">
        <button class="btn" onclick="capture(${i})">📏 Capture while bent</button>
        <span class="small" id="cap${i}"></span>
      </div>
    </div>`;
  }
  document.getElementById('cfgGrid').innerHTML = html;
}
function testSpeak(i){
  if(!cfg) return;
  const v = document.getElementById('phr'+i).value.trim() || '(empty)';
  speak(v);
}
async function capture(i){
  const span = document.getElementById('cap'+i);
  span.textContent=' Capturing... bend and hold';
  try{
    const r = await fetch('/cap?i='+i, {method:'POST'});
    const j = await r.json();
    if(j.ok){
      document.getElementById('thr'+i).value = j.newThr;
      document.getElementById('thrv'+i).textContent = j.newThr;
      span.textContent = ' set to '+j.newThr+' (peak '+j.peak+')';
    }else{ span.textContent=' failed'; }
  }catch(e){ span.textContent=' failed'; }
  setTimeout(()=>span.textContent='', 2000);
}
document.getElementById('saveBtn').addEventListener('click', async ()=>{
  const params = new URLSearchParams();
  for(let i=0;i<4;i++){
    params.append('phr'+i, document.getElementById('phr'+i).value.trim());
    params.append('thr'+i, document.getElementById('thr'+i).value);
  }
  const st = document.getElementById('voiceStatus');
  st.textContent=' Saving...';
  try{
    const r = await fetch('/set', {method:'POST', body: params});
    st.textContent = r.ok ? ' Saved' : ' Save failed';
    setTimeout(()=>st.textContent='',1500);
    await fetchCfg();
  }catch(e){ st.textContent=' Save failed'; setTimeout(()=>st.textContent='',1500); }
});
document.getElementById('defaultBtn').addEventListener('click', async ()=>{
  const r=await fetch('/defaults', {method:'POST'});
  if(r.ok) await fetchCfg();
});
document.getElementById('recalBtn').addEventListener('click', async ()=>{
  const el=document.getElementById('recalStatus');
  el.textContent=' Recalibrating... keep hand open';
  try{
    const r=await fetch('/recal', {method:'POST'});
    el.textContent = r.ok ? ' Done' : ' Failed';
  }catch(e){ el.textContent=' Failed'; }
  setTimeout(()=>el.textContent='',1800);
});

/* ======== POLLING ======== */
async function refresh(){
  try{
    const r = await fetch('/state?ts='+Date.now(), {cache:'no-store'});
    const j = await r.json();
    renderBadges(j.bent);
    if(j.event && j.event!==lastEvent){
      lastEvent=j.event;
      if(j.last && j.last!=='—'){
        showAlert(j.last);
        if(voiceOK){ speakRepeat(j.last); }
        addHistory(j.last);
      }
    }
  }catch(e){}
  setTimeout(refresh, 800);
}
async function fetchCfg(){
  try{
    const r = await fetch('/cfg?ts='+Date.now(), {cache:'no-store'});
    cfg = await r.json();
    renderCfg();
  }catch(e){}
}
refresh(); fetchCfg();
</script>
</body></html>
)HTML";



// ------- helpers -------
bool tickEvery(uint32_t ms){ static uint32_t last=0; uint32_t now=millis(); if(now-last>=ms){ last=now; return true; } return false; }

void recomputeThresholds(){
  for(int i=0;i<4;i++){
    int noiseBased = (int)(8.0f * sigma[i]);
    thrOn[i]  = max(thrFloor[i], max(40, noiseBased)); 
    thrOff[i] = max(24, (int)(0.6f * thrOn[i]));
  }
}

void loadConfig(){
  prefs.begin("glove", true); // read-only
  for(int i=0;i<4;i++){
    String keyP = String("phr")+i;
    String keyT = String("thr")+i;
    String vP = prefs.getString(keyP.c_str(), "");
    int    vT = prefs.getInt(keyT.c_str(), -1);
    if(vP.length()>0) PHRASE[i] = vP;
    if(vT >= 20 && vT <= 800) thrFloor[i] = vT;
  }
  prefs.end();
}

void saveConfig(){
  prefs.begin("glove", false); // write
  for(int i=0;i<4;i++){
    prefs.putString((String("phr")+i).c_str(), PHRASE[i]);
    prefs.putInt   ((String("thr")+i).c_str(), thrFloor[i]);
  }
  prefs.end();
}

String jsonCfg(){ // device order
  String j = "{\"phr\":[";
  for(int i=0;i<4;i++){ j += "\""+PHRASE[i]+"\""; if(i<3) j+=','; }
  j += "],\"thr\":[";
  for(int i=0;i<4;i++){ j += String(thrFloor[i]); if(i<3) j+=','; }
  j += "]}";
  return j;
}

void recalibrate(){

  double sum[4]={0}, sum2[4]={0}; uint32_t n=0, t0=millis();
  while(millis()-t0<CALIB_MS){
    for(int i=0;i<4;i++){ uint16_t r=analogRead(SENS_PINS[i]); sum[i]+=r; sum2[i]+=(double)r*(double)r; }
    n++; delay(3);
  }
  if(!n) n=1;
  for(int i=0;i<4;i++){
    double mean=sum[i]/(double)n, var=max(0.0,(sum2[i]/(double)n)-mean*mean);
    baseVal[i]=mean; sigma[i]=sqrt(var); filtVal[i]=baseVal[i];
    bent[i]=false; bentSince[i]=0; needRelease[i]=false;
  }
  recomputeThresholds();
}

void setup(){
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);


  for(int i=0;i<4;i++) analogSetPinAttenuation(SENS_PINS[i], ADC_11db);


  loadConfig();

  // Initial calibration
  recalibrate();

  // SoftAP + captive DNS
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apGW, apMask);
  WiFi.softAP(AP_SSID, AP_PASS, 6 /*channel*/, false /*hidden*/, 8 /*max clients*/);
  dns.start(53, "*", apIP);

  // --- Web routes ---
  server.on("/", [](){ server.send(200,"text/html",HTML); });

  // Live state for badges & TTS
  server.on("/state", [](){
    String j = "{\"bent\":[";
    for(int k=0;k<4;k++){ int i=UI_ORDER[k]; j += (bent[i]?"true":"false"); if(k<3) j+=','; }
    j += "],\"last\":\""+lastPhrase+"\",\"event\":"+String(lastEventId)+"}";
    server.send(200,"application/json", j);
  });

  // Return current config (device order)
  server.on("/cfg", [](){
    server.sendHeader("Cache-Control","no-store");
    server.send(200,"application/json", jsonCfg());
  });

  // Save config from form (device order)
  server.on("/set", HTTP_POST, [](){
    for(int i=0;i<4;i++){
      String pKey = "phr"+String(i);
      String tKey = "thr"+String(i);
      if(server.hasArg(pKey)) PHRASE[i] = server.arg(pKey);
      if(server.hasArg(tKey)) {
        int v = server.arg(tKey).toInt();
        if(v>=20 && v<=800) thrFloor[i] = v;
      }
    }
    saveConfig();
    recomputeThresholds();
    server.send(200,"text/plain","OK");
  });

  // Restore defaults
  server.on("/defaults", HTTP_POST, [](){
    PHRASE[0]="Yes"; PHRASE[1]="Help"; PHRASE[2]="Water please"; PHRASE[3]="Pain";
    thrFloor[0]=thrFloor[1]=thrFloor[2]=thrFloor[3]=40;
    saveConfig(); recomputeThresholds();
    server.send(200,"text/plain","OK");
  });

  // Recalibrate baselines/sigma (blocking ~2s)
  server.on("/recal", HTTP_POST, [](){
    recalibrate();
    server.send(200,"text/plain","OK");
  });

  // Capture bent peak for a finger -> set new thr floor as CAPTURE_FRACTION * peak
  server.on("/cap", HTTP_POST, [](){
    if(!server.hasArg("i")){ server.send(400,"application/json","{\"ok\":false}"); return; }
    int i = server.arg("i").toInt();
    if(i<0 || i>3){ server.send(400,"application/json","{\"ok\":false}"); return; }

    // Sample for ~800 ms quickly and find peak |Δ|
    uint32_t t0 = millis();
    float peak = 0;
    while(millis()-t0 < 800){
      uint16_t r = analogRead(SENS_PINS[i]);
      float d = fabsf((float)r - baseVal[i]);
      if(d > peak) peak = d;
      delay(2);
    }
    int newThr = max(20, (int)(CAPTURE_FRACTION * peak));
    thrFloor[i] = newThr;
    saveConfig();
    recomputeThresholds();

    String j = String("{\"ok\":true,\"peak\":") + String((int)peak) + ",\"newThr\":" + String(newThr) + "}";
    server.send(200,"application/json", j);
  });

  server.begin();
}

void loop(){
  dns.processNextRequest();
  server.handleClient();

  if(!tickEvery(READ_PERIOD_MS)) return;

  // --- Quiet burst sampling (~200 ms) ---
  float avg[4]={0};
  for(int s=0;s<SAMPLES_PER_TICK;s++){
    for(int i=0;i<4;i++) avg[i]+=analogRead(SENS_PINS[i]);
    delay(SAMPLE_DELAY_MS);
  }
  for(int i=0;i<4;i++) avg[i]/=(float)SAMPLES_PER_TICK;

  // --- Decide once per second (stable) ---
  uint32_t now = millis();
  for(int i=0;i<4;i++){
    filtVal[i] = EMA_ALPHA*avg[i] + (1.0f-EMA_ALPHA)*filtVal[i];
    float dAbs = fabsf(filtVal[i] - baseVal[i]);

    bool prevBent = bent[i];
    if(!bent[i] && dAbs > thrOn[i]) {
      bent[i] = true;
      if(!bentSince[i]) bentSince[i] = now;
    } else if(bent[i] && dAbs < thrOff[i]) {
      bent[i] = false;
      bentSince[i] = 0;
      needRelease[i] = false; // open detected -> allow next trigger
    }

    if(bent[i] && !prevBent) bentSince[i] = now; // just entered bent

    bool holdOK = bentSince[i] && (now - bentSince[i] >= HOLD_MS);
    bool spacingOK = (now - lastTrigAt[i] >= RETRIGGER_GAP_MS);

    if(bent[i] && holdOK && !needRelease[i] && spacingOK){
      digitalWrite(LED_PIN, HIGH);
      lastPhrase = PHRASE[i];
      lastEventId++;
      digitalWrite(LED_PIN, LOW);
      lastTrigAt[i] = now;
      needRelease[i] = true;  // require opening before next trigger
    }
  }
}
