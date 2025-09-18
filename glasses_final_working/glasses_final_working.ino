/*
  Board: Seeed XIAO ESP32S3 Sense
  Tools → PSRAM: OPI PSRAM
*/

#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <Wire.h>

#include <SparkFun_VL53L5CX_Library.h>   =
SparkFun_VL53L5CX tof;
VL53L5CX_ResultsData tofData;
bool tof_ok = false;

#define TOF_LPN_PIN   3          


#define TOF_ROTATION  90         

// ====== Camera pins (XIAO ESP32S3 Sense, OV3660) ======
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   10
#define SIOD_GPIO_NUM   40
#define SIOC_GPIO_NUM   39
#define Y9_GPIO_NUM     48
#define Y8_GPIO_NUM     11
#define Y7_GPIO_NUM     12
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     16
#define Y4_GPIO_NUM     18
#define Y3_GPIO_NUM     17
#define Y2_GPIO_NUM     15
#define VSYNC_GPIO_NUM  38
#define HREF_GPIO_NUM   47
#define PCLK_GPIO_NUM   13

// ====== Wi-Fi (Change to your Wifi) ======
static const char* WIFI_SSID = "TECNO CAMON 30 5G";
static const char* WIFI_PASS = "HAPPY KA BA HAHA";

httpd_handle_t httpd = nullptr;

// ---------- Small /plain test page ----------
static const char PLAIN_HTML[] PROGMEM =
"<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'/>"
"<title>Plain JPEG</title><body style='margin:0;background:#000'>"
"<img id='v' style='width:100%;display:block'/>"
"<script>const v=document.getElementById('v');function t(){v.src='/jpg?ts='+Date.now()}setInterval(t,500);t()</script>";

// ---------- HTTP handlers ----------
esp_err_t jpg_handler(httpd_req_t* req){
  camera_fb_t* fb = esp_camera_fb_get();
  if(!fb) return httpd_resp_send_500(req);
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  httpd_resp_set_hdr(req, "Pragma", "no-cache");
  httpd_resp_set_hdr(req, "Expires", "0");
  httpd_resp_set_hdr(req, "Connection", "close");
  esp_err_t r = httpd_resp_send(req, (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return r;
}

esp_err_t plain_handler(httpd_req_t* req){
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, PLAIN_HTML, HTTPD_RESP_USE_STRLEN);
}

esp_err_t health_handler(httpd_req_t* req){
  char buf[160];
  int n = snprintf(buf, sizeof(buf),
    "{\"heap\":%u,\"psram\":%u,\"uptime_ms\":%u}",
    ESP.getFreeHeap(), ESP.getFreePsram(), (unsigned)millis());
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, buf, n);
}

// rotate 8x8 by N deg CW; writes JSON into 'json'
void build_rotated_grid_json(String& json){
  if(!tof_ok || !tof.isDataReady()){
    json = "[]"; return;
  }
  if(!tof.getRangingData(&tofData)){        // returns true on success
    json = "[]"; return;
  }

  json.reserve(64 * 6);
  json = "[";
  for(int y=0; y<8; ++y){
    for(int x=0; x<8; ++x){
      int idx_old;
      #if TOF_ROTATION == 0
        idx_old = y*8 + x;
      #elif TOF_ROTATION == 90
        // new(y,x) = old(7 - x, y)
        idx_old = (7 - x)*8 + y;
      #elif TOF_ROTATION == 180
        // new(y,x) = old(7 - y, 7 - x)
        idx_old = (7 - y)*8 + (7 - x);
      #elif TOF_ROTATION == 270
        // new(y,x) = old(x, 7 - y)
        idx_old = x*8 + (7 - y);
      #else
        idx_old = y*8 + x;
      #endif

      if(y || x) json += ",";
      json += (int)tofData.distance_mm[idx_old];
    }
  }
  json += "]";
}

esp_err_t grid_handler(httpd_req_t* req){
  httpd_resp_set_type(req, "application/json");
  String j; build_rotated_grid_json(j);
  return httpd_resp_send(req, j.c_str(), j.length());
}

void start_httpd(){
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  cfg.stack_size  = 6144;
  cfg.core_id     = tskNO_AFFINITY;
  if(httpd_start(&httpd, &cfg) == ESP_OK){
    httpd_uri_t u_root  = { .uri="/",      .method=HTTP_GET, .handler=plain_handler, .user_ctx=NULL };
    httpd_uri_t u_plain = { .uri="/plain", .method=HTTP_GET, .handler=plain_handler, .user_ctx=NULL };
    httpd_uri_t u_jpg   = { .uri="/jpg",   .method=HTTP_GET, .handler=jpg_handler,   .user_ctx=NULL };
    httpd_uri_t u_grid  = { .uri="/grid",  .method=HTTP_GET, .handler=grid_handler,  .user_ctx=NULL };
    httpd_uri_t u_health= { .uri="/health",.method=HTTP_GET, .handler=health_handler,.user_ctx=NULL };
    httpd_register_uri_handler(httpd, &u_root);
    httpd_register_uri_handler(httpd, &u_plain);
    httpd_register_uri_handler(httpd, &u_jpg);
    httpd_register_uri_handler(httpd, &u_grid);
    httpd_register_uri_handler(httpd, &u_health);
  }
}

// ---------- Camera ----------
bool startCamera(){
  esp_log_level_set("camera", ESP_LOG_ERROR);
  esp_log_level_set("cam_hal", ESP_LOG_ERROR);
  esp_log_level_set("sccb",   ESP_LOG_ERROR);

  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;

  c.xclk_freq_hz = 10000000;      // conservative; bump to 20000000 if stable
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size   = FRAMESIZE_QVGA; 
  c.jpeg_quality = 22;             // higher number -> smaller file
  c.fb_count     = 2;
  c.grab_mode    = CAMERA_GRAB_LATEST;
  c.fb_location  = CAMERA_FB_IN_DRAM;

  esp_err_t err = esp_camera_init(&c);
  if(err != ESP_OK){
    Serial.printf("Camera DRAM init failed: 0x%x, retry PSRAM\n", err);
    c.fb_location = CAMERA_FB_IN_PSRAM;
    err = esp_camera_init(&c);
    if(err != ESP_OK){
      Serial.printf("Camera init failed: 0x%x\n", err);
      return false;
    }
  }
  return true;
}

// ---------- ToF ----------
bool startToF(){
  
  if (TOF_LPN_PIN >= 0) {
    pinMode(TOF_LPN_PIN, OUTPUT);
    digitalWrite(TOF_LPN_PIN, LOW);   
    delay(5);
    digitalWrite(TOF_LPN_PIN, HIGH); 
    delay(20);
  }

  // IMPORTANT: XIAO Grove I2C pins are SDA=GPIO5 (D4), SCL=GPIO6 (D5)
  Wire.begin(5, 6);
  Wire.setClock(400000);           

  if(!tof.begin()){
    Serial.println("VL53L5/7CX: begin failed (check wiring/LPn)");
    return false;
  }
  tof.setResolution(8*8);         
  tof.setRangingFrequency(15);    
  if(!tof.startRanging()){
    Serial.println("VL53L5/7CX: startRanging failed");
    return false;
  }
  return true;
}

// ---------- Wi-Fi ----------
void startWiFi_STA(){
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to '%s'", WIFI_SSID);
  uint32_t t0 = millis();
  while(WiFi.status() != WL_CONNECTED && millis()-t0 < 15000){
    delay(300); Serial.print(".");
  }
  if(WiFi.status() == WL_CONNECTED){
    Serial.printf("\nSTA IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nSTA failed (check SSID/PASS and 2.4 GHz).");
  }
}

void setup(){
  Serial.begin(115200); delay(300);
  startWiFi_STA();
  if(!startCamera()) Serial.println("Camera failed to start"); else Serial.println("Camera OK");
  tof_ok = startToF();
  Serial.printf("ToF: %s\n", tof_ok ? "OK" : "FAIL");
  start_httpd();
  Serial.printf("Free heap: %u  Free PSRAM: %u\n", ESP.getFreeHeap(), ESP.getFreePsram());
}

void loop(){ /* http server + ranging run in background */ }
