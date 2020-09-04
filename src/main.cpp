#include "Arduino.h"
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "esp_camera.h"
#include "ArduinoNvs.h"
#include "esp_timer.h"
#include "esp32-hal-log.h"
 
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define led_pin           12

static const uint64_t uS_TO_H_FACTOR = 3600000000; /* Conversion factor for micro seconds to hours */ 
uint64_t TIME_TO_SLEEP = 0; /* Time ESP32 will go to sleep */
bool shutdown = false;

bool toggle = true;
bool wf_connected = false;
int64_t prev;
long rssi = 0;
int count = 0;

int64_t fr_start;
int64_t fr_end;
uint32_t cpt_time;
size_t fb_len = 0;
bool ready = false;
 
const char* ssid = "coldspot";
const char* password = "ponbun324";
 
AsyncWebServer server(80);
WiFiServer serialServer(8080);

WiFiClient client;
 
bool initCamera(){
   
  camera_config_t config;
   
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  config.frame_size = FRAMESIZE_UXGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;
    
  esp_err_t result = esp_camera_init(&config);
   
  if (result != ESP_OK) {
    return false;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_brightness(s, 0);     // -2 to 2
  s->set_contrast(s, 0);       // -2 to 2
  s->set_saturation(s, 0);     // -2 to 2
  s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
  s->set_aec2(s, 0);           // 0 = disable , 1 = enable
  s->set_ae_level(s, 0);       // -2 to 2
  s->set_aec_value(s, 300);    // 0 to 1200
  s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
  s->set_agc_gain(s, 0);       // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  s->set_bpc(s, 0);            // 0 = disable , 1 = enable
  s->set_wpc(s, 1);            // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  s->set_lenc(s, 1);           // 0 = disable , 1 = enable
  s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable

  return true;
}

static void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
  if (event == SYSTEM_EVENT_STA_DISCONNECTED){
    log_d("WiFi Disconnected");
    wf_connected = false;
    if (info.disconnected.reason == 6) {
      log_e("NOT_AUTHED reconnect");
      WiFi.reconnect();
    }
  }else if(event == SYSTEM_EVENT_STA_GOT_IP){
    log_d("WiFi Connected");
    wf_connected = true;
  }
}
 
void setup() {

  NVS.begin();
  
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH);

  log_d("Total heap: %d", ESP.getHeapSize());
  log_d("Free heap: %d", ESP.getFreeHeap());
  log_d("Total PSRAM: %d", ESP.getPsramSize());
  log_d("Free PSRAM: %d", ESP.getFreePsram());

  String stst = NVS.getString("st");
  log_d("NVS: stst %s", stst.c_str());
 
  if(!initCamera()){
     
    log_e("Failed to initialize camera...");
    return;  
  }
   
  WiFi.begin(ssid, password);
  WiFi.onEvent(WiFiEvent);
  WiFi.setAutoReconnect(true);
 
  while (WiFi.status() != WL_CONNECTED) {
    log_i("Connecting to WiFi..");
    delay(1000);
  }
 
  server.on("/picture", HTTP_GET, [](AsyncWebServerRequest * request) {
    
    digitalWrite(led_pin, HIGH);
    camera_fb_t * fb = NULL;

    fr_start = esp_timer_get_time();

    fb = esp_camera_fb_get();
    fb_len = fb->len;
    
    if (!fb) {
        log_e("Camera capture failed");
    }

    fr_end = esp_timer_get_time();
    cpt_time = (uint32_t)((fr_end - fr_start)/1000);

    request->send_P(200, "image/jpeg", (const uint8_t *)fb->buf, fb->len);

    log_d("Img size: %u B; capture time: %u ms; RSSI: %d dBm", (uint32_t)(fb_len), cpt_time, WiFi.RSSI());
    ready = true;
    digitalWrite(led_pin, LOW);
    esp_camera_fb_return(fb);
  });

  server.on("/shutdown", HTTP_GET, [](AsyncWebServerRequest * request){
    if(request->hasHeader("time")){
      AsyncWebHeader* h = request->getHeader("time");
      TIME_TO_SLEEP = atol(h->value().c_str());
      shutdown = true;
      request->send(200, "text/plain", "shutdown initiate");
    }else{
      request->send(400, "text/plain", "no header");
    }
  });
 
  server.begin();
  serialServer.begin();
  client = serialServer.available();
  prev = esp_timer_get_time();
}
 
void loop(){

  if (wf_connected){
    if(esp_timer_get_time() - prev > 1500000){
      toggle = !toggle;
      digitalWrite(led_pin, toggle);
      prev = esp_timer_get_time();
      log_i("RSSI : %d dBm", WiFi.RSSI());
    }
  }else{
    digitalWrite(led_pin, HIGH);
    log_w("WiFi disconnect");
  }

  if (client) {
    if(client.connected()) {
      if (ready){
        client.print("Img size: ");
        client.print((uint32_t)(fb_len));
        client.print(" B; capture time: ");
        client.print(cpt_time);
        client.print(" ms; RSSI: ");
        client.print(WiFi.RSSI());
        client.println(" dBm");
        ready = false;
      }
    }else{
      client.stop();
      log_w("Client disconnected");
    }  
  }else{
    client = serialServer.available();
  }

  if (shutdown){
    log_w("Initiate shutdown timer: %d h", TIME_TO_SLEEP);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_H_FACTOR);
    esp_deep_sleep_start();
  }

}