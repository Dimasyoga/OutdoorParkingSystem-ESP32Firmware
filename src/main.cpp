#include "Arduino.h"
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "esp_camera.h"
#include "ArduinoNvs.h"
#include "esp_timer.h"
#include "esp32-hal-log.h"
#include "SPIFFS.h"
 
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
#define reset_pin          2
#define uS_TO_H_FACTOR 3600000000
#define AP_wait 120000000

// static const uint64_t uS_TO_H_FACTOR = 3600000000; /* Conversion factor for micro seconds to hours */ 

uint64_t TIME_TO_SLEEP = 0; /* Time ESP32 will go to sleep */
bool shutdown = false;
RTC_DATA_ATTR bool force_APMode = false;

bool toggle = true;
bool wf_connected = false;
int64_t prev;
int64_t timer;
long rssi = 0;
bool STA_Mode = false;
bool STA_connect = false;

int64_t fr_start;
int64_t fr_end;
uint32_t cpt_time;
size_t fb_len = 0;
bool ready = false;

String device_name = "\0";
String Ap_pass = "\0";
String ssid = "\0";
String password = "\0";
 
AsyncWebServer server(80);
WiFiServer serialServer(8080);
WiFiClient client;

// Set your Static IP address
IPAddress local_IP(192, 168, 0, 1);
// Set your Gateway IP address
IPAddress gateway(192, 168, 0, 1);

IPAddress subnet(255, 255, 255, 0);

IPAddress IP;

IPAddress static_IP;
String static_ip_s;
IPAddress static_gateway;
String static_gateway_s;
IPAddress static_subnet;
String static_subnet_s;

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
  config.xclk_freq_hz = 16500000;
  config.pixel_format = PIXFORMAT_JPEG; 
  config.frame_size = FRAMESIZE_UXGA;
  config.jpeg_quality = 4;
  config.fb_count = 1;
    
  esp_err_t result = esp_camera_init(&config);
   
  if (result != ESP_OK) {
    return false;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_brightness(s, 0);     // -2 to 2
  s->set_contrast(s, 2);       // -2 to 2
  s->set_saturation(s, 0);     // -2 to 2
  s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
  s->set_aec2(s, 1);           // 0 = disable , 1 = enable
  s->set_ae_level(s, 0);       // -2 to 2
  // s->set_aec_value(s, 300);    // 0 to 1200
  s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
  // s->set_agc_gain(s, 0);       // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  s->set_bpc(s, 1);            // 0 = disable , 1 = enable
  s->set_wpc(s, 1);            // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  s->set_lenc(s, 1);           // 0 = disable , 1 = enable
  s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable

  return true;
}

void handleCapture(AsyncWebServerRequest *request){
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
}

void handleShutdown(AsyncWebServerRequest *request){
  if(request->hasHeader("time")){
    AsyncWebHeader* h = request->getHeader("time");
    TIME_TO_SLEEP = atol(h->value().c_str());
    shutdown = true;
    request->send(200, "text/plain", "shutdown initiate");
  }else{
    request->send(400, "text/plain", "no header");
  }
}

void handleSet(AsyncWebServerRequest *request){

  String resp = "";
  
  if(request->hasParam("device_name")){
    AsyncWebParameter* p = request->getParam("device_name");
    if (p->value() != device_name){
      if(!NVS.setString("_device", p->value())){
        log_e("can't write to NVS");
      }
    }
    resp += "device name set success\n";
    log_d("device name set success");
  }else{
    log_w("request doesnt have device_name param");
    resp += "no device_name param\n";
  }

  if(request->hasParam("device_pass")){
    AsyncWebParameter* p = request->getParam("device_pass");
    if (p->value() != Ap_pass){
      if(!NVS.setString("_APpass", p->value())){
        log_e("can't write to NVS");
      }
    }
    resp += "device pass set success\n";
    log_d("device pass set success");
  }else{
    log_w("request doesnt have device_pass param");
    resp += "no device_pass param\n";
  }

  if(request->hasParam("ssid")){
    AsyncWebParameter* p = request->getParam("ssid");
    if (p->value() != ssid){
      if(!NVS.setString("_SSID", p->value())){
        log_e("can't write to NVS");
      }
    }
    resp += "target SSID set success\n";
    log_d("target SSID set success");
  }else{
    log_w("request doesnt have ssid param");
    resp += "no ssid param\n";
  }

  if(request->hasParam("pass")){
    AsyncWebParameter* p = request->getParam("pass");
    if (p->value() != password){
      if(!NVS.setString("_pass", p->value())){
        log_e("can't write to NVS");
      }
    }
    resp += "target password set success\n";
    log_d("target password set success");
  }else{
    log_w("request doesnt have pass param");
    resp += "no pass param\n";
  }

  if(request->hasParam("static_ip")){
    AsyncWebParameter* p = request->getParam("static_ip");
    if (p->value() != static_ip_s){
      if(!NVS.setString("_static_ip", p->value())){
        log_e("can't write to NVS");
      }
    }
    resp += "target static IP set success\n";
    log_d("target static IP set success");
    log_d("value: %s", p->value());
  }else{
    log_w("request doesnt have static_ip param");
    resp += "no static_ip param\n";
  }

  if(request->hasParam("static_gateway")){
    AsyncWebParameter* p = request->getParam("static_gateway");
    if (p->value() != static_gateway_s){
      if(!NVS.setString("_static_gateway", p->value())){
        log_e("can't write to NVS");
      }
    }
    resp += "target static gateway set success\n";
    log_d("target static gateway set success");
    log_d("value: %s", p->value());
  }else{
    log_w("request doesnt have static_gateway param");
    resp += "no static_gateway param\n";
  }

  if(request->hasParam("static_subnet")){
    AsyncWebParameter* p = request->getParam("static_subnet");
    if (p->value() != static_subnet_s){
      if(!NVS.setString("_static_subnet", p->value())){
        log_e("can't write to NVS");
      }
    }
    resp += "target static subnet set success\n";
    log_d("target static subnet set success");
    log_d("value: %s", p->value());
  }else{
    log_w("request doesnt have static_subnet param");
    resp += "no static_subnet param\n";
  }

  resp += "disconnect from AP to restart";
  request->send(200, "text/plain", resp);
}

void handleReset(AsyncWebServerRequest *request){
  if(!NVS.eraseAll()){
    log_e("reset fail");
    request->send(400, "text/plain", "Reset fail");
  }else{
    request->send(200, "text/plain", "Reset done, disconnect from AP to restart");
  }
}

String processor(const String& var){
  log_d("var %s", var);
  if(var == "DEVICE_NAME"){
    return device_name;
  }
  if(var == "DEVICE_PASS"){
    return Ap_pass;
  }
  if(var == "TARGET_SSID"){
    return ssid;
  }
  if(var == "TARGET_PASS"){
    return password;
  }
  if(var == "STATIC_IP"){
    return static_ip_s;
  }
  if(var == "GATEWAY"){
    return static_gateway_s;
  }
  if(var == "SUBNET"){
    return static_subnet_s;
  }
  if(var == "DEVICE_IP"){
    return IP.toString();
  }
  if(var == "MAC"){
    return WiFi.macAddress();
  }
  return String();
}

void forceAP(){
  force_APMode = true;
  log_w("initiate force AP deep sleep");
  esp_sleep_enable_timer_wakeup(1000000);
  esp_deep_sleep_start();
}

bool setSTA_IP(String _ip, String _gateway, String _subnet){

  if ((_ip != NULL) && (_gateway != NULL) && (_subnet != NULL)){

    if (!static_IP.fromString(_ip.c_str())){
      log_d("string static ip conversion fail");
      return false;
    }else if (!static_gateway.fromString(_gateway.c_str())){
      log_d("string static gateway conversion fail");
      return false;
    }else if (!static_subnet.fromString(_subnet.c_str())){
      log_d("string static subnet conversion fail");
      return false;
    }else{
      log_d("all conversion success");
      log_d("Static IP addr %d.%d.%d.%d", static_IP[0], static_IP[1], static_IP[2], static_IP[3]);
      log_d("Static IP addr %d.%d.%d.%d", static_gateway[0], static_gateway[1], static_gateway[2], static_gateway[3]);
      log_d("Static IP addr %d.%d.%d.%d", static_subnet[0], static_subnet[1], static_subnet[2], static_subnet[3]);
    }

    if (!WiFi.config(static_IP, static_gateway, static_subnet)) {
      log_e("Static IP Failed to configure");
      return false;
    }

    return true;
  }else{
    return false;
  }
}

static void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
  if (event == SYSTEM_EVENT_STA_START){
    log_d("WiFi start");
  }else if (event == SYSTEM_EVENT_STA_DISCONNECTED){
    log_d("WiFi Disconnected, initiate AP Mode");
    forceAP();
  }else if(event == SYSTEM_EVENT_STA_CONNECTED){
    log_d("WiFi Connected");
  }else if(event == SYSTEM_EVENT_STA_GOT_IP){
    log_d("Got IP");
    wf_connected = true;
  }else if (event == SYSTEM_EVENT_AP_START){
    log_d("AP Start");
    if (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
      log_e("Access Point Failed to configure");
    }
  }else if (event == SYSTEM_EVENT_AP_STACONNECTED){
    log_d("station connect");
    STA_connect = true;
  }else if (event == SYSTEM_EVENT_AP_STADISCONNECTED){
    log_d("station disconnected");
    ESP.restart();
  }
}

bool initNVS(){

  bool ready = true;

  if (!NVS.getString("_device", device_name)){
    log_w("var device name not found, set to default");
    if(!NVS.setString("_device", "esp32")){
      log_e("can't write to NVS");
      ready = false;
    }
    device_name = "esp32";
  }else if (device_name == NULL){
    log_w("Null device name, set to default");
    if(!NVS.setString("_device", "esp32")){
      log_e("can't write to NVS");
      ready = false;
    }
    device_name = "esp32";
  }

  if (!NVS.getString("_APpass", Ap_pass)){
    log_w("var AP password not found, set to default");
    if(!NVS.setString("_APpass", "12345678")){
      log_e("can't write to NVS");
      ready = false;
    }
    Ap_pass = "12345678";
  }else if (Ap_pass == NULL){
    log_w("Null AP password, set to default");
    if(!NVS.setString("_APpass", "12345678")){
      log_e("can't write to NVS");
      ready = false;
    }
    Ap_pass = "12345678";
  }

  if (!NVS.getString("_SSID", ssid)){
    log_w("var ssid not found");
    if(!NVS.setString("_SSID", "")){
      log_e("can't write to NVS");
      ready = false;
    }
    ready = false;
  }else if (ssid == NULL){
    log_w("Null SSID value");
    ready = false;
  }

  if (!NVS.getString("_pass", password)){
    log_w("var password not found");
    if(!NVS.setString("_pass", "")){
      log_e("can't write to NVS");
      ready = false;
    }
    ready = false;
  }else if (password == NULL){
    log_w("Null password value");
    ready = false;
  }

  if (!NVS.getString("_static_ip", static_ip_s)){
    log_w("var static_ip_s not found");
    if(!NVS.setString("_static_ip", "")){
      log_e("can't write to NVS");
      ready = false;
    }
  }else if (static_ip_s == NULL){
    log_w("Null static_ip_s value");
  }

  if (!NVS.getString("_static_gateway", static_gateway_s)){
    log_w("var static_gateway_s not found");
    if(!NVS.setString("_static_gateway", "")){
      log_e("can't write to NVS");
      ready = false;
    }
  }else if (static_gateway_s == NULL){
    log_w("Null static_gateway_s value");
  }

  if (!NVS.getString("_static_subnet", static_subnet_s)){
    log_w("var static_subnet_s not found");
    if(!NVS.setString("_static_subnet", "")){
      log_e("can't write to NVS");
      ready = false;
    }
  }else if (static_subnet_s == NULL){
    log_w("Null static_subnet_s value");
  }

  log_d("name %s", device_name);
  log_d("AP Pass %s", Ap_pass);
  log_d("ssid %s", ssid);
  log_d("password %s", password);
  log_d("static_ip_s %s", static_ip_s);
  log_d("static_gateway_s %s", static_gateway_s);
  log_d("static_subnet_s %s", static_subnet_s);

  return ready;
}

void initSTA(){
  
  /* WiFi config */
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(WiFiEvent);
  WiFi.setAutoReconnect(true);

  if (setSTA_IP(static_ip_s, static_gateway_s, static_subnet_s)){
    log_d("static IP config success");
  }else{
    log_d("static IP config fail, use DHCP instead");
  }

  WiFi.begin(ssid.c_str(), password.c_str());
  log_i("Connecting to WiFi..");

  /* wait for WiFi connected status */
  prev = esp_timer_get_time();
  while (!wf_connected) {
    if ((esp_timer_get_time() - prev) > 15000000) { // Initiate AP mode if timeout
      log_w("initiate force AP");
      forceAP();
    }
  }

  delay(1000);
  IP = WiFi.localIP(); // Get device IP address
  log_d("IP addr %d.%d.%d.%d", IP[0], IP[1], IP[2], IP[3]);

  /* Initiate camera */
  if(!initCamera()){
    log_e("Failed to initialize camera...");
    ESP.restart();
  }

  /* Web server routing */
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    log_d("request root");
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    log_d("request css");
    request->send(SPIFFS, "/style.css", "text/css");
  });

  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request){
    log_d("request root");
    request->send(SPIFFS, "/config_AP.html", String(), false, processor);
  });

  server.on("/capture", HTTP_GET, handleCapture);
  server.on("/shutdown", HTTP_GET, handleShutdown);
  server.on("/set", HTTP_GET, handleSet);
  server.on("/reset", HTTP_GET, handleReset);
 
  server.begin();
  serialServer.begin();
  client = serialServer.available();
}

void initAP(){

  force_APMode = false;

  /* WiFi config */
  WiFi.mode(WIFI_AP);
  WiFi.onEvent(WiFiEvent);
  WiFi.softAP(device_name.c_str(), Ap_pass.c_str());
  delay(1000);

  IP = WiFi.softAPIP(); // Get device IP address
  log_d("IP addr %d.%d.%d.%d", IP[0], IP[1], IP[2], IP[3]);

  /* Web server routing */
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    log_d("request root");
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    log_d("request css");
    request->send(SPIFFS, "/style.css", "text/css");
  });

  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request){
    log_d("request root");
    request->send(SPIFFS, "/config_AP.html", String(), false, processor);
  });

  server.on("/set", HTTP_GET, handleSet);
  server.on("/reset", HTTP_GET, handleReset);

  server.begin();
}

void setup() {

  pinMode(led_pin, OUTPUT);
  pinMode(reset_pin, INPUT_PULLUP);
  digitalWrite(led_pin, HIGH); // Turn on led 

  /* Initialize SPIFFS */
  if(!SPIFFS.begin(true)){
    log_e("An Error has occurred while mounting SPIFFS");
    return;
  }
  /* Initialize NVS */
  NVS.begin(); 

  if (!initNVS() || force_APMode){
    log_d("config invalid, initiate AP Mode");
    initAP(); // Initiate AP mode
  }else{
    log_d("config valid, initiate STA Mode");
    initSTA(); //Initiate STA mode
    STA_Mode = true;
  }

  log_d("MAC address: %s", WiFi.macAddress()); // Print ESP MAC Address

  prev = esp_timer_get_time(); // For led blink timer
  timer = esp_timer_get_time(); // For AP mode timeout
}
 
void loop(){

  if(digitalRead(reset_pin) == LOW){
    if(!NVS.eraseAll()){
      log_i("Config reset failed");
    }else{
      log_i("Config reset success");
      ESP.restart();
    }
  }

  if (STA_Mode){
    /* Led blink controller */
    if (wf_connected){
      if(esp_timer_get_time() - prev > 1500000){
        toggle = !toggle;
        digitalWrite(led_pin, toggle);
        prev = esp_timer_get_time();
        log_i("RSSI : %d dBm", WiFi.RSSI());
      }
    }
    /* Initiate deep sleep mode */
    if (shutdown){
      log_w("Initiate shutdown timer: %d h", TIME_TO_SLEEP);
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_H_FACTOR);
      esp_deep_sleep_start();
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

  }else{
    /* AP mode timeout */
    if (((esp_timer_get_time() - timer) > AP_wait) && !STA_connect) {
      log_d("AP mode timeout, restarting...");
      ESP.restart();
    }
    /* Led blink controller */
    if (esp_timer_get_time() - prev > 4000000) {
      toggle = !toggle;
      digitalWrite(led_pin, toggle);
      prev = esp_timer_get_time();
    }
  }
  
}