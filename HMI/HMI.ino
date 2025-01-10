#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

//HTTP Client pair last version

#define USE_UI    //if you want to use the ui export from Squareline, please do not annotate this line.

#if defined USE_UI
#include <lvgl.h>
#include "ui.h"
#endif

#include "globals.h"

#include <Arduino_GFX_Library.h>
#define TFT_BL 2
#define GFX_BL DF_GFX_BL // default backlight pin, you may replace DF_GFX_BL to actual backlight pin

#define Display_70

const char* ssid = "Your SSID";
const char* password = "Your Password";

typedef struct struct_message {
    float number;
    float vSP;
    int percent;
    bool heater;
    bool emerStop;
    bool vSPdetection;
} struct_message;

struct_message incomingReadings;
struct_message sendDataStruct;

bool flagSendData = false;
bool tempAlarm = false;

float lastValue = -1.0;

uint8_t peerMAC[] = {0xC8, 0xF0, 0x9E, 0xF4, 0xC5, 0x58};

TaskHandle_t espNowTaskHandle = NULL;
QueueHandle_t incomingQueue;
QueueHandle_t outgoingQueue;

/*******************************************************************************
 * Screen Driver Configuration 
*******************************************************************************/
#if defined (Display_70)       //7.0INCH 800x480
Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
  GFX_NOT_DEFINED /* CS */, GFX_NOT_DEFINED /* SCK */, GFX_NOT_DEFINED /* SDA */,
  41 /* DE */, 40 /* VSYNC */, 39 /* HSYNC */, 0 /* PCLK */,
  14 /* R0 */, 21 /* R1 */, 47 /* R2 */, 48 /* R3 */, 45 /* R4 */,
  9 /* G0 */, 46 /* G1 */, 3 /* G2 */, 8 /* G3 */, 16 /* G4 */, 1 /* G5 */,
  15 /* B0 */, 7 /* B1 */, 6 /* B2 */, 5 /* B3 */, 4 /* B4 */
);
Arduino_RPi_DPI_RGBPanel *lcd = new Arduino_RPi_DPI_RGBPanel(
  bus,
  800 /* width */, 0 /* hsync_polarity */, 210 /* hsync_front_porch */, 1 /* hsync_pulse_width */, 46 /* hsync_back_porch */,
  480 /* height */, 0 /* vsync_polarity */, 22 /* vsync_front_porch */, 1 /* vsync_pulse_width */, 23 /* vsync_back_porch */,
  0 /* pclk_active_neg */, 16000000 /* prefer_speed */, true /* auto_flush */);
#endif

/*******************************************************************************
 * Screen Driver Configuration  end
*******************************************************************************/

/*******************************************************************************
   Please config the touch panel in touch.h
 ******************************************************************************/
#include "touch.h"

#ifdef USE_UI
/* Change to your screen resolution */
static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t disp_draw_buf[800 * 480 / 10];      //5,7inch: lv_color_t disp_draw_buf[800*480/10]            4.3inch: lv_color_t disp_draw_buf[480*272/10]
//static lv_color_t disp_draw_buf;
static lv_disp_drv_t disp_drv;

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  lcd->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  lcd->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PR;

      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
      Serial.print( "Data x :" );
      Serial.println( touch_last_x );

      Serial.print( "Data y :" );
      Serial.println( touch_last_y );
    }
    else if (touch_released())
    {
      data->state = LV_INDEV_STATE_REL;
    }
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
  delay(15);
}
#endif

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (incomingData == nullptr || len < sizeof(struct_message)) {
    Serial.println("Invalid data received");
    return;
  }
  struct_message msg;
  memcpy(&msg, incomingData, sizeof(struct_message));
  xQueueSend(incomingQueue, &msg, portMAX_DELAY);
}

void sendData(struct_message *data) {
  if (data == nullptr) {
    Serial.println("Invalid data pointer");
    return;
  }

  esp_err_t result = esp_now_send(peerMAC, (uint8_t *)data, sizeof(struct_message));

  if (result == ESP_OK) {
    Serial.println("Data sent successfully");
  } else {
    Serial.println("Error sending data");
    Serial.println(result);
  }
}

void espNowTask(void *pvParameters) {
  struct_message msg;
  while (true) {
    if (xQueueReceive(incomingQueue, &msg, portMAX_DELAY) == pdPASS) {
      // Process received data
      incomingReadings = msg;
      String temp = String(incomingReadings.number, 1);
      String vSP = String(incomingReadings.vSP, 1);
      String currentSPs = String(currentSP, 1);
      alarmsCheck(incomingReadings.number);
      
      const char* cString = temp.c_str();
      const char* cvSP = vSP.c_str();
      const char* ccurrentSP = currentSPs.c_str();

      if (!tempAlarm){
        lv_label_set_text(ui_pv, cString);
      } else {
        lv_label_set_text(ui_pv, "ERR");
      }

      if (incomingReadings.vSPdetection){
        lv_obj_set_style_bg_color(ui_sensorDetection1, lv_color_hex(0x00FF0D), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_label_set_text(ui_lastSPlabel, cvSP);
      }else{
        lv_obj_set_style_bg_color(ui_sensorDetection1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_label_set_text(ui_lastSPlabel, ccurrentSP);
      }

      lv_bar_set_value(ui_powerBar, incomingReadings.percent, LV_ANIM_OFF);
    }

    if (flagSendData) {
      sendDataStruct.number = finalValue;
      sendData(&sendDataStruct);
      Serial.println(String(sendDataStruct.number));
      flagSendData = false;
    }
  }
}

void setup() {
  Serial.begin(115200);
  // IO Port Pins
  pinMode(38, OUTPUT);
  digitalWrite(38, LOW);
  pinMode(17, OUTPUT);
  digitalWrite(17, LOW);
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);
  pinMode(42, OUTPUT);
  digitalWrite(42, LOW);
  // Init Display
  lcd->begin();
  lcd->fillScreen(BLACK);
  lcd->setTextSize(2);

  incomingQueue = xQueueCreate(10, sizeof(struct_message));
  outgoingQueue = xQueueCreate(10, sizeof(struct_message));
  
  xTaskCreate(espNowTask, "ESPNowTask", 4096, NULL, 1, &espNowTaskHandle);

  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to WiFi");

  uint8_t wifi_channel = WiFi.channel();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo)); 
  memcpy(peerInfo.peer_addr, peerMAC, 6);
  peerInfo.channel = wifi_channel;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

#ifdef USE_UI
  lv_init();
  delay(100);
  touch_init();

  screenWidth = lcd->width();
  screenHeight = lcd->height();

  lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, screenWidth * screenHeight / 10);
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);
#endif

#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif

  ui_init(); // UI from Squareline or GUI Guider
  Serial.println("Setup done");
}

void loop() {
  lv_timer_handler();
  delay(5);
}

void alarmsCheck(float temp) {
  if (temp <= 20.0 || temp >= 45.0){
    lv_obj_set_style_bg_color(ui_sensorDetection, lv_color_hex(0x00FF0D), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_label_set_text(ui_pv, "ERR");
  } else {
    lv_obj_set_style_bg_color(ui_sensorDetection, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
  }

  if (temp > 39.0){
    lv_obj_set_style_bg_color(ui_tempUpperLimit, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);
  } else {
    lv_obj_set_style_bg_color(ui_tempUpperLimit, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
  }

  if (temp < 35.0){
    lv_obj_set_style_bg_color(ui_tempLowerLimit, lv_color_hex(0x0800FF), LV_PART_MAIN | LV_STATE_DEFAULT);
  } else {
    lv_obj_set_style_bg_color(ui_tempLowerLimit, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
  }

  if (incomingReadings.heater){
    lv_obj_set_style_bg_color(ui_resistorDetection, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
  } else {
    lv_obj_set_style_bg_color(ui_resistorDetection, lv_color_hex(0xD4FF00), LV_PART_MAIN | LV_STATE_DEFAULT);    
  }

  if (incomingReadings.emerStop){
    lv_obj_set_style_bg_color(ui_emergencyStop, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
  } else {
    lv_obj_set_style_bg_color(ui_emergencyStop, lv_color_hex(0xFF7D00), LV_PART_MAIN | LV_STATE_DEFAULT);
  }
}