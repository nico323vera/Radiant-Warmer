#define BLYNK_TEMPLATE_ID "TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "TEMPLATE_NAME"
#define BLYNK_AUTH_TOKEN "AUTH_TOKEN"

#include <esp_timer.h>
#include <esp_now.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// Blynk Client with curve corrections and control.
int spValidation;
float vSP_Blynk;

const char* ssid = "Your SSID";
const char* password = "Your Password";

// Constants
const int SAMPLING_TIME_US = 1000000;
const float R5 = 4613.5;
const float P1 = -0.00550579311431793;
const float P2 = 60.0762980626865;
const int TEMP_PIN = 35;
const int INTERRUPT_PIN = 32;
const int RELE_PIN = 33;
const int TRIAC_PIN = 25;
const int CURRENT_PIN = 26;
const float T_LIM = 8333.3;
const float offSet_calib = 0.12;
const float tempOffset = 2.1;

// PID Constants
const float Ki = 0.0206098250355602;
const float Kp = 4.94211509619533;
const float Kd = 46.5440789482533;  
const float Ka = Ki;

// FIR Filter
const float a = 2.966737452995536e-04;
const float b = 0.000293267011239556;
const float c = 1.96535520406779;
const float d = 0.965945144824327;

// Closed Loop Variables
float err = 0.0;
float err_1 = 0.0;
float err_2 = 0.0;
float temp_f = 0.0;
float temp_f_1 = 0.0;
float temp_f_2 = 0.0;
float temp_1 = 0.0;
float temp_2 = 0.0;
float SP = 36.0;
float U1 = 0.0;
float U2 = 0.0;
float U = 0.0;
float U_sat = 0.0;
float diff_act = 0.0;

// Variables
volatile bool flagPrint = false;
volatile bool flagFlank = false;
volatile float Vr5;
volatile float temp = 0.0;
float percentage = 0.0;
float t_alpha = 0.0;
int64_t t1 = 0;
int64_t t2 = 0;
int counter_data_server = 0;
String printWord;

// ESPNOW Needed data
typedef struct struct_message {
  float number = 36.0;
  float vSP;
  int percent;
  bool heater;
  bool emerStop;
  bool vSPdetection;
} struct_message;

struct_message myData;
struct_message receivedData;

uint8_t receiverAddress[] = { 0xEC, 0xDA, 0x3B, 0x98, 0x76, 0xFC };  // Replace with the receiver's MAC address

String alarma;

// Data from server
bool indAdministrador = false;
float setPointTemperatura;

TaskHandle_t blynkTaskHandle = NULL;

// Timer interrupt
void IRAM_ATTR onTimer(void *arg) {
  Vr5 = analogRead(35);
  flagPrint = true;
}

void IRAM_ATTR handleFallingInterrupt() {
  t1 = esp_timer_get_time();
  flagFlank = true;
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
}

void sendData(struct_message *data) {
  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)data, sizeof(struct_message));
}

void handleTemperatureCalculation() {
  Vr5 = (3.3 * Vr5 / 1023.0) + offSet_calib;
  float RTD = ((3.3 / Vr5) * R5) - R5;
  temp = P1 * RTD + P2 - tempOffset;

  if (temp >= 45.0) {
    temp = 45.0;
  }
  if (temp <= 20.0) {
    temp = 20.0;
  }
}

void handleTriacTriggering() {
  if (flagFlank) {
    t2 = esp_timer_get_time();
  }
  if ((t2 - t1 >= t_alpha) && (t2 - t1 <= T_LIM)) {
    digitalWrite(TRIAC_PIN, HIGH);
    flagFlank = false;
  } else {
    digitalWrite(TRIAC_PIN, LOW);
  }
}

void percentageToAlpha() {
  t_alpha = -80.3 * percentage + 8030.0;
  if (t_alpha > T_LIM) {
    t_alpha = T_LIM;
  } else if (t_alpha < 0.0) {
    t_alpha = 0.0;
  }
}

void closedLoop() {
  err = SP - temp_f;
  U1 = Kp * err + Ki * (err_1 + err - diff_act * Ka);
  U2 = Kd * (temp_f - temp_f_1);
  U = U1 - U2;

  if (U >= 100.0) {
    U_sat = 100.0;
  } else if (U <= 0.0) {
    U_sat = 0.0;
  } else {
    U_sat = U;
  }

  err_1 = err_1 + err - diff_act * Ka;
  diff_act = U - U_sat;
  percentage = U_sat;
  percentageToAlpha();
}

void temperatureFIR() {
  temp_f = a * temp_1 + b * temp_2 + c * temp_f_1 - d * temp_f_2;
  temp_2 = temp_1;
  temp_1 = temp;
  temp_f_2 = temp_f_1;
  temp_f_1 = temp_f;

  if (temp_f >= 50.0 || temp <= 20.0) {
    resetSensorFIR();
  }
}

void resetSensorFIR() {
  temp_f = temp;
  temp_1 = temp;
  temp_2 = temp;
  temp_f_2 = temp;
  temp_f_1 = temp;
}

void alarmsChecker() {
  if (digitalRead(RELE_PIN)) {
    myData.emerStop = true;
  } else {
    myData.emerStop = false;
    alarma = "Parada de Emergencia";
  }

  if (analogRead(CURRENT_PIN) >= 300) {
    myData.heater = false;
  } else {
    myData.heater = true;
  }
}

// Blynk virtual pin handlers

BLYNK_WRITE(V3){
  spValidation = param.asInt();
}

BLYNK_WRITE(V1) {
  vSP_Blynk = param.asFloat();
}

void blynkTask(void * pvParameters) {
  for(;;) {
    Blynk.run();
    vTaskDelay(10 / portTICK_PERIOD_MS); // Adjust the delay as needed
  }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(10);

  // Set up the timer
  const esp_timer_create_args_t timerArgs = {
    .callback = &onTimer,
    .name = "sampleTimer"
  };

  esp_timer_handle_t timer;
  esp_timer_create(&timerArgs, &timer);

  // Start the timer
  esp_timer_start_periodic(timer, SAMPLING_TIME_US);

  // Setup external interrupt
  pinMode(TRIAC_PIN, OUTPUT);
  pinMode(RELE_PIN, INPUT);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleFallingInterrupt, FALLING);
  
  // Setup ESP-NOW
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
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = wifi_channel;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Vr5 = analogRead(35);
  handleTemperatureCalculation();

  temp_f = temp;
  temp_f_1 = temp;
  temp_f_2 = temp;
  temp_1 = temp;
  temp_2 = temp;

  // Initialize Blynk
  Blynk.config(BLYNK_AUTH_TOKEN);

  // Create a task for Blynk communication
  xTaskCreate(blynkTask, "Blynk Task", 4096, NULL, 1, &blynkTaskHandle);
}

void loop() {
  handleTriacTriggering();

  if (flagPrint) {
    handleTemperatureCalculation();
    temperatureFIR();
    closedLoop();
    alarmsChecker();
    printWord = String(String(temp) + '\t' + String(temp_f) + '\t' + String(U) + '\t' + String(U_sat) + '\t' + String(SP) + '\t' + String(err));
    Serial.println(printWord);
    flagPrint = false;
    myData.number = temp_f;
    myData.percent = U_sat;
    sendData(&myData);
    Blynk.virtualWrite(V0, temp_f);  // Send temperature to Blynk
    Blynk.virtualWrite(V2, U_sat);  // Send percentage to Blynk
  }

  if ((receivedData.number >= 0.0) && (spValidation == 0)) {
    SP = receivedData.number;
    myData.vSPdetection = false;
  }else{
    if (vSP_Blynk >= 36.0 && vSP_Blynk <= 38.0){
      SP = vSP_Blynk;
      myData.vSPdetection = true;
      myData.vSP = vSP_Blynk;
    }else{
      SP = receivedData.number;
    }
  }
}
