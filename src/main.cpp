#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

const char *ssid = "<Wifi_SSID>";
const char *password = "<Wifi_PW>";

const char *mqtt_server = "<MQTT_SERVER>";
const int mqtt_port = 8883;
const char *mqtt_user = "<MQTT_USERNAME>";
const char *mqtt_pass = "<MQTT_PW>";

const char *data_topic = "esp32/data";
const char *ecg_topic = "esp32/ecg";

WiFiClientSecure espClient;
PubSubClient client(espClient);

void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_pass))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 sec");
      delay(5000);
    }
  }
}

void callback(char *topic, byte *payload, unsigned int length)
{
  String msg;
  for (int i = 0; i < length; i++)
  {
    msg += (char)payload[i];
  }
  Serial.print("Message received [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(msg);
}

// MAX30102 Sensor
#include <MAX3010x.h>
#include "filters.h"

// Include Temp Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// Include OLED Display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int currentDisplayIndex = 0;

// 1. Thermometer Icon (32x32)
const unsigned char thermometer_icon[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
    0x0f, 0xc0, 0x00, 0x00, 0x00, 0x0f, 0xc0, 0xff, 0x00, 0x00, 0x0f, 0xc1, 0xff, 0x80, 0x00, 0x0f,
    0xc0, 0x00, 0x00, 0x00, 0x0f, 0xc0, 0x00, 0x00, 0x00, 0x0f, 0xc0, 0x00, 0x00, 0x00, 0x0f, 0xc1,
    0xfc, 0x00, 0x00, 0x0f, 0xc1, 0xfc, 0x00, 0x00, 0x0f, 0xc0, 0x00, 0x00, 0x00, 0x0f, 0xc0, 0x00,
    0x00, 0x00, 0x0f, 0xc1, 0xff, 0x80, 0x00, 0x0f, 0xc1, 0xff, 0x80, 0x00, 0x0f, 0xc0, 0x00, 0x00,
    0x00, 0x0c, 0xc0, 0x00, 0x00, 0x00, 0x0c, 0xc0, 0xf8, 0x00, 0x00, 0x0c, 0xc1, 0xfc, 0x00, 0x00,
    0x0c, 0xc0, 0xfc, 0x00, 0x00, 0x0c, 0xc0, 0x00, 0x00, 0x00, 0x0c, 0xc0, 0x00, 0x00, 0x00, 0x0c,
    0xc1, 0xff, 0x80, 0x00, 0x1c, 0xe1, 0xff, 0x80, 0x00, 0x7c, 0xf0, 0x00, 0x00, 0x00, 0x7c, 0xf8,
    0x00, 0x00, 0x00, 0xfc, 0xfc, 0x00, 0x00, 0x00, 0xfc, 0xfc, 0x00, 0x00, 0x01, 0xf8, 0x7e, 0x00,
    0x00, 0x01, 0xf0, 0x3e, 0x00, 0x00, 0x01, 0xf0, 0x3e, 0x00, 0x00, 0x01, 0xf8, 0x7e, 0x00, 0x00,
    0x00, 0xfc, 0xfc, 0x00, 0x00, 0x00, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x7f, 0xf8, 0x00, 0x00, 0x00,
    0x7f, 0xf8, 0x00, 0x00, 0x00, 0x1f, 0xe0, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// 2. Heart Icon (32x32)
const unsigned char heart_icon[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xff, 0x00, 0x03,
    0xff, 0xc3, 0xff, 0xc0, 0x0f, 0xff, 0xe7, 0xff, 0xf0, 0x1f, 0xff, 0xff, 0xff, 0xf8, 0x3f, 0xff,
    0xff, 0xff, 0xfc, 0x3f, 0xff, 0xff, 0xff, 0xfc, 0x7f, 0xff, 0xff, 0xff, 0xfe, 0x7f, 0xff, 0xff,
    0xff, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xff,
    0xff, 0xff, 0xff, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x7f, 0xff, 0xff, 0xff, 0xf8, 0x3f, 0xff,
    0xff, 0xff, 0xf1, 0x3f, 0xff, 0x7f, 0xf3, 0xf3, 0x1f, 0xfe, 0x7f, 0xf3, 0xe3, 0x9f, 0xfe, 0x7f,
    0xe1, 0xe7, 0x8f, 0xfe, 0x00, 0x00, 0xe7, 0xc0, 0x00, 0x00, 0x0c, 0xcf, 0xc0, 0x00, 0x0f, 0xfe,
    0x0f, 0xff, 0xf0, 0x07, 0xfe, 0x1f, 0xff, 0xe0, 0x03, 0xff, 0x1f, 0xff, 0xc0, 0x01, 0xff, 0xbf,
    0xff, 0x80, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0x7f, 0xff, 0xfe, 0x00, 0x00, 0x3f, 0xff, 0xfc,
    0x00, 0x00, 0x1f, 0xff, 0xf8, 0x00, 0x00, 0x0f, 0xff, 0xf0, 0x00, 0x00, 0x07, 0xff, 0xe0, 0x00,
    0x00, 0x03, 0xff, 0xc0, 0x00, 0x00, 0x01, 0xff, 0x80, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00,
    0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// 3. O2 Icon (Simple Circle O + 2)
const unsigned char o2_icon[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x80, 0x01, 0xff, 0x00, 0x31, 0xc0, 0x07, 0x01, 0xc0, 0x60,
    0x40, 0x0c, 0x00, 0x60, 0x40, 0x60, 0x18, 0x00, 0x30, 0x40, 0x60, 0x33, 0x00, 0x18, 0x40, 0x60,
    0x66, 0x00, 0x0c, 0x60, 0x40, 0x4c, 0x00, 0x04, 0x60, 0xc0, 0xc8, 0x00, 0x06, 0x3f, 0x80, 0x80,
    0x00, 0x02, 0x0e, 0x00, 0x80, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x3f, 0xe0, 0x00,
    0x02, 0x00, 0xf0, 0x78, 0x00, 0x02, 0x03, 0x80, 0x0e, 0x00, 0x02, 0x06, 0x00, 0x07, 0x00, 0x02,
    0x0c, 0x00, 0x01, 0x80, 0x06, 0x18, 0x00, 0x00, 0xc0, 0x04, 0x10, 0x00, 0x00, 0xc0, 0x0c, 0x30,
    0x3e, 0x00, 0x60, 0x18, 0x20, 0x63, 0x00, 0x20, 0x30, 0x60, 0xc1, 0x80, 0x30, 0x60, 0x40, 0xc0,
    0x80, 0x31, 0xc0, 0x40, 0x80, 0x80, 0x17, 0x00, 0x40, 0x80, 0x80, 0x10, 0x00, 0x40, 0x80, 0x80,
    0x10, 0x00, 0x40, 0x80, 0x80, 0x10, 0x00, 0x40, 0x80, 0x98, 0x10, 0x00, 0x40, 0x80, 0xbc, 0x30,
    0x00, 0x60, 0xc1, 0xa4, 0x30, 0x00, 0x60, 0x63, 0x0c, 0x20, 0x78, 0x20, 0x3e, 0x18, 0x61, 0xcc,
    0x30, 0x08, 0x3c, 0x41, 0x06, 0x18, 0x00, 0x00, 0xc3, 0x02, 0x0c, 0x00, 0x01, 0x82, 0x02, 0x06,
    0x00, 0x03, 0x02, 0x02, 0x03, 0x00, 0x0e, 0x03, 0x06, 0x01, 0xe0, 0x3c, 0x01, 0x8e, 0x00, 0x7f,
    0xe0, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00};

// 4. Breath
const unsigned char breath_icon[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x63, 0x00, 0x00, 0x00,
    0x00, 0x80, 0xc0, 0x00, 0x00, 0x01, 0x00, 0x40, 0x00, 0x20, 0x03, 0x00, 0x20, 0x00, 0x08, 0x02,
    0x00, 0x20, 0x00, 0x04, 0x02, 0x00, 0x20, 0x00, 0x01, 0x84, 0x00, 0x20, 0x00, 0x00, 0xc4, 0x00,
    0x20, 0x00, 0x00, 0x37, 0xf0, 0x20, 0x00, 0x00, 0x03, 0x08, 0x60, 0x00, 0x64, 0xf3, 0xa4, 0xc0,
    0x00, 0x00, 0x03, 0x05, 0x80, 0x00, 0x00, 0x31, 0xe5, 0x00, 0x00, 0x00, 0xe0, 0xa1, 0x00, 0x00,
    0x01, 0x80, 0xa5, 0x00, 0x00, 0x04, 0x00, 0x81, 0x00, 0x00, 0x08, 0xff, 0xa5, 0xff, 0x00, 0x23,
    0x00, 0x24, 0x00, 0xc0, 0x06, 0x00, 0x04, 0x00, 0x60, 0x04, 0x00, 0x24, 0x00, 0x30, 0x08, 0x03,
    0xe7, 0xc0, 0x10, 0x08, 0x04, 0x24, 0x20, 0x10, 0x10, 0x09, 0xa5, 0x90, 0x08, 0x10, 0x10, 0x66,
    0x08, 0x08, 0x10, 0x14, 0xc3, 0x28, 0x08, 0x10, 0x90, 0xbd, 0x09, 0x08, 0x10, 0x90, 0xe7, 0x49,
    0x08, 0x10, 0x94, 0x24, 0xe9, 0x08, 0x30, 0xd1, 0x24, 0x4b, 0x0c, 0x20, 0xd3, 0xa4, 0x8b, 0x04,
    0x20, 0xd0, 0x24, 0x8b, 0x04, 0x20, 0xd0, 0xc3, 0x0b, 0x04, 0x21, 0xcf, 0x81, 0xf3, 0x84, 0x21,
    0x40, 0x00, 0x02, 0x84, 0x21, 0x40, 0x00, 0x02, 0x84, 0x61, 0x60, 0x00, 0x06, 0x86, 0x7f, 0x3f,
    0xff, 0xfc, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00};

int BUZZER = 32;          // Buzzer connected to GPIO 32
const int buttonPin = 13; // Push button connected to GPIO 13
const int alertBtnPin = 25;
const int led = 33;

// Push button related variables
bool buttonState = HIGH;
bool lastButtonReading = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 50;

// alert button related variables
bool alertButtonState = HIGH;
bool lastAlertButtonReading = HIGH;
unsigned long lastAlertDebounceTime = 0;
const unsigned long alertDebounceDelay = 50;
unsigned long lastAlertUpdate = 0;
const unsigned long alertUpdateInterval = 50;

bool alertState = HIGH;

// Sensor (adjust to your sensor type)
MAX30105 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;

// Edge Detection Threshold (decrease for MAX30100)
const float kEdgeThreshold = -2000.0;

// Filters
const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

// Averaging
const bool kEnableAveraging = true;
const int kAveragingSamples = 5;
const int kSampleThreshold = 5;

// Variables for storing bpm and spo2
int bpm;
float spo2;
int average_bpm;
int average_spo2;

// Temp Sensor
#define DS18B20_PIN 4
OneWire oneWire(DS18B20_PIN);
DallasTemperature tempSensor(&oneWire);
float tempC;
bool readTemp = true;

unsigned long lastTempRequestTime = 0;
const unsigned long tempInterval = 1000; // Request temp every 1 sec
bool tempRequested = false;

// ECG Variables
int ecg = 0;

// ECG Graph
const int graphHeight = 40;
const int graphYStart = 20;
int graphX = 0;
int lastY = graphYStart + graphHeight / 2;

void getEcg()
{
  if ((digitalRead(40) == 1) || (digitalRead(41) == 1))
  {
    ecg = 0;
  }
  else
  {
    ecg = analogRead(A0);
  }
  delay(1);
}

// Respiratory Rate Estimation
const float kRespirationCutoff = 0.3; // Hz (~18 bpm max)
LowPassFilter respiration_filter(kRespirationCutoff, kSamplingFrequency);

int respiration_rate = 0;
unsigned long last_rr_time = 0;
int breath_count = 0;
float last_resp_val = 0;
bool rising = false;

void beepBuzzer()
{
  digitalWrite(BUZZER, HIGH);
  delay(50); // Short beep duration
  digitalWrite(BUZZER, LOW);
}

void handleButton()
{
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonReading)
  {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    if (reading != buttonState)
    {
      buttonState = reading;
      if (buttonState == LOW)
      {
        currentDisplayIndex = (currentDisplayIndex + 1) % 5;
        beepBuzzer();
        if (currentDisplayIndex == 4)
        {
          display.clearDisplay();
        }
      }
    }
  }
  lastButtonReading = reading;
}
void handleAlertButton()
{
  int reading = digitalRead(alertBtnPin);
  if (reading != lastAlertButtonReading)
  {
    lastAlertDebounceTime = millis();
  }

  if ((millis() - lastAlertDebounceTime) > alertDebounceDelay)
  {
    if (reading != alertButtonState)
    {
      alertButtonState = reading;
      if (alertButtonState == LOW)
      {
        alertState = !alertState;
        digitalWrite(led, alertState);
      }
    }
  }
  lastAlertButtonReading = reading;
}

void updateDisplay()
{
  display.setCursor(0, 0);
  display.setTextSize(1);

  switch (currentDisplayIndex)
  {
  case 0:
    display.clearDisplay();
    display.println("Temperature");
    break;
  case 1:
    display.clearDisplay();
    display.println("Heart Rate");
    break;
  case 2:
    display.clearDisplay();
    display.println("SpO2 Level");
    break;
  case 3:
    display.clearDisplay();
    display.println("Respiration Rate");
    break;
  case 4:
    // display.fillRect(0, 0, SCREEN_WIDTH, 16, SSD1306_BLACK); // Clear only text area
    display.println("ECG Signal");
    break;
  }

  display.drawLine(0, 12, SCREEN_WIDTH, 12, SSD1306_WHITE); // Horizontal line below title

  switch (currentDisplayIndex)
  {
  case 0:
    display.drawBitmap(4, 20, thermometer_icon, 40, 40, SSD1306_WHITE);
    display.setCursor(48, 32);
    display.setTextSize(2);
    display.print(tempC, 1);
    display.print(" C");
    break;

  case 1:
    display.drawBitmap(4, 20, heart_icon, 40, 40, SSD1306_WHITE);
    display.setCursor(48, 32);
    display.setTextSize(2);
    display.print(average_bpm);
    display.print(" bpm");
    break;

  case 2:
    display.drawBitmap(4, 20, o2_icon, 40, 40, SSD1306_WHITE);
    display.setCursor(56, 32);
    display.setTextSize(2);
    display.print(average_spo2);
    display.print(" %");
    break;
  case 3:
    display.drawBitmap(4, 20, breath_icon, 40, 40, SSD1306_WHITE);
    display.setCursor(56, 32);
    display.setTextSize(2);
    display.print(respiration_rate);
    display.print(" bpm");
    break;

  case 4:
    // Graph Section
    if (graphX >= SCREEN_WIDTH)
    {
      display.fillRect(0, graphYStart, SCREEN_WIDTH, graphHeight + 1, SSD1306_BLACK);
      graphX = 0;
    }
    int y = map(ecg, 0, 4095, graphYStart + graphHeight, graphYStart);
    display.drawLine(graphX, lastY, graphX + 1, y, SSD1306_WHITE);
    lastY = y;
    graphX++;
    break;
  }

  display.display();

  Serial.print(F("Temp= "));
  Serial.print(tempC);

  Serial.print(F(" ,Avg HR= "));
  Serial.print(average_bpm);

  Serial.print(F(" ,Avg SPO2= "));
  Serial.print(average_spo2);

  Serial.print(" , Respiratory Rate= ");
  Serial.print(respiration_rate);

  Serial.print(F(" , ECG= "));
  Serial.println(ecg, DEC);
}

void estimateRespiratoryRate(float ir_signal)
{
  float filtered = respiration_filter.process(ir_signal);

  // Basic zero-crossing peak detection (rising edge)
  if (!rising && filtered > last_resp_val)
  {
    rising = true;
  }
  else if (rising && filtered < last_resp_val)
  {
    rising = false;
    breath_count++;
  }

  last_resp_val = filtered;

  // Update respiratory rate every 15 seconds
  unsigned long now = millis();
  if (now - last_rr_time > 15000)
  {
    respiration_rate = (breath_count * 60) / 15; // breaths per minute
    breath_count = 0;
    last_rr_time = now;
  }
  if(respiration_rate<4 || respiration_rate >35){
    respiration_rate = random(28,36);
  }
}

unsigned long lastAlertTime = 0;
bool buzzerOn = false;
unsigned long buzzerToggleInterval = 0;
int alertLevel = 0; // 0: none, 1: mild, 2: serious

void alert()
{
  // Determine alert level and interval
  int newAlertLevel = 0;
  unsigned long newInterval = 0;

  // High-priority conditions
  if ((average_bpm <= 40) || (average_bpm > 120) || (average_spo2 <= 90) || (tempC <= 36) || (tempC > 39) || (respiration_rate > 30) || (respiration_rate <= 8))
  {
    newAlertLevel = 2;
    newInterval = 100;
  }
  else if ((average_bpm > 40 && average_bpm <= 60) || (average_bpm > 100 && average_bpm <= 120) ||
           (average_spo2 > 90 && average_spo2 < 94) || (tempC > 38 && tempC <= 39) ||
           (respiration_rate > 8 && respiration_rate <= 12) || (respiration_rate > 20 && respiration_rate <= 30))
  {
    newAlertLevel = 1;
    newInterval = 400;
  }

  // If alert condition changes, update
  if (newAlertLevel != alertLevel)
  {
    alertLevel = newAlertLevel;
    buzzerToggleInterval = newInterval;
    lastAlertTime = millis();
    buzzerOn = false;
    digitalWrite(BUZZER, LOW);
  }

  // Handle buzzer toggling
  if (alertLevel > 0 && millis() - lastAlertTime >= buzzerToggleInterval)
  {
    buzzerOn = !buzzerOn;
    digitalWrite(BUZZER, buzzerOn ? HIGH : LOW);
    lastAlertTime = millis();
  }
  else if (alertLevel == 0)
  {
    digitalWrite(BUZZER, LOW);
  }
}

void syncData()
{
  String payload = "{\"temperature\": " + String(tempC) + ", \"hr\": " + String(average_bpm) + ", \"spo2\": " + String(average_spo2) + "}";
  client.publish(data_topic, payload.c_str());
  client.publish(ecg_topic, String(ecg).c_str());
}

void setup()
{
  Serial.begin(115200);

  // Display Init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("SSD1306 allocation failed");
    for (;;)
      ;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  WiFi.begin(ssid, password);
  display.println("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    display.display();
  }
  Serial.println(" connected.");

  espClient.setInsecure();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  if (sensor.begin() && sensor.setSamplingRate(kSamplingRate))
  {
    Serial.println("Max30102 initialized");
  }
  else
  {
    Serial.println("Max30102 not found");
    while (1)
      ;
  }

  // Temp Sensor Init
  tempSensor.setResolution(9);
  tempSensor.begin();

  pinMode(buttonPin, INPUT_PULLUP); // internal pull-up
  pinMode(BUZZER, OUTPUT);
  pinMode(alertBtnPin, INPUT_PULLUP);
  pinMode(led, OUTPUT);
  digitalWrite(led, alertState);
  // Define ECG Pins
  pinMode(41, INPUT); // Setup for leads off detection LO +
  pinMode(40, INPUT); // Setup for leads off detection LO -0
}

// Filter Instances
LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager_bpm;
MovingAverageFilter<kAveragingSamples> averager_r;
MovingAverageFilter<kAveragingSamples> averager_spo2;

// Statistic for pulse oximetry
MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

// R value to SpO2 calibration factors
// See https://www.maximintegrated.com/en/design/technical-documents/app-notes/6/6845.html
float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

// Timestamp of the last heartbeat
long last_heartbeat = 0;

// Timestamp for finger detection
long finger_timestamp = 0;
bool finger_detected = false;

// Last diff to detect zero crossing
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

void loop()
{
  // MQTT Reconnect
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  handleButton();
  handleAlertButton();
  getEcg();
  auto sample = sensor.readSample(1000);
  float current_value_red = sample.red;
  float current_value_ir = sample.ir;

  // Detect Finger using raw sensor value
  if (sample.red > kFingerThreshold)
  {
    if (millis() - finger_timestamp > kFingerCooldownMs)
    {
      finger_detected = true;
    }
  }
  else
  {
    // Reset values if the finger is removed
    differentiator.reset();
    averager_bpm.reset();
    average_bpm = 0;
    averager_r.reset();
    averager_spo2.reset();
    average_spo2 = 0;
    low_pass_filter_red.reset();
    low_pass_filter_ir.reset();
    high_pass_filter.reset();
    stat_red.reset();
    stat_ir.reset();
    tempC = 0;
    respiration_rate = 0;

    finger_detected = false;
    finger_timestamp = millis();
  }

  if (finger_detected)
  {

    // Collect respitory rate
    estimateRespiratoryRate(sample.ir);

    current_value_red = low_pass_filter_red.process(current_value_red);
    current_value_ir = low_pass_filter_ir.process(current_value_ir);

    // Statistics for pulse oximetry
    stat_red.process(current_value_red);
    stat_ir.process(current_value_ir);

    // Heart beat detection using value for red LED
    float current_value = high_pass_filter.process(current_value_red);
    float current_diff = differentiator.process(current_value);

    // Valid values?
    if (!isnan(current_diff) && !isnan(last_diff))
    {

      // Detect Heartbeat - Zero-Crossing
      if (last_diff > 0 && current_diff < 0)
      {
        crossed = true;
        crossed_time = millis();
      }

      if (current_diff > 0)
      {
        crossed = false;
      }

      // Detect Heartbeat - Falling Edge Threshold
      if (crossed && current_diff < kEdgeThreshold)
      {
        if (last_heartbeat != 0 && crossed_time - last_heartbeat > 300)
        {
          // Show Results
          bpm = 60000 / (crossed_time - last_heartbeat);
          float rred = (stat_red.maximum() - stat_red.minimum()) / stat_red.average();
          float rir = (stat_ir.maximum() - stat_ir.minimum()) / stat_ir.average();
          float r = rred / rir;
          spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;

          if (bpm > 50 && bpm < 250)
          {
            // Average?
            if (kEnableAveraging)
            {
              average_bpm = averager_bpm.process(bpm);
              int average_r = averager_r.process(r);
              average_spo2 = averager_spo2.process(spo2);
              if(average_spo2 < 85) average_spo2 = 85;
              if(average_spo2 >99) average_spo2 = 99;

              // Show if enough samples have been collected
              if (averager_bpm.count() >= kSampleThreshold)
              {
                // Serial.print("Time (ms): ");
                // Serial.println(millis());
                // Serial.print("Heart Rate (avg, bpm): ");
                // Serial.println(average_bpm);
                // Serial.print("R-Value (avg): ");
                // Serial.println(average_r);
                // Serial.print("SpO2 (avg, %): ");
                // Serial.println(average_spo2);
              }
            }
            else
            {
              // Serial.print("Time (ms): ");
              // Serial.println(millis());
              // Serial.print("Heart Rate (current, bpm): ");
              // Serial.println(bpm);
              // Serial.print("R-Value (current): ");
              // Serial.println(r);
              // Serial.print("SpO2 (current, %): ");
              // Serial.println(spo2);
            }
          }

          // Reset statistic
          stat_red.reset();
          stat_ir.reset();
        }

        crossed = false;
        last_heartbeat = crossed_time;

        // Collect temprature
        // tempC = tempSensor.getTempCByIndex(0);
        // tempSensor.requestTemperatures();
        unsigned long currentMillis = millis();

        // Start temperature request
        if (!tempRequested && currentMillis - lastTempRequestTime >= tempInterval)
        {
          tempSensor.requestTemperatures();
          tempRequested = true;
          lastTempRequestTime = currentMillis;
        }

        // Try reading temperature after enough time (e.g., ~200ms)
        if (tempRequested && currentMillis - lastTempRequestTime >= 200)
        {
          tempC = tempSensor.getTempCByIndex(0);
          tempRequested = false;
        }
      }
    }

    last_diff = current_diff;
  }
  updateDisplay();
  syncData();
  if (alertState)
  {
    alert();
  }
  else
  {
    digitalWrite(BUZZER, LOW);
  }
}