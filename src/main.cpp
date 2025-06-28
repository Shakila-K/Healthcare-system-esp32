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

int BUZZER = 32;          // Buzzer connected to GPIO 32
const int buttonPin = 13; // Push button connected to GPIO 13

// Push button related variables
bool buttonState = HIGH;
bool lastButtonReading = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 50;

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
        currentDisplayIndex = (currentDisplayIndex + 1) % 4;
        beepBuzzer();
      }
    }
  }
  lastButtonReading = reading;
}

void updateDisplay()
{

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  switch (currentDisplayIndex)
  {
  case 0:
    display.println("Temperature");
    break;
  case 1:
    display.println("Heart Rate");
    break;
  case 2:
    display.println("SpO2 Level");
    break;
  case 3:
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
    display.print("%");
    break;

  case 3:
    display.setCursor(0, 32);
    display.setTextSize(2);
    display.print(ecg);
    break;
  }

  display.display();

  Serial.print(F("Temp= "));
  Serial.print(tempC);

  Serial.print(F(", HR= "));
  Serial.print(bpm);

  Serial.print(F(", SPO2= "));
  Serial.print(spo2);

  Serial.print(F(",Avg HR= "));
  Serial.print(average_bpm);

  Serial.print(F(",Avg SPO2= "));
  Serial.print(average_spo2);

  Serial.print(F(", ECG= "));
  Serial.println(ecg, DEC);
}

void setup()
{
  Serial.begin(115200);

  if (sensor.begin() && sensor.setSamplingRate(kSamplingRate))
  {
    Serial.println("Sensor initialized");
  }
  else
  {
    Serial.println("Sensor not found");
    while (1)
      ;
  }

  // Temp Sensor Init
  tempSensor.setResolution(9);
  tempSensor.begin();

  pinMode(buttonPin, INPUT_PULLUP); // internal pull-up
  pinMode(BUZZER, OUTPUT);
  // Define ECG Pins
  pinMode(41, INPUT); // Setup for leads off detection LO +
  pinMode(40, INPUT); // Setup for leads off detection LO -0

  // Display Init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("SSD1306 allocation failed");
    for (;;)
      ;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.display();
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
  handleButton();
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
    averager_r.reset();
    averager_spo2.reset();
    low_pass_filter_red.reset();
    low_pass_filter_ir.reset();
    high_pass_filter.reset();
    stat_red.reset();
    stat_ir.reset();

    finger_detected = false;
    finger_timestamp = millis();
  }

  if (finger_detected)
  {
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
}