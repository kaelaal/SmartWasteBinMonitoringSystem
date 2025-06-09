#define BLYNK_TEMPLATE_ID "TMPL6meIntBrB"
#define BLYNK_TEMPLATE_NAME "SmartTrashbin"
#define BLYNK_AUTH_TOKEN "QtMQBW6DBr0j--J86PX206Mr0W4Itvs_"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HX711.h>
#include <math.h>

// OLED settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// WiFi credentials
char ssid[] = "";
char pass[] = "";

// HX711 load cell pins and object
#define HX711_DOUT D6
#define HX711_SCK D5
HX711 LoadCell;

// Ultrasonic sensor pins
#define TRIG_PIN D4
#define ECHO_PIN D7

// Buzzer pin
#define BUZZER_PIN D8
bool buzzerOn = false;

// Bin height thresholds
const float binMaxHeight = 17.0;
const float binMinHeight = 5.0;

// Low-pass filter variables for weight
const float T = 1.0;
const float tau = 0.5;
float filteredWeight = 0.0;
bool firstWeightSample = true;

// Notification cooldown (in milliseconds)
unsigned long lastWeightAlertTime = 0;
unsigned long lastFullAlertTime = 0;
const unsigned long alertCooldown = 5000; // 5 seconds for testing

// Moving average for ultrasonic
float getStableDistanceCm(int samples = 5) {
  float total = 0;
  int validSamples = 0;

  for (int i = 0; i < samples; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout in µs
    float distance = duration * 0.0343 / 2.0;

    // Only include valid distances
    if (distance > 2 && distance < 400) {
      total += distance;
      validSamples++;
    }

    delay(50); // short delay to avoid echo interference
  }

  if (validSamples == 0) return 0;
  return total / validSamples;
}

void readSensorsAndUpdate() {
  bool alertTriggered = false;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // --- Read and filter weight ---
  if (LoadCell.is_ready()) {
    float rawWeight = LoadCell.get_units();

    if (firstWeightSample) {
      filteredWeight = rawWeight;
      firstWeightSample = false;
    } else {
      float alpha = T / (tau + T);
      filteredWeight = filteredWeight + alpha * (rawWeight - filteredWeight);
    }

    float averageWeight = abs(filteredWeight) < 5.0 ? 0.0 : filteredWeight;

    Serial.print("Weight (filtered): ");
    Serial.print(averageWeight, 2);
    Serial.println(" g");

    display.setCursor(0, 0);
    display.print("Weight: ");
    display.print(averageWeight, 1);
    display.print("g");

    Blynk.virtualWrite(V1, averageWeight);

    if (averageWeight >= 750.0) {
      Serial.println("Weight exceeds 750g! Triggering weight_alert...");
      alertTriggered = true;
      if (millis() - lastWeightAlertTime > alertCooldown) {
        Blynk.logEvent("weight_alert", "Trash bin is getting heavy!");
        lastWeightAlertTime = millis();
        delay(500);
      }
    }
  }

  // --- Read ultrasonic distance using moving average ---
  float distanceCm = getStableDistanceCm();

  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.println(" cm");

  display.setCursor(0, 16);
  display.print("Distance: ");
  display.print(distanceCm, 1);
  display.print("cm");

  Blynk.virtualWrite(V0, distanceCm);

  // --- Bin level calculation ---
  float binLevel = 0;
  if (distanceCm <= binMinHeight) {
    binLevel = 100;
  } else if (distanceCm >= binMaxHeight) {
    binLevel = 0;
  } else {
    binLevel = ((binMaxHeight - distanceCm) / (binMaxHeight - binMinHeight)) * 100.0;
  }

  Serial.print("Bin Level: ");
  Serial.print(binLevel);
  Serial.println(" %");

  display.setCursor(0, 32);
  display.print("Fill Level: ");
  display.print(binLevel, 1);
  display.print("%");

  Blynk.virtualWrite(V2, binLevel);

  if (distanceCm <= 5.0) {
    Serial.println("Distance is <= 5.0 cm! Triggering full alert...");
    alertTriggered = true;
    if (millis() - lastFullAlertTime > alertCooldown) {
      Blynk.logEvent("alert", "Trash bin is full!");
      lastFullAlertTime = millis();
      delay(500);
    }
  }

  // --- Buzzer control ---
  if (alertTriggered && !buzzerOn) {
    tone(BUZZER_PIN, 1000);
    buzzerOn = true;
  } else if (!alertTriggered && buzzerOn) {
    noTone(BUZZER_PIN);
    buzzerOn = false;
  }

  display.display();
}

void setup() {
  Serial.begin(9600);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Initializing...");
  display.display();

  LoadCell.begin(HX711_DOUT, HX711_SCK);
  LoadCell.set_scale(424.5575);
  LoadCell.tare();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
  Blynk.run();
  readSensorsAndUpdate();
  delay(2000);
}
