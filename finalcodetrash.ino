#define BLYNK_TEMPLATE_ID "TMPL6meIntBrB" 
#define BLYNK_TEMPLATE_NAME "SmartTrashbin"
#define BLYNK_AUTH_TOKEN "QtMQBW6DBr0j--J86PX206Mr0W4Itvs_"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include <math.h>
#include <HX711.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

// WiFi credentials
char ssid[] = "";      
char pass[] = "";   

// Load Cell Setup
#define HX711_DOUT D6
#define HX711_SCK D5
HX711 LoadCell;

// Ultrasonic Sensor
#define TRIG_PIN D4
#define ECHO_PIN D7

// Buzzer
#define BUZZER_PIN D8
bool buzzerOn = false;

// LCD using HD44780
hd44780_I2Cexp lcd;

// Moving Average
const int bufferSize = 3;
float weightBuffer[bufferSize];
int bufferIndex = 0;
bool bufferFilled = false;

// DFT Sampling
#define SAMPLE_SIZE 64
float weightSamples[SAMPLE_SIZE];
int sampleIndex = 0;

// Bin dimensions (adjust as needed)
const float binMaxHeight = 30.0; // in cm (distance when empty)
const float binMinHeight = 5.0;  // in cm (distance when full)

void performDFT(float* data, int N) {
  Serial.println("DFT Magnitude Spectrum:");
  for (int k = 0; k < N / 2; k++) {
    float real = 0;
    float imag = 0;
    for (int n = 0; n < N; n++) {
      float angle = 2 * PI * k * n / N;
      real += data[n] * cos(angle);
      imag -= data[n] * sin(angle);
    }
    float magnitude = sqrt(real * real + imag * imag);
    Serial.print("Freq bin ");
    Serial.print(k);
    Serial.print(": ");
    Serial.println(magnitude, 2);
  }
}

void readSensorsAndUpdate() {
  bool alertTriggered = false;

  // Load cell
  if (LoadCell.is_ready()) {
    float rawWeight = LoadCell.get_units();

    weightBuffer[bufferIndex] = rawWeight;
    bufferIndex = (bufferIndex + 1) % bufferSize;
    if (bufferIndex == 0) bufferFilled = true;

    float sum = 0;
    int count = bufferFilled ? bufferSize : bufferIndex;
    for (int i = 0; i < count; i++) {
      sum += weightBuffer[i];
    }
    float averageWeight = sum / count;
    if (abs(averageWeight) < 5.0) averageWeight = 0.0;

    Serial.print("Weight (avg): ");
    Serial.print(averageWeight, 2);
    Serial.println(" g");

    lcd.setCursor(0, 0);
    lcd.print("Wt:");
    lcd.print(averageWeight, 1);
    lcd.print("g     ");

    Blynk.virtualWrite(V1, averageWeight);

    if (averageWeight >= 100.0) {
      alertTriggered = true;
    }

    weightSamples[sampleIndex] = rawWeight;
    sampleIndex++;
    if (sampleIndex >= SAMPLE_SIZE) {
      performDFT(weightSamples, SAMPLE_SIZE);
      sampleIndex = 0;
    }
  }

  // Ultrasonic sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  float distanceCm = duration * 0.0343 / 2.0;

  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.println(" cm");

  lcd.setCursor(0, 1);
  lcd.print("Dst:");
  lcd.print(distanceCm, 1);
  lcd.print("cm     ");

  Blynk.virtualWrite(V0, distanceCm);

  // Calculate bin level (%)
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

  Blynk.virtualWrite(V2, binLevel);

  if (distanceCm <= 5.0) {
    alertTriggered = true;
  }

  if (alertTriggered && !buzzerOn) {
    tone(BUZZER_PIN, 1000);
    buzzerOn = true;
  } else if (!alertTriggered && buzzerOn) {
    noTone(BUZZER_PIN);
    buzzerOn = false;
  }
}

void setup() {
  Serial.begin(9600);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  int status = lcd.begin(16, 2);
  if (status) {
    Serial.print("LCD init failed with status: ");
    Serial.println(status);
  } else {
    lcd.backlight();
    lcd.print("Initializing...");
  }

  LoadCell.begin(HX711_DOUT, HX711_SCK);
  LoadCell.set_scale(-696.0); // Replace with your calibration
  LoadCell.tare();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
  Blynk.run();
  readSensorsAndUpdate();
  delay(1000);
}
