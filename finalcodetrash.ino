#include <HX711_ADC.h>  // Library for the HX711 load cell amplifier

// --- Load Cell Setup ---
const int HX711_dout = 2;
const int HX711_sck = 3;
HX711_ADC LoadCell(HX711_dout, HX711_sck);
const int calVal_eepromAdress = 0;

// --- Ultrasonic Sensor Setup ---
const int trigPin = 9;
const int echoPin = 10;
long duration;
float distanceCm;

// --- Buzzer Setup ---
const int buzzerPin = 8;

// --- Moving Average for Load Cell ---
const int bufferSize = 3;
float weightBuffer[bufferSize];
int bufferIndex = 0;
bool bufferFilled = false;

// --- Fourier Series Sampling ---
#define SAMPLE_SIZE 64
float weightSamples[SAMPLE_SIZE];
unsigned long lastSampleTime = 0;
const int sampleInterval = 50;  // ms between samples (~20 Hz)
int sampleIndex = 0;

void setup() {
  Serial.begin(9600);

  // Initialize Load Cell
  LoadCell.begin();
  float calibrationValue = 696.0;
  unsigned long stabilizingtime = 500;
  boolean _tare = true;
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("LoadCell Timeout, check wiring!");
    while (1);
  } else {
    LoadCell.setCalFactor(calibrationValue);
    Serial.println("LoadCell ready.");
  }

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
}

void loop() {
  bool alertTriggered = false;

  // --- Sample Load Cell Data for Fourier Analysis ---
  if (millis() - lastSampleTime >= sampleInterval) {
    lastSampleTime = millis();

    if (LoadCell.update()) {
      float rawWeight = LoadCell.getData();

      // --- Moving Average ---
      weightBuffer[bufferIndex] = rawWeight;
      bufferIndex = (bufferIndex + 1) % bufferSize;
      if (bufferIndex == 0) bufferFilled = true;

      float sum = 0;
      int count = bufferFilled ? bufferSize : bufferIndex;
      for (int i = 0; i < count; i++) {
        sum += weightBuffer[i];
      }
      float averageWeight = sum / count;

      if (abs(averageWeight) < 5.0) {
        averageWeight = 0.0;
      }

      Serial.print("Weight (avg): ");
      Serial.print(averageWeight, 2);
      Serial.println(" g");

      if (averageWeight >= 100.0) {
        Serial.println("Weight over 100g!");
        alertTriggered = true;
      }

      // Store raw weight sample for DFT
      weightSamples[sampleIndex] = rawWeight;
      sampleIndex++;

      // Once we have enough samples, perform DFT
      if (sampleIndex >= SAMPLE_SIZE) {
        performDFT(weightSamples, SAMPLE_SIZE);
        sampleIndex = 0;
      }
    }
  }

  // --- Ultrasonic Sensor ---
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distanceCm = duration * 0.0343 / 2;

  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.println(" cm");

  if (distanceCm <= 5.0) {
    Serial.println("Object detected within 5 cm!");
    alertTriggered = true;
  }

  // --- Buzzer Logic ---
  if (alertTriggered) {
    tone(buzzerPin, 1000, 200);
  }

  delay(10);
}

// --- Discrete Fourier Transform (DFT) ---
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
