// Blynk project and authentication setup 
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

// HX711 load cell pins and object initialization 
#define HX711_DOUT D6 
#define HX711_SCK D5 
HX711 LoadCell; 

// Ultrasonic sensor pins 
#define TRIG_PIN D4 
#define ECHO_PIN D7

// Buzzer pin and buzzer state variable 
#define BUZZER_PIN D8 
bool buzzerOn = false; 

// LCD object for 16x2 display via I2C expander 
hd44780_I2Cexp lcd; 

// Bin height thresholds (in cm) 
const float binMaxHeight = 30.0; 
const float binMinHeight = 10.0; 

// --- Low-pass filter variables for load cell ---
const float T = 1.0;       // sample period in seconds (loop delay 1000ms)
const float tau = 0.5;     // filter time constant in seconds (adjust for smoothing)
float filteredWeight = 0.0; 
bool firstWeightSample = true; 

// Function to read sensors and update display, Blynk, and buzzer 
void readSensorsAndUpdate() { 
  bool alertTriggered = false; 

  // Read weight from load cell if ready 
  if (LoadCell.is_ready()) { 
    float rawWeight = LoadCell.get_units(); 

    // --- LOW-PASS FILTER IMPLEMENTATION ---
    if (firstWeightSample) {
      filteredWeight = rawWeight;  // initialize filtered value on first sample
      firstWeightSample = false;
    } else {
      float alpha = T / (tau + T);
      filteredWeight = filteredWeight + alpha * (rawWeight - filteredWeight);
    }
    float averageWeight = filteredWeight;
    // --- END LOW-PASS FILTER ---

    // Filter out very small weight values (noise) 
    if (abs(averageWeight) < 5.0) averageWeight = 0.0; 

    // Print average weight to serial and display on LCD 
    Serial.print("Weight (filtered): "); 
    Serial.print(averageWeight, 2); 
    Serial.println(" g"); 
    lcd.setCursor(0, 0); 
    lcd.print("Wt:"); 
    lcd.print(averageWeight, 1); 
    lcd.print("g "); 

    // Send weight value to Blynk app virtual pin V1 
    Blynk.virtualWrite(V1, averageWeight); 

    // Trigger alert if weight exceeds threshold 
    if (averageWeight >= 100.0) {
      alertTriggered = true; 
    } 
  } 

  // Ultrasonic sensor trigger pulse 
  digitalWrite(TRIG_PIN, LOW); 
  delayMicroseconds(2); 
  digitalWrite(TRIG_PIN, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(TRIG_PIN, LOW); 

  // Measure echo pulse duration and calculate distance in cm 
  long duration = pulseIn(ECHO_PIN, HIGH); 
  float distanceCm = duration * 0.0343 / 2.0; 

  // Print distance to serial and display on LCD 
  Serial.print("Distance: "); 
  Serial.print(distanceCm); 
  Serial.println(" cm"); 
  lcd.setCursor(0, 1); 
  lcd.print("Dst:"); 
  lcd.print(distanceCm, 1); 
  lcd.print("cm "); 

  // Send distance value to Blynk app virtual pin V0 
  Blynk.virtualWrite(V0, distanceCm); 

  // Calculate bin fill level percentage based on distance
  float binLevel = 0; 
  if (distanceCm <= binMinHeight) { 
    binLevel = 100; 
  } else if (distanceCm >= binMaxHeight) { 
    binLevel = 0; 
  } else { 
    binLevel = ((binMaxHeight - distanceCm) / (binMaxHeight - binMinHeight)) * 100.0; 
  } 

  // Print bin level and send to Blynk virtual pin V2 
  Serial.print("Bin Level: "); 
  Serial.print(binLevel); 
  Serial.println(" %"); 
  Blynk.virtualWrite(V2, binLevel); 

  // Trigger alert if bin is nearly full (distance <= 5 cm) 
  if (distanceCm <= 10.0) { 
    alertTriggered = true; 
  } 

  // Control buzzer based on alert state 
  if (alertTriggered && !buzzerOn) { 
    tone(BUZZER_PIN, 1000); // Turn buzzer on at 1000 Hz 
    buzzerOn = true; 
  } else if (!alertTriggered && buzzerOn) { 
    noTone(BUZZER_PIN); // Turn buzzer off 
    buzzerOn = false; 
  } 
} 

// Arduino setup function 
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

  // Initialize load cell 
  LoadCell.begin(HX711_DOUT, HX711_SCK); 
  LoadCell.set_scale(696.0); 
  LoadCell.tare(); 

  // Set ultrasonic sensor and buzzer pin modes 
  pinMode(TRIG_PIN, OUTPUT); 
  pinMode(ECHO_PIN, INPUT); 
  pinMode(BUZZER_PIN, OUTPUT); 
} 

// Main loop 
void loop() {
  Blynk.run(); 
  readSensorsAndUpdate(); 
  delay(1000); 
}
