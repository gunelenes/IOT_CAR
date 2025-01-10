#define BLYNK_TEMPLATE_ID "TMPL4RNir00W4"
#define BLYNK_TEMPLATE_NAME "IOT CAR APP"
#define BLYNK_AUTH_TOKEN "63v6OyI4BO0uFN6B_o1h-c7SMwX4Ovln"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include "HX711.h"
#include <Wire.h>
#include <MPU6050.h>

// Wi-Fi Ayarları
char ssid[] = "Eness";   // Wi-Fi ağınızın adı
char pass[] = "1234567890"; // Wi-Fi şifreniz

// HX711 pinleri
#define DT 4 // HX711 veri pini (DT)
#define SCK 5 // HX711 saat pini (SCK)
// Pin tanımlamaları
const int buttonPin = 23; // Düğme GPIO 23'e bağlı
const int ledPin = 2;     // LED GPIO 2'ye bağlı (dahili LED)

// HX711 nesnesi
HX711 scale;

// MPU6050 nesnesi
MPU6050 mpu;

// Blynk Virtual Pin Tanımları
#define VPIN_WEIGHT V0 // Ağırlık için Virtual Pin
#define VPIN_AX V2     // X ivmesi için Virtual Pin
#define VPIN_AY V3     // Y ivmesi için Virtual Pin
#define VPIN_AZ V4     // Z ivmesi için Virtual Pin
#define VPIN_GX V5
#define VPIN_TEMPERETURE V1
#define VPIN_LED V6
#define VPIN_LED_AX V7
#define VPIN_LED_AY V8
#define VPIN_LED_AZ V9
#define VPIN_LED_GX V10

// Başlangıç kalibrasyon faktörü
float calibration_factor = 5555; // İhtiyaca göre ayarlanacak

// MPU6050 Kalibrasyon için ortalama değerler ve sapmalar
float avgAx = 0, avgAy = 0, avgAz = 0;
float avgGx = 0, avgGy = 0, avgGz = 0;
float stdDevAx = 0, stdDevAy = 0, stdDevAz = 0;
float stdDevGx = 0, stdDevGy = 0, stdDevGz = 0;

// Low-pass filter değişkenleri
float alpha = 0.3; // Filtre katsayısı (0 ile 1 arasında)
float filteredAx = 0, filteredAy = 0, filteredAz = 0;

// İvme eşik değerleri
float thresholdX = 0, thresholdY = 0, thresholdZ = 0;

void setup() {
   pinMode(buttonPin, INPUT_PULLUP); // Dahili pull-up direnci aktif
  pinMode(ledPin, OUTPUT);          // LED çıkış olarak ayarlandı
  digitalWrite(ledPin, LOW);        // LED başlangıçta kapalı
  Serial.begin(115200);
  Wire.begin();
 // Blynk başlatma
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  // HX711 başlatma
  scale.begin(DT, SCK);
  Serial.println("HX711 Kalibrasyon Başlatılıyor...");
  delay(1000);

  if (!scale.is_ready()) {
    Serial.println("HX711 algılanamadı. Lütfen bağlantıları kontrol edin.");
    while (1); // Sonsuz döngü, bağlantı sorunu varsa durdur
  }

  scale.set_scale(calibration_factor); // Varsayılan ölçek faktörünü ayarla
  scale.tare(); // Yük hücresini sıfırla
  Serial.println("HX711 Başlatıldı ve Sıfırlandı.");

  // MPU6050 başlatma
  Serial.println("MPU6050 sensör başlatılıyor...");
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 başlatma başarısız!");
    Serial.println("Bağlantıları kontrol edin ve tekrar deneyin.");
    while (1);
  }

  delay(3000);
  Serial.println("MPU6050 başlatıldı.");

  // MPU6050 Kalibrasyon
  Serial.println("Kalibrasyon yapılıyor, lütfen 15 saniye bekleyin...");
  unsigned long startTime = millis();
  int sampleCount = 0;

  float sumAx = 0, sumAy = 0, sumAz = 0;
  float sumGx = 0, sumGy = 0, sumGz = 0;
  float sqSumAx = 0, sqSumAy = 0, sqSumAz = 0;
  float sqSumGx = 0, sqSumGy = 0, sqSumGz = 0;

  while (millis() - startTime < 15000) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);

    sumAx += ax;
    sumAy += ay;
    sumAz += az;
    sumGx += gx;
    sumGy += gy;
    sumGz += gz;

    sqSumAx += ax * ax;
    sqSumAy += ay * ay;
    sqSumAz += az * az;
    sqSumGx += gx * gx;
    sqSumGy += gy * gy;
    sqSumGz += gz * gz;

    sampleCount++;
    delay(100);
  }

  avgAx = sumAx / sampleCount;
  avgAy = sumAy / sampleCount;
  avgAz = sumAz / sampleCount;
  avgGx = sumGx / sampleCount;
  avgGy = sumGy / sampleCount;
  avgGz = sumGz / sampleCount;

  stdDevAx = sqrt((sqSumAx / sampleCount) - (avgAx * avgAx));
  stdDevAy = sqrt((sqSumAy / sampleCount) - (avgAy * avgAy));
  stdDevAz = sqrt((sqSumAz / sampleCount) - (avgAz * avgAz));
  stdDevGx = sqrt((sqSumGx / sampleCount) - (avgGx * avgGx));
  stdDevGy = sqrt((sqSumGy / sampleCount) - (avgGy * avgGy));
  stdDevGz = sqrt((sqSumGz / sampleCount) - (avgGz * avgGz));

  thresholdX = stdDevAx * 1;
  thresholdY = stdDevAy * 1;
  thresholdZ = stdDevAz * 0.4;

  Serial.println("Kalibrasyon tamamlandı.");
  Serial.println("Eşik Değerleri:");
  Serial.print("X: "); Serial.println(thresholdX);
  Serial.print("Y: "); Serial.println(thresholdY);
  Serial.print("Z: "); Serial.println(thresholdZ);

  delay(1000);
}

void ledControl(float value, int vPin) {
  if (value >= 50 && value < 100) {  
      Blynk.virtualWrite(vPin, 255);
      delay(500);
      Blynk.virtualWrite(vPin, 0);
      delay(500);   
  } else if (value >= 100 && value < 200) {
      Blynk.virtualWrite(vPin, 255);
      delay(1500);
      Blynk.virtualWrite(vPin, 0);
      delay(1500);
  } else if (value >= 200) {
    for (int i = 0; i < 5; i++) {
      Blynk.virtualWrite(vPin, 255);
      delay(1000);
      Blynk.virtualWrite(vPin, 0);
      delay(1000);
    }
  } else {
    Blynk.virtualWrite(vPin, 0);
  }
}

void loop() {
 Blynk.run(); // Blynk bağlantısını sürdür
 // Düğme durumunu oku
  int buttonState = digitalRead(buttonPin);

  // LED durumunu düğme durumuna göre ayarla
  if (buttonState == LOW) { // Düğme kapalı pozisyondaysa
    digitalWrite(ledPin, HIGH); // LED'i yak
    Serial.println("LED State: ON");
     Blynk.virtualWrite(VPIN_LED, 1);
  } else { // Düğme açık pozisyondaysa
    digitalWrite(ledPin, LOW); // LED'i söndür
    Serial.println("LED State: OFF");
     Blynk.virtualWrite(VPIN_LED, 0);
  }

  delay(10); // Küçük bir gecikme (stabilite için)


  // HX711 veri okuma
  float weight = 0;
  if (scale.is_ready()) {
    weight = scale.get_units(10);
    if (weight < 0) {
      weight = 0;
    } else if (weight > 200) {
      weight = 200;
    }
  } else {
    Serial.println("HX711 algılanamadı.");
  }

  // MPU6050 veri okuma
  unsigned long currentTime = millis();
  static unsigned long previousTime = 0;
  if (currentTime - previousTime >= 10) {
    previousTime = currentTime;

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);

    float calibratedAx = (ax - avgAx) / stdDevAx;
    float calibratedAy = (ay - avgAy) / stdDevAy;
    float calibratedAz = (az - avgAz) / stdDevAz;

    filteredAx = alpha * filteredAx + (1 - alpha) * calibratedAx;
    filteredAy = alpha * filteredAy + (1 - alpha) * calibratedAy;
    filteredAz = alpha * filteredAz + (1 - alpha) * calibratedAz;

    if (fabs(filteredAx) < thresholdX) filteredAx = 0;
    if (fabs(filteredAy) < thresholdY) filteredAy = 0;
    if (fabs(filteredAz) < thresholdZ) filteredAz = 0;

      
    
    float calibratedGx = (gx - avgGx) / stdDevGx;
// Sensör Verilerini Blynk'e Gönder
     Blynk.virtualWrite(VPIN_WEIGHT, weight);       // Ağırlık verisini V0'a gönder
     Blynk.virtualWrite(VPIN_AX, filteredAx);     // X ivmesini V1'e gönder
     Blynk.virtualWrite(VPIN_AY, filteredAy);     // Y ivmesini V2'ye gönder
     Blynk.virtualWrite(VPIN_AZ, filteredAz);     // Z ivmesini V3'e gönder
     Blynk.virtualWrite(VPIN_GX, calibratedGx);

     // LED kontrol fonksiyonları
     ledControl(filteredAx, VPIN_LED_AX);
     ledControl(filteredAy, VPIN_LED_AY);
     ledControl(filteredAz, VPIN_LED_AZ);

if (calibratedGx >= 600) {
    for (int i = 0; i < 5; i++) {
      Blynk.virtualWrite(VPIN_LED_GX, 255);
      delay(1000);
      Blynk.virtualWrite(VPIN_LED_GX, 0);
      delay(1000);
    }
  }else if (calibratedGx >= 100 && calibratedGx < 400) {
      Blynk.virtualWrite(VPIN_LED_GX, 255);
      delay(1500);
      Blynk.virtualWrite(VPIN_LED_GX, 0);
      delay(1500);
  } else {
    Blynk.virtualWrite(VPIN_LED_GX, 0);
  }
     
    // Serial Plotter çıktısı
    Serial.print(filteredAx); Serial.print(",");
    Serial.print(filteredAy); Serial.print(",");
    Serial.print(filteredAz); Serial.print(",");
    Serial.print(calibratedGx); Serial.print(",");
    Serial.println(weight);
  }

  // Seri port üzerinden yeni kalibrasyon faktörünü ayarla
  while (Serial.available()) {
    String input = Serial.readString();
    calibration_factor = input.toFloat();
    scale.set_scale(calibration_factor);

    Serial.print("Yeni Kalibrasyon Faktörü: ");
    Serial.println(calibration_factor);
  }

  delay(100); // Genel döngü gecikmesi
}
