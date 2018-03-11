#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"'
#include <TinyGPS++.h>
#include <LoRa.h>
#include <ArduinoJson.h>

static const uint32_t GPSBaud = 9600;

// WIFI_LoRa_32 ports
// GPIO5  -- SX1278's SCK
// GPIO19 -- SX1278's MISO
// GPIO27 -- SX1278's MOSI
// GPIO18 -- SX1278's CS
// GPIO14 -- SX1278's RESET
// GPIO26 -- SX1278's IRQ(Interrupt Request)

//GPIO16 OLED RESET


#define SS      18
#define RST     14
#define DI0     26

#define STARTPIN 25
#define BUZZERPIN 13
#define ZEROPIN 2


#define spreadingFactor 9
#define SignalBandwidth 125E3
#define preambleLength 8
#define codingRateDenominator 8

#define FREQUENCY 868.9E6

#define TIMEOUT 2000
#define DELAY_MS 400

// The TinyGPS++ object
TinyGPSPlus gps;
int rssi;
int performance;
int power = 7;
float average;
float zero = 0;

// SSD1306  display(0x3c, 21, 22); // TTGOV2
SSD1306  display(0x3c, 4, 15); // TTGo V1

// The serial connection to the GPS device
HardwareSerial GPSserial(1);

struct locStruct
{
  bool isValid;
  bool isUpdated;
  uint32_t age;
  double lat;
  double lng;
  double altitude;
  double hdop;
  int power;
  int rssi;
} sendLoc, recLoc;

struct GPStimeStruct
{
  bool isValid;
  bool isUpdated;
  uint32_t age;
  uint32_t value;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t centisecond;
} GPStime;


void initDisplay() {
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_24);
  display.setColor(WHITE);
}

void getPosition() {

  unsigned long GPSentry = millis();
  while (GPSserial.available() > 0)
    if (gps.encode(GPSserial.read()))
      if (millis() > GPSentry + 5000 && gps.charsProcessed() < 10)
      {
        Serial.println(F("No GPS detected: check wiring."));
      }
  if (gps.location.isValid())
  {
    if (gps.location.isValid()) {
      sendLoc.isUpdated = (gps.location.isUpdated());
      sendLoc.lat = gps.location.lat();
      sendLoc.lng = gps.location.lng();
      sendLoc.altitude = gps.altitude.meters();
      sendLoc.hdop = gps.hdop.value();
    }
    if (gps.time.isValid()) {
      GPStime.year = gps.date.year();
      GPStime.month = gps.date.month();
      GPStime.day = gps.date.day();
      GPStime.hour = gps.time.hour();
      GPStime.minute = gps.time.minute();
      GPStime.second = gps.time.second();
      GPStime.centisecond = gps.time.centisecond();
    }
  }
}


bool sendMessage() {
  bool success = false;
  char json[500];
  StaticJsonBuffer<500> jsonBuffer;
  JsonObject& sendMessage = jsonBuffer.createObject();
  sendMessage["freq"] = FREQUENCY;
  sendMessage["sf"] = spreadingFactor;
  sendMessage["power"] = power;
  sendMessage["rLat"] = "";
  sendMessage["rLng"] = "";
  sendMessage["rAlt"] = "";
  sendMessage["rHdop"] = "";
  sendMessage["RSSI"] = "";

  // Send Packet
  String sMessage;
  Serial.println("Send");
  sendMessage.printTo(sMessage);
  LoRa.setTxPower(power);
  LoRa.beginPacket();
  LoRa.print(sMessage);
  LoRa.endPacket();

  // Receive Message
  unsigned long entry = millis();
  Serial.println("wait for receive");
  while (LoRa.parsePacket() == 0 && (millis() < entry + TIMEOUT));
  Serial.println("receive");
  if (millis() < entry + TIMEOUT - 1) {
    // received a packet
    json[0] = '\0';
    while (LoRa.available()) {
      byte hi = strlen(json);
      json[hi] = (char)LoRa.read();
      json[hi + 1] = '\0';
    }
    JsonObject& recMessage = jsonBuffer.parse(json);
    if (recMessage.success()) {
      recLoc.lat = recMessage["rLat"];
      recLoc.lng = recMessage["rLng"];
      recLoc.altitude = recMessage["rAlt"];
      recLoc.hdop = recMessage["rHdop"];
      power = recMessage["power"];
      rssi = recMessage["RSSI"];
      Serial.print("Power: ");
      Serial.print(power);
      Serial.print(" RSSI: ");
      Serial.print(rssi);
      Serial.print(" Performance: ");
      performance = (rssi - power);
      Serial.println(performance);
      Serial.print(" Relative Performance: ");
      Serial.println(performance - zero);

      success = true;
    } else  Serial.println("parseObject() failed");
  } else Serial.println("Timeout");
  return success;
}

double calculateDistance(double senderLat, double senderLng, double receiverLat, double receiverLng) {
  double distanceKm =
    gps.distanceBetween(
      receiverLat,
      receiverLng,
      senderLat,
      senderLng);
  /*
    Serial.println("Coordinates");
    Serial.println(receiverLat);
    Serial.println(receiverLng);
    Serial.println(senderLat);
    Serial.println(senderLng);
    Serial.println();
  */
  return distanceKm;
}

double calculateCourse(double senderLat, double senderLng, double receiverLat, double receiverLng) {
  double courseTo =
    gps.courseTo(
      receiverLat,
      receiverLng,
      senderLat,
      senderLng);
  return courseTo;
}
void initLoRa() {

  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(FREQUENCY)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.print("FREQUENCY ");
  Serial.println(FREQUENCY);
  LoRa.setTxPower(1);
  Serial.print("LoRa Spreading Factor: ");
  Serial.println(spreadingFactor);
  LoRa.setSpreadingFactor(spreadingFactor);
  Serial.print("LoRa Signal Bandwidth: ");
  Serial.println(SignalBandwidth);
  LoRa.setSignalBandwidth(SignalBandwidth);
  Serial.println("LoRa Initial OK!");
}

void buzz(int duration, int number) {
  for (int i = 0; i < number; i++) {
    digitalWrite(BUZZERPIN, 1);
    delay(duration);
    digitalWrite(BUZZERPIN, 0);
  }
}

void setup()
{
  Serial.begin(115200);
  GPSserial.begin(GPSBaud);
  pinMode(16, OUTPUT);
  digitalWrite(16, HIGH);
  initDisplay();
  pinMode(STARTPIN, INPUT_PULLUP);
  pinMode(ZEROPIN, INPUT_PULLUP);
  pinMode(BUZZERPIN, OUTPUT);
  digitalWrite(BUZZERPIN, 0);
  getPosition();
  initLoRa();
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_24);
  display.drawString(64, 0, "Press");
  display.drawString(64, 35, "Button");
  display.display();
}

void loop()
{
  Serial.println("Press button");
  while (digitalRead(STARTPIN) > 0) { // Wait for start
  }
  Serial.println("waiting");
  display.clear();
  display.setColor(WHITE);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_24);
  display.drawString(64, 15, "Waiting");
  display.display();
  delay(4000);
  buzz(100, 1);
  Serial.println("Start ");
  float sum = 0;
  for (int i = 0; i < 5; i++) {
    while (!sendMessage()) {
      Serial.println("Failure");
      delay(DELAY_MS);
      // buzz(500, 1);
    }
    sum = sum + performance;
    display.clear();
    display.setColor(WHITE);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_24);
    display.drawString(0, 0, "RSSI: " + String(rssi));
    display.drawString(0, 35, "IND: " + String(performance - zero));
    display.display();
    delay(DELAY_MS);
  }
  buzz(100, 3);
  average = (sum / 5.0);
  Serial.print("Average RSSI ");
  Serial.println(average);
  double distance = calculateDistance(sendLoc.lat, sendLoc.lng, recLoc.lat, recLoc.lng);
  //    Serial.print("Distance ");
  //    Serial.println(distance);
  double direction =  calculateCourse(sendLoc.lat, sendLoc.lng, recLoc.lat, recLoc.lng);
  //   Serial.print("Direction ");
  //   Serial.println(direction);

  // dtostrf(distance, 7, 2, line1);
  // dtostrf(direction, 7, 0, line2);

  char line3[15];
  dtostrf(average-zero, 7, 1, line3);
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_24);
  display.drawString(0, 0, "AVG:" );
  display.drawString(50, 0, line3);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 35, "Button");
  display.display();

  while ((digitalRead(STARTPIN) == 1) &&  (digitalRead(ZEROPIN) == 1)) {
    // Serial.print(digitalRead(ZEROPIN));
    delay(50);
    if (digitalRead(ZEROPIN) == 0) {
      zero = average;
      Serial.print("Zero ");
      Serial.println(zero);
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_CENTER);
      display.setFont(ArialMT_Plain_24);
      display.drawString(64, 0, "Zero");
      display.drawString(64, 35, "Button");
      display.display();
      if (digitalRead(ZEROPIN) == 0);
    }
  }
}
