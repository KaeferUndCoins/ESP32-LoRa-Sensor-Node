#include <Arduino.h>
#include <SPI.h>
// GPS conected to
// RXD -> 17
// TXD -> 16
// GND -> GND
// VCC -> 5V
// PPS -> NC
#include <LoRa.h>
#include <TinyGPS++.h>

#define myID 1
//define the pins used by the transceiver module
#define ss 18
#define rst 14
#define dio0 2
#define ADC_1 36
#define ledpin 25
int counter = 0;

TinyGPSPlus gps;
void setup()
{
  //initialize Serial Monitor
  Serial.begin(9600);
  Serial2.begin(9600);
  while (!Serial);
  Serial.println("LoRa Sender");
  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  //replace the LoRa.begin(---E-) argument with your location's frequency
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  pinMode(ledpin, OUTPUT);
  int c = 0;
  while (!LoRa.begin(868E6))
  {
    Serial.print(".");
    c++;
    if (c >= 50)
    {
      c = 0;
      Serial.println(".");
    }
    delay(1000);
  }
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  digitalWrite(ledpin, HIGH);
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
  pinMode(ADC_1, INPUT);
}

int incomingByte = 0; // for incoming serial data
int sensIN;
int lastMsg;
int interval = 1000;
void loop()
{
  while (Serial2.available())
  {
    if (gps.encode(Serial2.read()))
      //  displayInfo();
      if (millis() > 5000 && gps.charsProcessed() < 10)
      {
        Serial.println(F("No GPS detected: check wiring."));
        //while (true);
      }
  }
  if (Serial.available() > 0)
  {
    // read the incoming byte:
    incomingByte = Serial.read();
    // say what you got:
    Serial.print(incomingByte, DEC);
    Serial.println(" on Serial 0");
  }
  sensIN = analogRead(ADC_1);

  if (millis() - lastMsg >= interval)
  {
    lastMsg = millis();
    Serial.print("Sending packet: ");
    Serial.println(counter);
    //Send LoRa packet to receiver
    LoRa.beginPacket();
    LoRa.print("MsgNr:");
    LoRa.print(counter);
    LoRa.print(" SenderID:");
    LoRa.print(myID);
    LoRa.print("|Payload: ");
    LoRa.print(sensIN);
    if (gps.date.isValid())
    {
      LoRa.print("|Date:");
      if (gps.date.day() < 10)
        LoRa.print("0");
      LoRa.print(gps.date.day());
      LoRa.print(".");
      if (gps.date.month() < 10)
        LoRa.print("0");
      LoRa.print(gps.date.month());
      LoRa.print(".");
      LoRa.print(gps.date.year());
    }
    if (gps.time.isValid())
    {
      LoRa.print("|Time:");
      if (gps.time.hour() < 10)
        LoRa.print("0");
      LoRa.print(gps.time.hour());
      LoRa.print(":");
      if (gps.time.minute() < 10)
        LoRa.print("0");
      LoRa.print(gps.time.minute());
      LoRa.print(":");
      if (gps.time.second() < 10)
        LoRa.print("0");
      LoRa.print(gps.time.second());
    }
    if (gps.location.isValid())
    {
      LoRa.print("|Loaction:");
      LoRa.print(gps.location.lat(),6);
      LoRa.print(",");
      LoRa.print(gps.location.lng(),6);
    }
    LoRa.endPacket();
    counter++;
    if (counter >= 10000)
    {
      counter = 0;
    }
  }
  int packetSize = LoRa.parsePacket();
  if (packetSize)
  {
    String LoRaData = "";
    // received a packet
   // Serial.print("Received packet '");
    // read packet
    while (LoRa.available())
    {
      LoRaData = LoRa.readString();
      Serial.print(LoRaData);
    }
    // print RSSI of packet
    Serial.print("' recieved from id: ");
    Serial.print(myID);
    Serial.print(" with RSSI ");
    int rssi_rx = LoRa.packetRssi();
    Serial.println(rssi_rx);
  }
}