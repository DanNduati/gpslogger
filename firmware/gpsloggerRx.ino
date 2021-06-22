#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <RH_RF95.h>

//sd
File gpsDataFile; //file to store gps data

const int chipSelect = 4;
//rfm95x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0 //LoRa radio module frequency, must match RX's freq!
RH_RF95 rf95(RFM95_CS, RFM95_INT); //create an instance of the radio driver class
long lastSendTime = 0; // last send time
int interval = 10000; // ping interval in ms
int16_t packetnum = 0; //packet counter

void setup() {
  Serial.begin(115200);
  Serial.println("GPSlogger RX");
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false); //set receiver power to 23 dBm
  //initialize the sd card
  Serial.print("Initializing SD card...");
  // check if the SD card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("card failed or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  writeDataHeader(); //table header for easier data readability
}

void loop() {
  if (millis() - lastSendTime > interval) {
    Serial.println("Sending ping to logger");
    char packet[10] = "Ping";
    packetnum ++;
    //send the ping
    rf95.send((uint8_t *)packet, 10);
    Serial.println("Waiting for packet to complete...");
    delay(10);
    rf95.waitPacketSent();
    //wait for a reply from the logger
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    Serial.println("Waiting for reply...");
    if (rf95.waitAvailableTimeout(1000))
    {
      // Should be a reply message for us now
      if (rf95.recv(buf, &len))
      {
        char * incoming = ((char*)buf);
        Serial.print("Got reply: ");
        Serial.println((char*)buf);
        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);
        //decode received data and store to sd card
        float longitude = 0;//to be changed once max pushes logger code
        float latitude = 0;//to be changed once max pushes logger code
        writeGpsData(latitude,longitude);
      }
      else
      {
        Serial.println("Receive failed");
      }
    }
    else
    {
      Serial.println("No reply, is there a listener around?");
    }
  }
}

void writeDataHeader() {
  gpsDataFile = SD.open("data.csv", FILE_WRITE);
  gpsDataFile.print("latitude,longitude");
  gpsDataFile.println();
  gpsDataFile.close();
}

void writeGpsData(float latitude,float longitude) {
  //open the file where the received gps data is to be written to
  gpsDataFile = SD.open("data.csv", FILE_WRITE);
  if (gpsDataFile) {
    gpsDataFile.print(latitude);
    gpsDataFile.print(", ");
    gpsDataFile.println(longitude);
    gpsDataFile.close();
    // print to the serial port too:
    Serial.println("data stored to sd card");
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("could not open datalog file");
  }
}


