#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <RH_RF95.h>
#include <RTCZero.h> // M0 Real Time Clock

//sd
File gpsDataFile; //file to store gps data

const int chipSelect = 10;
//rfm95x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0 //LoRa radio module frequency, must match RX's freq!

RH_RF95 rf95(RFM95_CS, RFM95_INT); //create an instance of the radio driver class
long lastSendTime = 0; // last send time
int interval = 10000; // ping interval in ms
int16_t packetnum = 0; //packet counter
//payload split variables
#define maxWordCount 10
char *Words[maxWordCount];

//time
RTCZero rtc;
/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 54;
const byte hours = 18;

/* Change these values to set the current initial date */
const byte day = 25;
const byte month = 6;
const byte year = 21;

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
  rtc.begin(); // initialize RTC

  // Set the time
  rtc.setHours(hours);
  rtc.setMinutes(minutes);
  rtc.setSeconds(seconds);

  // Set the date
  rtc.setDay(day);
  rtc.setMonth(month);
  rtc.setYear(year);
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
        char * incoming = (char*)buf;
        Serial.print("Got reply: ");
        Serial.println((char*)buf);
        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);
        //decode received data and store to sd card
        decodeData(incoming);//pass the received buffer as a parameter
        //writeGpsData(latitude, longitude);
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
  gpsDataFile.print("dateTime,latitude,longitude,alt,speed,course,hdop,satellites,batt,RSSI");
  gpsDataFile.println();
  gpsDataFile.close();
}

void writeTxData(char * latitude, char * longitude, char * alt, char * speedval, char * courseval, char * hdopval, char * satelliteval, char * battval, int rssival) {
  //open the file where the received gps data is to be written to
  gpsDataFile = SD.open("data.csv", FILE_WRITE);
  if (gpsDataFile) {
    //store the datetime from the rtc
    /* Change these values to set the current initial time */
    byte seconds = rtc.getSeconds();
    byte minutes = rtc.getMinutes();
    byte hours = rtc.getHours();
    byte day = rtc.getDay();
    byte month = rtc.getMonth();
    byte year = rtc.getYear();
    gpsDataFile.print(hours);
    gpsDataFile.print(":");
    gpsDataFile.print(minutes);
    gpsDataFile.print(":");
    gpsDataFile.print(seconds);
    gpsDataFile.print(" ");
    gpsDataFile.print(year);
    gpsDataFile.print('/');
    gpsDataFile.print(month);
    gpsDataFile.print('/');
    gpsDataFile.print(day);
    gpsDataFile.print(", ");
    gpsDataFile.print(latitude);
    gpsDataFile.print(", ");
    gpsDataFile.print(longitude);
    gpsDataFile.print(", ");
    gpsDataFile.print(alt);
    gpsDataFile.print(", ");
    gpsDataFile.print(speedval);
    gpsDataFile.print(", ");
    gpsDataFile.print(courseval);
    gpsDataFile.print(", ");
    gpsDataFile.print(hdopval);
    gpsDataFile.print(", ");
    gpsDataFile.print(satelliteval);
    gpsDataFile.print(", ");
    gpsDataFile.print(battval);
    gpsDataFile.print(", ");
    gpsDataFile.println(rssival);
    gpsDataFile.close();
    // print to the serial port too:
    Serial.println("data stored to sd card");
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("could not open datalog file");
  }
}


char * splitPayload(char * str) {
  //convert byte array to string
  char payloadStr[sizeof(str)+1];
  memcpy(payloadStr,str, sizeof(str));
  payloadStr[sizeof(str)]  = 0; //null termination
  //split the string via strtok
  int wordCount = 0;
  char *item = strtok(payloadStr,",");//get the first word
  while(item != NULL){//loop through all the comma seperated variables
    Words[wordCount] = item;
    item = strtok (NULL, " ,"); //getting subsequence word
    Serial.println(Words[wordCount]);
    wordCount++;
    //delay(100);
  }
  return 0;
}

void decodeData(char * buf) {
  //decode the received encrypted data
  Serial.println((char*)buf);//print the received message
  //decrypt
  //byte  payload = decryptData(128, buf);
  //split the comma seperated values of the payload
  splitPayload(buf);
  //store the decoded data to sd card
  //get the RSSI
  int rssiVal = rf95.lastRssi();
  //store the data to the sdcard
  writeTxData(Words[0],Words[1],Words[2],Words[4],Words[4],Words[5],Words[6],Words[7],rssiVal);
  //clear payload array
  memset(Words,0,sizeof(Words));
}
