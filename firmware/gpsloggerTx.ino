#include <SPI.h>
#include <TinyGPS++.h>
#include <RH_RF95.h>
#include <FlashStorage.h>
#include <RTCZero.h>
#include <avr/dtostrf.h> 


//By default, Feather M0 uses SerialUSB and not Serial. Thus we define SerialUSB as Serial
#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

//Pin for battery monitoring
#define VBATPIN A7

//loRa pins
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 915.0/434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


int16_t packetnum = 0;  // packet counter, we increment per xmigpsSerialion

// Connect GPS Rx and Tx to hardware Tx and Rx respectively
int RXPin = 1;
int TXPin = 0;

//By default, Neo 6m has a baudrate of 9600
int GPSBaud = 9600;

//GPS serial
#define gpsSerial Serial1

//Create GPS Instance
TinyGPSPlus gps;

 // Create an rtc object
RTCZero rtc;

//GPS parameters
char gpsDate[32] = {"\0"};
char gpsTime[32] = {"\0"};
char longitude[15] = {"\0"};
char latitude[15] =  {"\0"};
char Altitude[10] = {"\0"};
char Speed[10] = {"\0"};
char Course[10] = {"\0"};
char Hdop[10] = {"\0"};
char Satellites[5] = {"\0"};


//Battery level threshold -  Battery level should not go below this value -Change it to most suitable value
const float bat_threshold = 3.0;

int BEACON_INTERVAL = 10;
int MOVEMENT_INTERVAL= 5;

volatile bool use_movement_interval = 0;
int tilt_sensor = A5;
volatile unsigned long last_movement_time =0;
int movements_during_movement_interval = 0; //number of movements occured during MOVEMENT_INTERVAL time... (if movement_interval was enabled)
volatile bool done_waiting = 0; //a variable to keep check of the state.
long iterationCounter = 0; // Increment each time a transmission is attempted


bool fixFound = false;
bool charsSeen = false;
int loop_step = 0;
unsigned long tnow;

typedef struct { // Define a struct to hold the flash variable(s)
  int PREFIX; // Flash storage prefix (0xB5); used to test if flash has been written to before 
  int INTERVAL; // Message interval in minutes
  int MOV_INTERVAL; //movement interval
  char *LONG; //longitude
  char *LAT; //latitude
  char *ALT; //altitude
  char *SPEED; //speed  
  char *COURSE; //course
  char *HDOP; //hdop
  char *SAT; //satellite
  float BAT; //battery level
  int CSUM; // Flash storage checksum; the modulo-256 sum of PREFIX and INTERVAL; used to check flash data integrity
} FlashVarsStruct;

FlashStorage(flashVarsMem, FlashVarsStruct); // Reserve memory for the flash variables
FlashVarsStruct flashVars; // Define the global to hold the variables

//function declarations
void transmit();
void GPSData();
float measureBattery();
bool batteryLevelOK();
void tilt_sensor_interrupt();
void alarmMatch();
bool listener();
void flashStorage();

void setup() 
{
  // Ensure serial flash is not interfering with radio communication on SPI bus
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }
  delay(100);
  // Start the GPS's serial
  gpsSerial.begin(GPSBaud);
  delay(100);
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  //initialize LoRa radio
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  pinMode(tilt_sensor,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(tilt_sensor), tilt_sensor_interrupt, CHANGE);

  
  // See if global variables have already been stored in flash
  // If they have, read them. If not, initialise them.
  flashVars = flashVarsMem.read(); // Read the flash memory
  int csum = flashVars.PREFIX + flashVars.INTERVAL + flashVars.MOV_INTERVAL; // Sum the prefix and data
  csum = csum & 0xff; // Limit checksum to 8-bits
  if ((flashVars.PREFIX == 0xB5) and (csum == flashVars.CSUM)) { // Check prefix and checksum match
    // Flash data is valid so update globals using the stored values
    BEACON_INTERVAL = flashVars.INTERVAL;
    //if MOV interval is 0...
    if(flashVars.MOV_INTERVAL == 0){
      
      //SET IT TO MOVEMENT_INTERVAL..
      flashVars.MOV_INTERVAL = MOVEMENT_INTERVAL;
      flashVars.LAT = (char*)malloc(15);
      strcpy(flashVars.LAT, latitude);
      flashVars.LONG = (char*)malloc(15);
      strcpy(flashVars.LONG, longitude);
      flashVars.ALT = (char*)malloc(15);
      strcpy(flashVars.ALT, Altitude);
      flashVars.SPEED = (char*)malloc(10);
      strcpy(flashVars.SPEED, Speed);
      flashVars.COURSE = (char*)malloc(10);
      strcpy(flashVars.COURSE, Course);
      flashVars.HDOP = (char*)malloc(10);
      strcpy(flashVars.HDOP, Hdop);
      flashVars.SAT = (char*)malloc(5);
      strcpy(flashVars.SAT, Satellites);
      flashVars.BAT = measureBattery();
      
      csum = flashVars.PREFIX + flashVars.INTERVAL + flashVars.MOV_INTERVAL; // Initialise the checksum
      csum = csum & 0xff;
      flashVars.CSUM = csum;

      //WRITING INTO THE FLASH MEMORY.
      flashVarsMem.write(flashVars); // Write the flash variables
      
      //free allocated memory
 
      free(flashVars.LAT);
      free(flashVars.LONG);
      free(flashVars.ALT);
      free(flashVars.SPEED);
      free(flashVars.COURSE);
      free(flashVars.HDOP);
      free(flashVars.SAT);
     
      //READING updated flash memory.
      flashVars = flashVarsMem.read(); // Read the flash memory
    }
    
    MOVEMENT_INTERVAL = flashVars.MOV_INTERVAL; //redundant
  }
  else {
    // Flash data is corrupt or hasn't been initialised so do that now
    flashVars.PREFIX = 0xB5; // Initialise the prefix
    flashVars.INTERVAL = BEACON_INTERVAL; // Initialise the beacon interval
    flashVars.MOV_INTERVAL = MOVEMENT_INTERVAL;
    flashVars.LAT = (char*)malloc(15);
    strcpy(flashVars.LAT, latitude);
    flashVars.LONG = (char*)malloc(15);
    strcpy(flashVars.LONG, longitude);
    flashVars.ALT = (char*)malloc(15);
    strcpy(flashVars.ALT, Altitude);
    flashVars.SPEED = (char*)malloc(10);
    strcpy(flashVars.SPEED, Speed);
    flashVars.COURSE = (char*)malloc(10);
    strcpy(flashVars.COURSE, Course);
    flashVars.HDOP = (char*)malloc(10);
    strcpy(flashVars.HDOP, Hdop);
    flashVars.SAT = (char*)malloc(5);
    strcpy(flashVars.SAT, Satellites);
    flashVars.BAT = measureBattery();
    csum = flashVars.PREFIX + flashVars.INTERVAL + flashVars.MOV_INTERVAL; // Initialise the checksum
    csum = csum & 0xff;
    flashVars.CSUM = csum;
    flashVarsMem.write(flashVars); // Write the flash variables

    //free allocated memory
    
    free(flashVars.LAT);
    free(flashVars.LONG);
    free(flashVars.ALT);
    free(flashVars.SPEED);
    free(flashVars.COURSE);
    free(flashVars.HDOP);
    free(flashVars.SAT);
  }

  rtc.begin(); // Start the RTC now that BEACON_INTERVAL has been updated
  rtc.setAlarmSeconds(rtc.getSeconds()); // Initialise RTC Alarm Seconds
  
  alarmMatch(); // Set next alarm time using updated BEACON_INTERVAL
  rtc.enableAlarm(rtc.MATCH_HHMMSS); // Alarm Match on hours, minutes and seconds
  rtc.attachInterrupt(alarmMatch); // Attach alarm interrupt
  
  iterationCounter = 0; // Make sure iterationCounter is set to zero (indicating a reset)
  loop_step = 0; // Make sure loop_step is set to init
}

void loop()
{
  //Check battery level before doing anything else
  if(batteryLevelOK()){
    
    //Incase a signal comes from the receiver, transmit
    if(listener()){
      //transmit data
      transmit();
    
    }
  }else{
    Serial.println("Battery Voltage is below the Threshold.");   
  }
}

void transmit(){
  char radiopacket[256];
  flashVars = flashVarsMem.read();
  char batbuf[10];
  dtostrf(flashVars.BAT, 6, 3, batbuf);
  sprintf(radiopacket, "%s,%s,%s,%s,%s,%s,%s,%s", flashVars.LAT, flashVars.LONG, flashVars.ALT, flashVars.SPEED, flashVars.COURSE, flashVars.HDOP, flashVars.SAT, batbuf);
  Serial.print("Transmitting: "); 
  Serial.println(radiopacket);
  delay(10);
  rf95.send((uint8_t *)radiopacket, sizeof(radiopacket));
  Serial.println("Waiting for packet to complete..."); 
  delay(10);
  rf95.waitPacketSent();
}

bool listener(){
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  Serial.println("Waiting for reply...");
  if (rf95.waitAvailableTimeout(1000))
  { 
    // Should be a reply megpsSerialage for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RgpsSerialI: ");
      Serial.println(rf95.lastRssi(), DEC); 
      return true;   
    }
    else
    {
      Serial.println("Receive failed");
      return false;
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
    return false;
  }
}

void GPSData(){
  
  while (gpsSerial.available() > 0){
    if (gps.encode(gpsSerial.read()))
    {
      //location
      if (gps.location.isValid())
      {
        dtostrf(gps.location.lat(),10, 7, latitude);
        dtostrf(gps.location.lng(),10, 7, longitude);
        Serial.print(latitude);
        Serial.print(F(","));
        Serial.println(longitude);
      }//if gps location is not valid, return 0.0000 as co-ordinates. This can happen when the Gps is unable to find satellites do to a poor antenna
      else
      {
        float invalid = 0.00000;        
        dtostrf(invalid,7, 5, latitude);
        dtostrf(invalid,7, 5, longitude);
      }
      //altitude
      if (gps.altitude.isValid())
      {
        dtostrf(gps.altitude.meters(),7, 2, Altitude);
        Serial.print(Altitude);
      }
      else
      {
        float invalid = 0000.00;        
        dtostrf(invalid,7, 2, Altitude);
      }
      //speed
      if (gps.speed.isValid())
      {
        dtostrf(gps.speed.kmph(),6, 2, Speed);
        Serial.print(Speed);
      }
      else
      {
        float invalid = 000.00;        
        dtostrf(invalid,6, 2, Speed);
      }
      //course
      if (gps.course.isValid())
      {
        dtostrf(gps.course.deg(),7, 2, Course);
        Serial.print(Course);
        
      }
      else
      {
        float invalid = 000.00;        
        dtostrf(invalid,7, 2, Course);
      }
      //hdop
      if (gps.hdop.isValid())
      {
        dtostrf(gps.hdop.hdop(),6, 1, Hdop);
        Serial.print(Hdop);
      }
      else
      {
        float invalid = 000.00;        
        dtostrf(invalid,6, 1, Hdop);
      }
      //Satellites
      if (gps.satellites.value())
      {
        itoa(gps.satellites.value(), Satellites, 10);
        Serial.print(Satellites);
      }
      else
      {
        int invalid = 0;        
        itoa(invalid, Satellites, 10);
      }
    }
  }
 }


 float measureBattery(){
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we (2 voltage dividers resistor) divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  
  return measuredvbat;
 }

 
 bool batteryLevelOK(){
  
   float measuredvbat = measureBattery();
   if(measuredvbat < bat_threshold){
          return false;
   }else{
          return true;
   }
 }

 // RTC alarm interrupt
void alarmMatch()
{ 
  done_waiting = 1;
  
  int rtc_mins = rtc.getMinutes(); // Read the RTC minutes
  int rtc_hours = rtc.getHours(); // Read the RTC hours
  
  if(!use_movement_interval){
    Serial.println("beachon_interval ALARM occured...");
  if (BEACON_INTERVAL > 1440) BEACON_INTERVAL = 1440; // Limit BEACON_INTERVAL to one day
  rtc_mins = rtc_mins + BEACON_INTERVAL; // Add the BEACON_INTERVAL to the RTC minutes}
  }
  else{
    Serial.println("Movvement_interval ALARM occured...");
    if (MOVEMENT_INTERVAL > 1440) MOVEMENT_INTERVAL = 1440; // Limit MOVEMENT_INTERVAL to one day
    rtc_mins = rtc_mins + MOVEMENT_INTERVAL; // Add the MOVEMENT_INTERVAL to the RTC minutes}
  }
  
  while (rtc_mins >= 60) { // If there has been an hour roll over
    rtc_mins = rtc_mins - 60; // Subtract 60 minutes
    rtc_hours = rtc_hours + 1; // Add an hour
  }
  rtc_hours = rtc_hours % 24; // Check for a day roll over
  rtc.setAlarmMinutes(rtc_mins); // Set next alarm time (minutes)
  rtc.setAlarmHours(rtc_hours); // Set next alarm time (hours)

}

//Tilt sensor interrupt.............
void tilt_sensor_interrupt(){
  detachInterrupt(digitalPinToInterrupt(tilt_sensor));
  Serial.println("Tilt interrupt called..");
  Serial.println("Tilt interrupt dettached...");
  
  last_movement_time = millis(); //Noting the time when a movement occured.

  //if NOT already in movement_interval enabled mode...
  if(!use_movement_interval){
    Serial.println("First time interrupt");
    //update alarm setting..
    use_movement_interval = 1;
    movements_during_movement_interval = 0;
    
    alarmMatch();
    rtc.enableAlarm(rtc.MATCH_HHMMSS); // Alarm Match on hours, minutes and seconds
    rtc.attachInterrupt(alarmMatch); // Attach alarm interrupt
  }
  
  GPSData();
  delay(100);
  
  //log data to flush memory
  // Update flash memory
  flashStorage();

  Serial.println("Attaching interrupt again (FOR NOTING THE LAST MOVEMENT time only)");
  attachInterrupt(digitalPinToInterrupt(tilt_sensor), tilt_sensor_interrupt, CHANGE);
}

void flashStorage(){
  int new_movement_interval = 0;
  MOVEMENT_INTERVAL = new_movement_interval; // Update BEACON_INTERVAL
  // Update flash memory
  flashVars.PREFIX = 0xB5; // Reset the prefix (hopefully redundant!)
  flashVars.MOV_INTERVAL = new_movement_interval; // Store the new beacon interval
  flashVars.LAT = (char*)malloc(15);
  strcpy(flashVars.LAT, latitude);
  flashVars.LONG = (char*)malloc(15);
  strcpy(flashVars.LONG, longitude);
  flashVars.ALT = (char*)malloc(15);
  strcpy(flashVars.ALT, Altitude);
  flashVars.SPEED = (char*)malloc(10);
  strcpy(flashVars.SPEED, Speed);
  flashVars.COURSE = (char*)malloc(10);
  strcpy(flashVars.COURSE, Course);
  flashVars.HDOP = (char*)malloc(10);
  strcpy(flashVars.HDOP, Hdop);
  flashVars.SAT = (char*)malloc(5);
  strcpy(flashVars.SAT, Satellites);
  flashVars.BAT = measureBattery();
  int csum = flashVars.PREFIX + flashVars.INTERVAL + flashVars.MOV_INTERVAL; // Update the checksum
  csum = csum & 0xff;
  flashVars.CSUM = csum;
  flashVarsMem.write(flashVars); // Write the flash variables

  //free allocated memory
  free(flashVars.LAT);
  free(flashVars.LONG);
  free(flashVars.ALT);
  free(flashVars.SPEED);
  free(flashVars.COURSE);
  free(flashVars.HDOP);
  free(flashVars.SAT);
}
