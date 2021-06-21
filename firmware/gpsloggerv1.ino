#include <SPI.h>
#include <RH_RF95.h>
#include <NMEAGPS.h>
#include <IridiumSBD.h> // Requires V2: https://github.com/mikalhart/IridiumSBD
#include <PString.h> // String buffer formatting: http://arduiniana.org
#include <GPSport.h>
#include <RTCZero.h> // M0 Real Time Clock
#include <FlashStorage.h>// Flash Storage
#include <SD.h> //sd card

#define RB_destination 0 // Serial number of the destination RockBLOCK (int). Set to zero to disable RockBLOCK message forwarding
#define RB_source 0 // Serial number of this unit (int)

int BEACON_INTERVAL = 10;
int MOVEMENT_INTERVAL = 5;
volatile bool use_movement_interval = 0;
int tilt_sensor = A5;
volatile unsigned long last_movement_time = 0;
int movements_during_movement_interval = 0; //number of movements occured during MOVEMENT_INTERVAL time... (if movement_interval was enabled)
volatile bool done_waiting = 0; //a variable to keep check of the state.
volatile int log_number = 0;

char outBuffer[340]; //outBuffer of 340 bytes..
NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values



RTCZero rtc; // Create an rtc object


typedef struct { // Define a struct to hold the flash variable(s)
  int PREFIX; // Flash storage prefix (0xB5); used to test if flash has been written to before
  int INTERVAL; // Message interval in minutes
  int MOV_INTERVAL; //movement interval
  // RockBLOCK source serial number: stored as an int; i.e. the RockBLOCK serial number of the 9603N attached to this beacon
  int RBSOURCE;
  // RockBLOCK destination serial number: stored as an int; i.e. the RockBLOCK serial number of the 9603N you would like the messages delivered _to_
  int RBDESTINATION; // Set this to zero to disable RockBLOCK gateway message forwarding
  int CSUM; // Flash storage checksum; the modulo-256 sum of PREFIX and INTERVAL; used to check flash data integrity
} FlashVarsStruct;

FlashStorage(flashVarsMem, FlashVarsStruct); // Reserve memory for the flash variables
FlashVarsStruct flashVars; // Define the global to hold the variables
int RBSOURCE = RB_source;
int RBDESTINATION = RB_destination;

// Serial2 pin and pad definitions (in Arduino files Variant.h & Variant.cpp)
#define PIN_SERIAL2_RX       (34ul)               // Pin description number for PIO_SERCOM on D12
#define PIN_SERIAL2_TX       (36ul)               // Pin description number for PIO_SERCOM on D10
#define PAD_SERIAL2_TX       (UART_TX_PAD_2)      // SERCOM pad 2 (SC1PAD2)
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_3)    // SERCOM pad 3 (SC1PAD3)
// Instantiate the Serial2 class
Uart Serial2(&sercom1, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);
HardwareSerial &ssIridium(Serial2);

#define ssGPS Serial1 // Use M0 Serial1 to interface to the MAX-M8Q

//static const int len_setNav = 42;
static const int Enable_9603N = 5; // Enables the MPM3610 to provide power for the 9603N
static const int IridiumSleepPin = 6; // Iridium Sleep connected to D6
static const int RingIndicator = 21; // Iridium Ring Indicator connected to D21
static const int STEPUP_KEY = 2;
IridiumSBD isbd(ssIridium, IridiumSleepPin); // This should disable the 9603 and disable Ring alerts
//TinyGPS tinygps;
long iterationCounter = 0; // Increment each time a transmission is attempted

static const int ledPin = 13; // Red LED on pin D13
//#define NoLED // Uncomment this line to disable the LED

static const int GPS_EN = 11; // GNSS Enable on pin D11
#define GPS_ON LOW
#define GPS_OFF HIGH
#define VAP A7 // Bus voltage analog pin (bus voltage divided by 4.3)
#define VBAT_LOW 3.0 // Minimum voltage for MPM3610

// Loop Steps
#define initseq          0
#define start_GPS     1
#define read_GPS      2
#define start_9603    3
#define zzz           4
#define wake          5

void triggerStepUp(bool turnOn);

// Variables used by Loop
int year;
byte month, day, hour, minute, second, hundredths;
unsigned long dateFix, locationFix;
float latitude, longitude;
long altitude;
float speed;
short satellites;
long course;
long hdop;
bool fixFound = false;
bool charsSeen = false;
int loop_step = initseq;
float vbat = 5.0;
unsigned long tnow;

// Storage for the average voltage during Iridium callbacks
const int numReadings = 25;   // number of samples
int readings[numReadings];    // the readings from the analog input
int readIndex = 0;            // the index of the current reading
long int total = 0;           // the running total
int latest_reading = 0;       // the latest reading
int average_reading = 0;      // the average reading

//rfm9x
#define RFM95_CS 8 //cs
#define RFM95_RST 4 //rst
#define RFM95_INT 3 //irq
#define RF95_FREQ 915.0 //frequency of the transiever modules
RH_RF95 rf95(RFM95_CS, RFM95_INT); //create an instance of the radio driver class
int16_t packetnum = 0;  // packet counter to be incremented per transmission

//sdcard
const int chipSelect = 4;
// IridiumSBD Callbacks
bool ISBDCallback()
{
#ifndef NoLED
  // 'Flash' the LED
  if ((millis() / 333) % 2 == 1) {
    digitalWrite(ledPin, HIGH);
  }
  else {
    digitalWrite(ledPin, LOW);
  }
#endif

  // Check the 'battery' voltage now we are drawing current for the 9603
  // If voltage is low, stop Iridium send
  get_vbat_smooth();

  if (vbat < VBAT_LOW) {
    Serial.print("***!!! LOW VOLTAGE (ISBDCallback) ");
    Serial.print(vbat, 2);
    Serial.println("V !!!***");
    return false; // Returning false causes IridiumSBD to terminate
  }
  else {
    return true;
  }

  delay(1);
}

// V2 console and diagnostic callbacks (replacing attachConsole and attachDiags)
void ISBDConsoleCallback(IridiumSBD *device, char c) {
  Serial.write(c);
}
void ISBDDiagsCallback(IridiumSBD *device, char c) {
  Serial.write(c);
}

// Interrupt handler for SERCOM1 (essential for Serial2 comms)
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

// RTC alarm interrupt
void alarmMatch()
{


  done_waiting = 1;

  int rtc_mins = rtc.getMinutes(); // Read the RTC minutes
  int rtc_hours = rtc.getHours(); // Read the RTC hours

  if (!use_movement_interval) {
    Serial.println("beachon_interval ALARM occured...");
    if (BEACON_INTERVAL > 1440) BEACON_INTERVAL = 1440; // Limit BEACON_INTERVAL to one day
    rtc_mins = rtc_mins + BEACON_INTERVAL; // Add the BEACON_INTERVAL to the RTC minutes}
  }
  else {
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

// Read and smooth the 'battery' voltage
// Average voltage over numReadings to smooth out any short dips
void get_vbat_smooth() {
  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  latest_reading = analogRead(VAP);
  readings[readIndex] = latest_reading;
  // add the reading to the total:
  total = total + latest_reading;
  // advance to the next position in the array:
  readIndex = readIndex + 1;
  // if we're at the end of the array...wrap around to the beginning:
  if (readIndex >= numReadings) readIndex = 0;
  // calculate the average:
  average_reading = total / numReadings; // Seems to work OK with integer maths - but total does need to be long int
  vbat = float(average_reading) * (4.3 * 3.3 / 1023.0); // Calculate average battery voltage
}

// Read the instantaneous 'battery' voltage
void get_vbat() {
  vbat = analogRead(VAP) * (4.3 * 3.3 / 1023.0); // Read battery voltage from resistor divider
}

// Initialise the smoothed 'battery' voltage
void init_vbat()
{
  // Initialise voltage sample buffer with current readings
  total = 0;
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = analogRead(VAP);
    total = total + readings[thisReading];
    delay(1);
  }
  get_vbat_smooth();
}

// Send message in u-blox UBX format
// Calculates and appends the two checksum bytes
// Doesn't add the 0xb5 and 0x62 sync chars (these need to be included at the start of the message)
void sendUBX(const uint8_t *message, const int len) {
  int csum1 = 0; // Checksum bytes
  int csum2 = 0;
  for (int i = 0; i < len; i++) { // For each byte in the message
    ssGPS.write(message[i]); // Write the byte
    if (i >= 2) { // Don't include the sync chars in the checksum
      csum1 = csum1 + message[i]; // Update the checksum bytes
      csum2 = csum2 + csum1;
    }
  }
  csum1 = csum1 & 0xff; // Limit checksums to 8-bits
  csum2 = csum2 & 0xff;
  ssGPS.write((uint8_t)csum1); // Send the checksum bytes
  ssGPS.write((uint8_t)csum2);
}



//Tilt sensor interrupt.............
void tilt_sensor_interrupt() {
  detachInterrupt(digitalPinToInterrupt(tilt_sensor));
  Serial.println("Tilt interrupt called..");
  Serial.println("Tilt interrupt dettached...");

  last_movement_time = millis(); //Noting the time when a movement occured.

  //if NOT already in movement_interval enabled mode...
  if (!use_movement_interval) {
    Serial.println("First time interrupt");
    //update alarm setting..
    use_movement_interval = 1;
    movements_during_movement_interval = 0;


    alarmMatch();
    rtc.enableAlarm(rtc.MATCH_HHMMSS); // Alarm Match on hours, minutes and seconds
    rtc.attachInterrupt(alarmMatch); // Attach alarm interrupt
  }
  else {
    Serial.println("(Only noting the movement_time");
  }
  Serial.println("Attaching interrupt again (FOR NOTING THE LAST MOVEMENT time only)");
  attachInterrupt(digitalPinToInterrupt(tilt_sensor), tilt_sensor_interrupt, CHANGE);
}

//this function basically takes a GPS reading, and adds it in the global OutBuffer (msg to be sent)
void log_gps_reading() {
  // Power up the GNSS

  Serial.println("Powering up the GNSS...");
  digitalWrite(GPS_EN, GPS_ON); // Enable the GNSS

  delay(2000); // Allow time for both to start


  // Start the GNSS serial port
  ssGPS.begin(9600);

  delay(1000); // Allow time for the port to open

  // Configure GNSS
  Serial.println("Configuring GNSS...");

  while (ssGPS.available()) {
    ssGPS.read(); // Flush RX buffer so we don't confuse TinyGPS with UBX acknowledgements
  }

  // Reset TinyGPS and begin listening to the GNSS
  Serial.println("Beginning to listen for GNSS traffic...");
  fixFound = false; // Reset fixFound
  charsSeen = false; // Reset charsSeen


  // Look for GNSS signal for up to 5 minutes
  for (tnow = millis(); !fixFound && millis() - tnow < 5UL * 60UL * 1000UL;)
  {

    if (gps.available( ssGPS )) {
      fix = gps.read();
      charsSeen = true;
      latitude = fix.latitude();
      longitude = fix.longitude();
      altitude = fix.altitude_cm();
      speed = fix.speed_kph() * 1000 / 3600;
      satellites = fix.satellites;

      year = fix.dateTime.year;
      month = fix.dateTime.month;
      day = fix.dateTime.date;
      hour = fix.dateTime.hours;
      minute = fix.dateTime.minutes;
      second = fix.dateTime.seconds;
      hundredths = fix.dateTime_cs;
      course = fix.heading_cd();
      //hdop = fix.hdop;

      if ( fix.valid.date && fix.valid.time && fix.valid.location && fix.valid.altitude && fix.valid.speed && fix.valid.heading ) {
        fixFound = 1;
      }
    }


    // if we haven't seen any GNSS data in 10 seconds, then stop waiting
    if (!charsSeen && millis() - tnow > 10000) {
      break;
    }

    // Check battery voltage now we are drawing current for the GNSS
    // If voltage is low, stop looking for GNSS and go to sleep
    get_vbat_smooth();
    if (vbat < VBAT_LOW) {
      break;
    }

#ifndef NoLED
    // 'Flash' the LED
    if ((millis() / 1000) % 2 == 1) {
      digitalWrite(ledPin, HIGH);
    }
    else {
      digitalWrite(ledPin, LOW);
    }
#endif

  }

  Serial.println(charsSeen ? fixFound ? F("A GNSS fix was found!") : F("No GNSS fix was found.") : F("Wiring error: No GNSS data seen."));
  Serial.print("Latitude (degrees): "); Serial.println(latitude, 6);
  Serial.print("Longitude (degrees): "); Serial.println(longitude, 6);
  Serial.print("Altitude (m): "); Serial.println(altitude / 100); // Convert altitude from cm to m

  if (vbat < VBAT_LOW) {
    Serial.print("***!!! LOW VOLTAGE (read_GPS) ");
    Serial.print(vbat, 2);
    Serial.println("V !!!***");

  }
  else if (!charsSeen) {
    Serial.println("***!!! No GNSS data received !!!***");

  }
  else {
    // Power down the GNSS
    Serial.println("Powering down the GNSS...");
    digitalWrite(GPS_EN, GPS_OFF); // Disable the GPS
    delay(1000); // Let the voltage decay

    char temp_msg[120]; // Always try to keep message short (maximum should be ~101 chars including RockBLOCK destination and source)

    if (fixFound)
    {
      if (RBDESTINATION > 0) {
        sprintf(temp_msg, "RB%07d,%d%02d%02d%02d%02d%02d,", RBDESTINATION, year, month, day, hour, minute, second);
      }
      else {
        sprintf(temp_msg, "%d%02d%02d%02d%02d%02d,", year, month, day, hour, minute, second);
      }
      int len = strlen(temp_msg);
      PString str(temp_msg + len, sizeof(temp_msg) - len);
      str.print(latitude, 6);
      str.print(",");
      str.print(longitude, 6);
      str.print(",");
      str.print(altitude / 100); // Convert altitude from cm to m
      str.print(",");
      str.print(speed, 1); // Speed in metres per second
      str.print(",");
      str.print(course / 100); // Convert from 1/100 degree to degrees
      str.print(",");
      str.print((((float)hdop) / 100), 1); // Convert from 1/100 m to m
      str.print(",");
      str.print(satellites);
      str.print(",");
      str.print("0,"); // Set pressure to zero
      str.print("0.0,"); // Set temperature to zero
      str.print(vbat, 2);
      str.print(",");
      str.print(float(iterationCounter), 0);
      if (RBDESTINATION > 0) { // Append source RockBLOCK serial number (as text) to the end of the message
        char sourceBuffer[12];
        sprintf(sourceBuffer, "RB%07d", RBSOURCE);
        str.print(",");
        str.print(sourceBuffer);
      }

      str.print("|"); //writing "|" at the end of the msg
    }

    else
    {
      // No GNSS fix found!
      if (RBDESTINATION > 0) {
        sprintf(temp_msg, "RB%07d,19700101000000,0.0,0.0,0,0.0,0,0.0,0,", RBDESTINATION);
      }
      else {
        sprintf(temp_msg, "19700101000000,0.0,0.0,0,0.0,0,0.0,0,");
      }
      int len = strlen(temp_msg);
      PString str(temp_msg + len, sizeof(temp_msg) - len);
      str.print("0,"); // Set pressure to zero
      str.print("0.0,"); // Set temperature to zero
      str.print(vbat, 2);
      str.print(",");
      str.print(float(iterationCounter), 0);
      if (RBDESTINATION > 0) { // Append source RockBLOCK serial number (as text) to the end of the message
        char sourceBuffer[12];
        sprintf(sourceBuffer, "RB%07d", RBSOURCE);
        str.print(",");
        str.print(sourceBuffer);
      }
      str.print("|");//writing "|" at the end of the msg
    }



    //concatenating the readings msg to the end of the OutBuffer string..
    sprintf(outBuffer, "%s%s", outBuffer, temp_msg);



  }



}


void setup()
{
  Serial.begin(115200);
  //initialize the LoRa radio transiever
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
  //set the transmission frequency
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);
  //initialize the sdcard module
  Serial.print("Initializing SD card...");
  // check if the SD card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("card failed or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  //SETTING PINS 3,4,8 TO OUTPUT modes. They are active-LOW, so setting them HIGH by default.
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);

  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);


  pinMode(tilt_sensor, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(tilt_sensor), tilt_sensor_interrupt, CHANGE);


  pinMode(Enable_9603N, OUTPUT); // 9603N enable via MPM3010
  digitalWrite(Enable_9603N, HIGH); // Disable the 9603N

  pinMode(GPS_EN, OUTPUT); // GNSS enable
  digitalWrite(GPS_EN, GPS_OFF); // Disable the GNSS

  pinMode(IridiumSleepPin, OUTPUT); // The call to IridiumSBD should have done this - but just in case
  digitalWrite(IridiumSleepPin, LOW); // Disable the Iridium 9603
  pinMode(RingIndicator, INPUT_PULLUP); // 9603N Ring Indicator

  pinMode(ledPin, OUTPUT); // LED
  digitalWrite(ledPin, LOW); // Disable the LED

  // See if global variables have already been stored in flash
  // If they have, read them. If not, initialise them.
  flashVars = flashVarsMem.read(); // Read the flash memory
  int csum = flashVars.PREFIX + flashVars.INTERVAL + flashVars.MOV_INTERVAL + flashVars.RBSOURCE + flashVars.RBDESTINATION; // Sum the prefix and data
  csum = csum & 0xff; // Limit checksum to 8-bits
  if ((flashVars.PREFIX == 0xB5) and (csum == flashVars.CSUM)) { // Check prefix and checksum match
    // Flash data is valid so update globals using the stored values
    BEACON_INTERVAL = flashVars.INTERVAL;

    //if MOV interval is 0...
    if (flashVars.MOV_INTERVAL == 0) {

      //SET IT TO MOVEMENT_INTERVAL..
      flashVars.MOV_INTERVAL = MOVEMENT_INTERVAL;
      csum = flashVars.PREFIX + flashVars.INTERVAL + flashVars.MOV_INTERVAL + flashVars.RBSOURCE + flashVars.RBDESTINATION; // Initialise the checksum
      csum = csum & 0xff;
      flashVars.CSUM = csum;

      //WRITING INTO THE FLASH MEMORY.
      flashVarsMem.write(flashVars); // Write the flash variables

      //rEADING updated flash memory.
      flashVars = flashVarsMem.read(); // Read the flash memory
    }

    MOVEMENT_INTERVAL = flashVars.MOV_INTERVAL; //redundant
    RBSOURCE = flashVars.RBSOURCE;
    RBDESTINATION = flashVars.RBDESTINATION;
  }
  else {
    // Flash data is corrupt or hasn't been initialised so do that now
    flashVars.PREFIX = 0xB5; // Initialise the prefix
    flashVars.INTERVAL = BEACON_INTERVAL; // Initialise the beacon interval
    flashVars.MOV_INTERVAL = MOVEMENT_INTERVAL; //movement interval
    flashVars.RBSOURCE = RBSOURCE; // Initialise the source RockBLOCK serial number
    flashVars.RBDESTINATION = RBDESTINATION; // Initialise the destination RockBLOCK serial number
    csum = flashVars.PREFIX + flashVars.INTERVAL + flashVars.MOV_INTERVAL + flashVars.RBSOURCE + flashVars.RBDESTINATION; // Initialise the checksum
    csum = csum & 0xff;
    flashVars.CSUM = csum;
    flashVarsMem.write(flashVars); // Write the flash variables
  }

  rtc.begin(); // Start the RTC now that BEACON_INTERVAL has been updated
  rtc.setAlarmSeconds(rtc.getSeconds()); // Initialise RTC Alarm Seconds

  alarmMatch(); // Set next alarm time using updated BEACON_INTERVAL
  rtc.enableAlarm(rtc.MATCH_HHMMSS); // Alarm Match on hours, minutes and seconds
  rtc.attachInterrupt(alarmMatch); // Attach alarm interrupt

  iterationCounter = 0; // Make sure iterationCounter is set to zero (indicating a reset)
  loop_step = initseq; // Make sure loop_step is set to init
}



void loop()
{
  unsigned long loopStartTime = millis();

  switch (loop_step) {

    case initseq:

#ifndef NoLED
      digitalWrite(ledPin, HIGH);
#endif

      // Start the serial console
      Serial.begin(115200);
      delay(10000); // Wait 10 secs - allow time for user to open serial monitor

      // Send welcome message
      Serial.println("Iridium 9603N Mini Beacon");

      // Echo the BEACON_INTERVAL
      Serial.print("Using a BEACON_INTERVAL of ");
      Serial.print(BEACON_INTERVAL);
      Serial.println(" minutes");

      // Echo the MOVEMENT_INTERVAL
      Serial.print("Using a MOVEMENT_INTERVAL of ");
      Serial.print(MOVEMENT_INTERVAL);
      Serial.println(" minutes");

      // Echo RBDESTINATION and RBSOURCE
      Serial.print("Using an RBDESTINATION of ");
      Serial.println(RBDESTINATION);
      Serial.print("Using an RBSOURCE of ");
      Serial.println(RBSOURCE);

      // Setup the IridiumSBD
      // (attachConsole and attachDiags methods have been replaced with ISBDConsoleCallback and ISBDDiagsCallback)
      isbd.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE); // Change power profile to default
      isbd.useMSSTMWorkaround(false); // Redundant?

      // Check the battery voltage
      // If voltage is low, go to sleep
      init_vbat(); // Use init_vbat to make sure the current true smoothed voltage is used (after coming out of deep sleep)
      if (vbat < VBAT_LOW) {
        Serial.print("***!!! LOW VOLTAGE (init) ");
        Serial.print(vbat, 2);
        Serial.println(" !!!***");
        loop_step = zzz;
      }
      else {
        loop_step = start_GPS;
      }

      break;

    case start_GPS:

      //comes here, then takes a GPS reading..

      if (use_movement_interval) {
        Serial.println("Movement interval alarm..");

        if (log_number < 4) {
          Serial.println("Log_number<4, so logging GPS value and going to sleep...");
          log_gps_reading();
          log_number++;
          loop_step = zzz;
        }
        else {
          Serial.println("Log_number>=4, resetting it to 0 and sendin the data through Iriduium...");
          log_gps_reading();
          log_number = 0;
          loop_step = start_9603;

        }
      }
      else {
        Serial.println("Beacon interval alarm...(Sending GPS data...)");
        log_gps_reading();
        loop_step = start_9603;
      }
      if (charsSeen == false) {
        Serial.println("****NO GNSS DATA RECEIVED. Please check (going to sleep)**** ");
        loop_step = zzz;
      }
      break;

    case start_9603:

#ifndef NoLED
      digitalWrite(ledPin, HIGH);
#endif

      // Start talking to the 9603 and power it up
      Serial.println("Beginning to talk to the 9603...");

      digitalWrite(Enable_9603N, LOW); // Enable the MPM3610
      delay(2000);

      // Turn on step-up
      triggerStepUp(true);

      ssIridium.begin(19200);
      delay(1000);

      if (isbd.begin() == ISBD_SUCCESS) // isbd.begin powers up the 9603
      {

        uint8_t mt_buffer[100]; // Buffer to store Mobile Terminated SBD message
        size_t mtBufferSize = sizeof(mt_buffer); // Size of MT buffer
        init_vbat(); // init_vbat to make sure the current true smoothed voltage is used now that the 9603N is active

        if (isbd.sendReceiveSBDText(outBuffer, mt_buffer, mtBufferSize) == ISBD_SUCCESS) { // Send the message; download an MT message if there is one
          if (mtBufferSize > 0) { // Was an MT message received?
            // Check message content
            mt_buffer[mtBufferSize] = 0; // Make sure message is NULL terminated
            String mt_str = String((char *)mt_buffer); // Convert message into a String
            Serial.print("Received a MT message: "); Serial.println(mt_str);

            // Check if the message contains a correctly formatted BEACON_INTERVAL: "[INTERVAL=nnn]"
            int new_interval = 0;
            int starts_at = -1;
            int ends_at = -1;
            starts_at = mt_str.indexOf("[INTERVAL="); // See is message contains "[INTERVAL="
            if (starts_at >= 0) { // If it does:
              ends_at = mt_str.indexOf("]", starts_at); // Find the following "]"
              if (ends_at > starts_at) { // If the message contains both "[INTERVAL=" and "]"
                String new_interval_str = mt_str.substring((starts_at + 10), ends_at); // Extract the value after the "="
                Serial.print("Extracted an INTERVAL of: "); Serial.println(new_interval_str);
                new_interval = (int)new_interval_str.toInt(); // Convert it to int
              }
            }
            if ((new_interval > 0) and (new_interval <= 1440)) { // Check new interval is valid
              Serial.print("New BEACON_INTERVAL received. Setting BEACON_INTERVAL to ");
              Serial.print(new_interval);
              Serial.println(" minutes.");
              BEACON_INTERVAL = new_interval; // Update BEACON_INTERVAL
              // Update flash memory
              flashVars.PREFIX = 0xB5; // Reset the prefix (hopefully redundant!)
              flashVars.INTERVAL = new_interval; // Store the new beacon interval
              int csum = flashVars.PREFIX + flashVars.INTERVAL + flashVars.RBSOURCE + flashVars.RBDESTINATION; // Update the checksum
              csum = csum & 0xff;
              flashVars.CSUM = csum;
              flashVarsMem.write(flashVars); // Write the flash variables
            }

            //-------------------------Check if message contains a correctly formatted MOVEMENT_INTERVAL: "[MOVEMENT_INTERVAL=nnnnn]"
            // Check if the message contains a correctly formatted MOVEMENT_INTERVAL: "[MOVEMENT_INTERVAL=nnnnn]"
            int new_movement_interval = 0;
            starts_at = -1;
            ends_at = -1;
            starts_at = mt_str.indexOf("[MOVEMENT_INTERVAL="); // See is message contains "[MOVEMENT_INTERVAL="
            if (starts_at >= 0) { // If it does:
              ends_at = mt_str.indexOf("]", starts_at); // Find the following "]"
              if (ends_at > starts_at) { // If the message contains both "[MOVEMENT_INTERVAL=" and "]"
                String new_movement_interval_str = mt_str.substring((starts_at + 10), ends_at); // Extract the value after the "="
                Serial.print("Extracted a MOVEMENT_INTERVAL of: "); Serial.println(new_movement_interval_str);
                new_movement_interval = (int)new_movement_interval_str.toInt(); // Convert it to int
              }
            }
            if ((new_movement_interval > 0) and (new_movement_interval <= 1440)) { // Check new interval is valid
              Serial.print("New MOVEMENT_INTERVAL received. Setting MOVEMENT_INTERVAL to ");
              Serial.print(new_movement_interval);
              Serial.println(" minutes.");
              MOVEMENT_INTERVAL = new_movement_interval; // Update BEACON_INTERVAL
              // Update flash memory
              flashVars.PREFIX = 0xB5; // Reset the prefix (hopefully redundant!)
              flashVars.MOV_INTERVAL = new_movement_interval; // Store the new beacon interval
              int csum = flashVars.PREFIX + flashVars.INTERVAL + flashVars.MOV_INTERVAL + flashVars.RBSOURCE + flashVars.RBDESTINATION; // Update the checksum
              csum = csum & 0xff;
              flashVars.CSUM = csum;
              flashVarsMem.write(flashVars); // Write the flash variables
            }

            //-----------------------------------------------------------------------------------------------------------------------

            // Check if the message contains a correctly formatted RBSOURCE: "[RBSOURCE=nnnnn]"
            int new_source = -1;
            starts_at = -1;
            ends_at = -1;
            starts_at = mt_str.indexOf("[RBSOURCE="); // See is message contains "[RBSOURCE="
            if (starts_at >= 0) { // If it does:
              ends_at = mt_str.indexOf("]", starts_at); // Find the following "]"
              if (ends_at > starts_at) { // If the message contains both "[RBSOURCE=" and "]"
                String new_source_str = mt_str.substring((starts_at + 10), ends_at); // Extract the value after the "="
                Serial.print("Extracted an RBSOURCE of: "); Serial.println(new_source_str);
                new_source = (int)new_source_str.toInt(); // Convert it to int
              }
            }
            // toInt returns zero if the conversion fails, so it is not possible to distinguish between a source of zero and an invalid value!
            // An invalid value will cause RBSOURCE to be set to zero
            if (new_source >= 0) { // If new_source was received
              Serial.print("New RBSOURCE received. Setting RBSOURCE to ");
              Serial.println(new_source);
              RBSOURCE = new_source; // Update RBSOURCE
              // Update flash memory
              flashVars.PREFIX = 0xB5; // Reset the prefix (hopefully redundant!)
              flashVars.RBSOURCE = new_source; // Store the new RockBLOCK source serial number
              int csum = flashVars.PREFIX + flashVars.INTERVAL + flashVars.RBSOURCE + flashVars.RBDESTINATION; // Update the checksum
              csum = csum & 0xff;
              flashVars.CSUM = csum;
              flashVarsMem.write(flashVars); // Write the flash variables
            }

            // Check if the message contains a correctly formatted RBDESTINATION: "[RBDESTINATION=nnnnn]"
            int new_destination = -1;
            starts_at = -1;
            ends_at = -1;
            starts_at = mt_str.indexOf("[RBDESTINATION="); // See is message contains "[RBDESTINATION="
            if (starts_at >= 0) { // If it does:
              ends_at = mt_str.indexOf("]", starts_at); // Find the following "]"
              if (ends_at > starts_at) { // If the message contains both "[RBDESTINATION=" and "]"
                String new_destination_str = mt_str.substring((starts_at + 15), ends_at); // Extract the value after the "="
                Serial.print("Extracted an RBDESTINATION of: "); Serial.println(new_destination_str);
                new_destination = (int)new_destination_str.toInt(); // Convert it to int
              }
            }
            // toInt returns zero if the conversion fails, so it is not possible to distinguish between a destination of zero and an invalid value!
            // An invalid value will cause RBDESTINATION to be set to zero
            if (new_destination >= 0) { // If new_destination was received
              Serial.print("New RBDESTINATION received. Setting RBDESTINATION to ");
              Serial.println(new_destination);
              RBDESTINATION = new_destination; // Update RBDESTINATION
              // Update flash memory
              flashVars.PREFIX = 0xB5; // Reset the prefix (hopefully redundant!)
              flashVars.RBDESTINATION = new_destination; // Store the new RockBLOCK destination serial number
              int csum = flashVars.PREFIX + flashVars.INTERVAL + flashVars.RBSOURCE + flashVars.RBDESTINATION; // Update the checksum
              csum = csum & 0xff;
              flashVars.CSUM = csum;
              flashVarsMem.write(flashVars); // Write the flash variables
            }



            //Reading msg to check for OUTPUT pin states changes...

            //for PIN3
            // Check if the message contains a correctly formatted PIN3: "[PIN3=n]"       n=0 (LOW), n=1 (HIGH)
            int new_pin3 = -1;
            starts_at = -1;
            ends_at = -1;
            starts_at = mt_str.indexOf("[PIN3="); // See is message contains "[PIN3="
            if (starts_at >= 0) { // If it does:
              ends_at = mt_str.indexOf("]", starts_at); // Find the following "]"
              if (ends_at > starts_at) { // If the message contains both "[RBDESTINATION=" and "]"
                String new_pin3_str = mt_str.substring((starts_at + 15), ends_at); // Extract the value after the "="
                Serial.print("Extracted PIN3 : "); Serial.println(new_pin3_str);
                new_pin3 = (int)new_pin3_str.toInt(); // Convert it to int
              }
            }
            // toInt returns zero if the conversion fails, so it is not possible to distinguish between a destination of zero and an invalid value!
            // An invalid value will cause PIN3 to be set to zero
            if (new_pin3 >= 0) { // If new_destination was received
              Serial.print("New PIN3 state received. Setting PIN3 to ");
              Serial.println(new_pin3);

              //Setting pin3 state:
              digitalWrite(3, new_pin3);

            }

            //----------------------------------------------------------------------
            //for PIN4
            // Check if the message contains a correctly formatted PIN4: "[PIN4=n]"       n=0 (LOW), n=1 (HIGH)
            int new_pin4 = -1;
            starts_at = -1;
            ends_at = -1;
            starts_at = mt_str.indexOf("[PIN4="); // See is message contains "[PIN4="
            if (starts_at >= 0) { // If it does:
              ends_at = mt_str.indexOf("]", starts_at); // Find the following "]"
              if (ends_at > starts_at) { // If the message contains both "[PIN4=" and "]"
                String new_pin4_str = mt_str.substring((starts_at + 15), ends_at); // Extract the value after the "="
                Serial.print("Extracted PIN4 : "); Serial.println(new_pin4_str);
                new_pin4 = (int)new_pin4_str.toInt(); // Convert it to int
              }
            }
            // toInt returns zero if the conversion fails, so it is not possible to distinguish between a destination of zero and an invalid value!
            // An invalid value will cause PIN4 to be set to zero
            if (new_pin4 >= 0) { // If new_destination was received
              Serial.print("New PIN4 state received. Setting PIN4 to ");
              Serial.println(new_pin4);

              //Setting pin4 state:
              digitalWrite(4, new_pin4);

            }
            //----------------------------------------------------------------------
            //for PIN8
            // Check if the message contains a correctly formatted PIN8: "[PIN8=n]"       n=0 (LOW), n=1 (HIGH)
            int new_pin8 = -1;
            starts_at = -1;
            ends_at = -1;
            starts_at = mt_str.indexOf("[PIN8="); // See is message contains "[PIN8="
            if (starts_at >= 0) { // If it does:
              ends_at = mt_str.indexOf("]", starts_at); // Find the following "]"
              if (ends_at > starts_at) { // If the message contains both "[PIN8=" and "]"
                String new_pin8_str = mt_str.substring((starts_at + 15), ends_at); // Extract the value after the "="
                Serial.print("Extracted PIN8 : "); Serial.println(new_pin8_str);
                new_pin8 = (int)new_pin8_str.toInt(); // Convert it to int
              }
            }
            // toInt returns zero if the conversion fails, so it is not possible to distinguish between a destination of zero and an invalid value!
            // An invalid value will cause PIN8 to be set to zero
            if (new_pin8 >= 0) { // If new_destination was received
              Serial.print("New PIN8 state received. Setting PIN8 to ");
              Serial.println(new_pin8);

              //Setting pin8 state:
              digitalWrite(8, new_pin8);

            }

            //-----------------------------------------------------------------------------------------
          }

#ifndef NoLED
          // Give the LED ~ten short flashes to indicate successful transmission
          for (int flashCount = 0; flashCount < 11; flashCount++)
          {
            digitalWrite(ledPin, LOW);
            delay(100);
            digitalWrite(ledPin, HIGH);
            delay(100);
          }
#endif
        }
        ++iterationCounter; // Increment iterationCounter (regardless of whether send was successful)
      }

      sprintf(outBuffer, ""); //resetting the output STRING.. to empty string..
      movements_during_movement_interval = 0; //setting the movements count to 0


      loop_step = zzz;

      break;

    case zzz:

      // Get ready for sleep
      Serial.println("Putting 9603N and GNSS to sleep...");
      isbd.sleep(); // Put 9603 to sleep
      delay(1000);
      ssIridium.end(); // Close GNSS, Iridium and eRIC serial ports
      ssGPS.end();
      delay(1000); // Wait for serial ports to clear

      // Disable: GNSS and MPM3610
      digitalWrite(GPS_EN, GPS_OFF); // Disable the GPS
      digitalWrite(Enable_9603N, HIGH); // Disable the 9603N

      // Turn LED off
      digitalWrite(ledPin, LOW);

      // Close and detach the serial console (as per CaveMoa's SimpleSleepUSB)
      Serial.println("Going to sleep until next alarm time...");
      delay(1000); // Wait for serial port to clear
      //Serial.end(); // Close the serial console
      USBDevice.detach(); // Safely detach the USB prior to sleeping

      // Sleep until next alarm match
      rtc.standbyMode();

      // Turn off step-up
      triggerStepUp(false);


      // Wake up!
      loop_step = wake;



      if ( (millis() - last_movement_time > 2 * MOVEMENT_INTERVAL * 1000 * 60) && use_movement_interval ) {
        Serial.println("2x movement interval passed without a movement....Disabling it...");
        //DISABLE MOVEMENT_INTERVAL
        use_movement_interval = 0;

        //Set alarm setting to BEACON-INTERVAL again, and activate the 9603 for ONCE right now.
        alarmMatch();
        rtc.enableAlarm(rtc.MATCH_HHMMSS); // Alarm Match on hours, minutes and seconds
        rtc.attachInterrupt(alarmMatch); // Attach alarm interrupt
      }

      break;

    case wake:
      // Attach and reopen the serial console
      USBDevice.attach(); // Re-attach the USB
      delay(1000);  // Delay added to make serial more reliable

      // Now loop back to init
      loop_step = initseq;

      break;
  }
}

// When SAMD turns on we send 100ms and when it sleeps it must send 2s
void triggerStepUp(bool turnOn) {
  if (turnOn) {
    digitalWrite(STEPUP_KEY, HIGH);
    delay(50);
    digitalWrite(STEPUP_KEY, LOW);
    delay(100);
    digitalWrite(STEPUP_KEY, HIGH);
  }
  else {
    digitalWrite(STEPUP_KEY, LOW);
    delay(2000);
  }
}
