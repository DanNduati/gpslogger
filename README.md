# GpsLogger

## Hardware components
- Feather M0 with RFM9X
- SSD1306 oled 
- NEO-6M gps module
- sdcard module

## Description
- 2 different logging intervals in minutes
	- MOVEMENT INTERVAL 
		- triggered by a simple tilt sensor as interrupt using ON_CHANGE
		- When it is activated it will log positions according to that time variable(5 minutes)
		- After 2X movement interval time has passed (2x5=10minutes) it reverts back to 'BEACON_INTERVAL'
	- BEACON INTERVAL
		- Remains passive until it receives a 'ping' from the rx unit it will then transfer all the logged positions from the logger to the RX unit
- Transmission of the logger data to the RX should occur by LoRa radio protocol
- The data should have some form of encryption whilst stored on the logger and while being transferred from the logger to the RX and the RX should be able to decrypt the data
- The logger should be in deep sleep while not logging and should utilise the RTC function of the SAMD21
