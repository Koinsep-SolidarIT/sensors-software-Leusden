
# Software for Sensor.Community / Luftdaten.Info Sensor

## airrohr-firmware

The maintained main firmware for the Luftdaten.Info Sensor. 

## airrohr-update-loader

A transitional firmware which will look for a firmware file
stored on SPIFFS to replace itself with for next reboot
or do an endless loop of panic LED blinking if this fails.

This allows to do an Over-the-air (OTA) procedure on setups
that have a 1M/3M split layout (rather the more modern 2M/2M)
for firmwares larger than 512k (up to ~ 740k).

# Directories 

* BeginnersGuide	  Beginners guide to ESP programming with Arduino code
* airrohr-....        currently used versions (2017-05-23+)
* airrohr-update-...  currently used for OTA (2019-08-30+)
* apiclients	      clients for the dusti API and other APIs
* arduino	          native arduino code
* esp8266-lua	      nodemcu firmware code, lua scripts (not maintained after change to arduino)
* esp8266-arduino	  programming esp8266 with arduino code/IDE
* r-scripts	          sensor data analysis with R
* schematics	      frizzing schematics (not updated, 2016-03-01)

# Development

The development is done in the beta branch.

## New Sensors

* SEN5XX
* CO2 on SCD30

## Plugins

* MQTT
* Static IP
* Power save

## Directories 

* /bin      - flasher (FlashESP8266) and firmware bin
* /Doc      - manual/handleiding 
* /Doc/PCB 	- schematic and Gerberfile


## WIFI 

* password is 'airrohrcfg'

## Wiki

https://github.com/FijnStofGroep/sensors-software-Leusden/wiki

## Handleidingen NL

https://www.cmlleusden.nl/handleidingen-fijnstofmeter.html

## Config SEN55 on opensensemap and sensorcommunity

Sensorcommunity PM is on PIN 1 and Temp/Hum is on PIN 7 

See example picture on WIKI

https://github.com/FijnStofGroep/sensors-software-Leusden/wiki
