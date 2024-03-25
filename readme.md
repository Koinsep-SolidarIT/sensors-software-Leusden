
# Software for Sensor.Community / Luftdaten.Info Sensor

## airrohr-firmware

The maintained main firmware for the Luftdaten.Info Sensor. 

# Development

The development is done in the beta branch.

## New Sensors

* SEN5XX
* CO2 on SCD30

## Plugins

* MQTT
* Static IP
* Power save

## March 2024
* Update ca-root certificat
* Emulate SEN55 => SPS30 and SHT35 (Madavi.de / Feistaub-app / Openesensemap.org /sensor.community)
* Checkbox SEN55 on Pin 16 | sps30 PIN1 AND sht35 PIN7 (sensor.community)

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
