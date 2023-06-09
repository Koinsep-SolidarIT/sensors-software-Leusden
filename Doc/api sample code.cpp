var Timer changeTimerBME = null
var Timer changeTimerSDS = null
val String sensorID = "esp8266-YOURSENSORID"

rule "Send BME280 data to sensor.community"
when
    Item BME280_Temperature changed or
    Item BME280_HumidityRel changed or
    Item BME280_Pressure    changed
then
    if (changeTimerBME === null) {
         changeTimerBME = createTimer(now.plusSeconds(1), [ |
            var String url = "https://api.sensor.community/v1/push-sensor-data/"
            val String contentType = "application/json"
            val String content = '{
   "sensordatavalues":[
     {"value_type":"temperature","value":"' + (BME280_Temperature.state as Number).floatValue + '"},
     {"value_type":"humidity","value":"' + (BME280_HumidityRel.state as Number).floatValue + '"},
     {"value_type":"pressure","value":"' + (BME280_Pressure.state as Number).floatValue * 100 + '"}' + // API expects pressure in Pa, not in hPa
'  ]
}'
            val headers = newHashMap("X-Pin" -> "11", "X-Sensor" -> sensorID)
            val timeout = 60000

            // var String result =
            sendHttpPostRequest(url, contentType, content, headers, timeout)
            //logInfo("SensorCommunity", "sent »" + content + "«, result »" + result + "«")

            changeTimerBME = null // clear for another time
         ] )
   }  // else timer already running and we ignore trigger 
end

rule "Send SDS011 data to sensor.community"
when
    Item SDS011_PM25 changed or
    Item SDS011_PM10 changed
then
    if (changeTimerSDS === null) {
         changeTimerSDS = createTimer(now.plusSeconds(1), [ |
            var String url = "https://api.sensor.community/v1/push-sensor-data/"
            val String contentType = "application/json"
            val String content = '{
   "sensordatavalues":[
     {"value_type":"P1","value":"' + (SDS011_PM10.state as Number).floatValue + '"},
     {"value_type":"P2","value":"' + (SDS011_PM25.state as Number).floatValue + '"}
  ]
}'
            val headers = newHashMap("X-Pin" -> "1", "X-Sensor" -> sensorID)
            val timeout = 60000

            //var String result =
            sendHttpPostRequest(url, contentType, content, headers, timeout)
            //logInfo("SensorCommunity", "sent »" + content + "«, result »" + result + "«")

            changeTimerSDS = null // clear for another time
         ] )
   }  // else timer already running and we ignore trigger 
end