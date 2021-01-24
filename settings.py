#!/usr/bin/python

MQTT_ServerIP     = "192.168.5.248"
MQTT_ServerPort   = 1883
serialPortDevice  = '/dev/ttyUSB0'
serialPortBaudrate = 57600
LOG_FILENAME      = "/home/pi/log/jeelink_mqtt.log"

MQTT_TOPIC_OUT       = 'huis/JeeLink/+/out'
#Bediend door gebruiker:
MQTT_TOPIC_HOMEASSISTANT_BEDIENING = 'huis/HomeLogic/+/bediening'
#Omgeschakeld in script:
MQTT_TOPIC_HOMELOGIC_BEDIENING     = 'huis/HomeAssistant/Actief-Hoog-tarief-CEZ/bediening'

MQTT_TOPIC_TOTALS    = 'huis/JeeLink/+/totals'
MQTT_TOPIC_CHECK     = "huis/JeeLink/RPiHome/check"
MQTT_TOPIC_REPORT    = "huis/JeeLink/RPiHome/report"
