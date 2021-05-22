#!/usr/bin/python

MQTT_ServerIP     = "192.168.5.248"
MQTT_ServerPort   = 1883

# On these ports the right boards will be attached
SERIAL_PORT_DEVICE_BASE = '/dev/ttyUSB*'
# serialPortDevice  = '/dev/ttyUSB0'

BOARD_NAME = '[RF12demo.12]'
NR_OF_BOARDS = 1

SERIAL_PORT_BAUDRATE = 57600
LOG_FILENAME         = "/home/pi/log/jeelink_mqtt.log"

MQTT_TOPIC_OUT       = 'huis/JeeLink/+/out'
#Bediend door gebruiker:
MQTT_TOPIC_HOMEASSISTANT_BEDIENING = 'huis/HomeLogic/+/bediening'
#Omgeschakeld in script:
MQTT_TOPIC_HOMELOGIC_BEDIENING     = 'huis/HomeAssistant/Actief-Hoog-tarief-CEZ/bediening'

MQTT_TOPIC_TOTALS    = 'huis/JeeLink/+/totals'
MQTT_TOPIC_CHECK     = "huis/JeeLink/RPiHome/check"
MQTT_TOPIC_REPORT    = "huis/JeeLink/RPiHome/report"
