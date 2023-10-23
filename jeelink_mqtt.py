#!/usr/bin/python3

import os
import signal
import time
import serial
import _thread
import traceback
import json
import glob

from queue import Queue
import paho.mqtt.publish as mqtt_publish
import paho.mqtt.client as mqtt_client

# external files/classes
import logger
import serviceReport
import settings

rainQuantity = {"idx": 154, "svalue": "0;197.0"}

# MQTT
kwhMeterStatus = {"totalKWHpulsesI": 0, "totalKWHpulsesII": 0, "power": 0.0}
waterPompStatus = {}
postbusStatus = {}

# Rain detection
raining = False
oldRaining = False
oldMmRain = 0
rainTimer = 0  # sec
firstRainPulsesMsg = True

totalKWHpulsesI = 0      # High-tarif
totalKWHpulsesII = 0     # Low-tarif
activeLowTarif = False  # Actief Laag tarief CEZ
totalsInitialised = False
firstPulsesMsg = True
oldPulseTimer = 0
oldPulses = 0
# oldPulsesCEZ = {}
sendQueue = Queue(maxsize=0)
openPorts = {}

jeeNodeRSSIlevels = ["dummy", "(-106dB)", "(-100dB)", "(-94dB)", "(-88dB)", "(-82dB)", "(-76dB)", "(-70dB)"]

# Waterpomp
oldPompAanRelais = 0

exitThread = False
serialPort = None


def current_sec_time():
    return int(round(time.time()))


def signal_handler(_signal, frame):
    global exitThread

    print('You pressed Ctrl+C!')
    exitThread = True


def on_message(_client, userdata, msgJson):
    print(('ERROR: Received ' + msgJson.topic + ' in on_message function' + str(msgJson.payload)))


def setFS20ST(unit, val):
    # 10;NewKaku;00baea06;3;ON;
    if int(val) != 0:
        sendQueue.put("17,17,%d,17f".encode() % unit)
    else:
        sendQueue.put("17,17,%d,0f".encode() % unit)


def on_message_out(_client, userdata, msg):
    # print(("on_message_out:" + msg.topic + " " + str(msg.payload)))
    # print(msg.topic + " " + str(msg.payload))
    topics = msg.topic.split("/")

    deviceName = topics[2]  # huis/JeeLink/FS20ST-1/out
    cmnd = deviceName.split("-")  # KaKu-12

    if cmnd[0] == "FS20ST":
        # print("Activate FS20ST WCD: %s" % cmnd[1])
        setFS20ST(int(cmnd[1]), msg.payload)


def on_message_homeassistant_bediening(_client, userdata, msg):
    global activeLowTarif

    # print("on_message_homeassistant_bediening:" + msg.topic + " " + str(msg.payload))
    # print(msg.topic + " " + str(msg.payload))

    topc = msg.topic.split('/')
    deviceName = topc[2]
    # print(deviceName)
    if deviceName == 'Waterpomp-Reset':
        if int(msg.payload) == 1:
            print("Waterpomp: Reset")
            sendQueue.put("2,3a".encode())  # Waterpomp: Reset
    elif deviceName == 'Waterpomp-Blokkeren':
        stat = int(msg.payload)
        if stat == 1:
            print("Waterpomp: Blokkeren")
            sendQueue.put("9,3a".encode())  # Waterpomp: Blokkeren
    elif deviceName == 'Waterpomp-GetInfo':
        # stat = int(msg.payload)
        sendQueue.put("10,3a".encode())  # Waterpomp: Info ophalen
    elif deviceName == 'Waterpomp-Debug':
        if int(msg.payload) == 1:
            print("Waterpomp: Debug aan")
            sendQueue.put("1,3a".encode())  # Waterpomp: Debug aan
        else:
            print("Waterpomp: Debug uit")
            sendQueue.put("0,3a".encode())  # Waterpomp: Debug uit


def on_message_homelogic_bediening(_client, userdata, msg):
    global activeLowTarif

    # print("on_message_homelogic_bediening:" + msg.topic + " " + str(msg.payload))
    # print(msg.topic + " " + str(msg.payload))

    topc = msg.topic.split('/')
    deviceName = topc[2]
    # print(deviceName)
    if deviceName == 'Actief-Laag-tarief-CEZ':
        if int(msg.payload) == 1:
            activeLowTarif = True
        else:
            activeLowTarif = False


# huis/JeeLink/Get-kWh-Totals/totals
def on_message_totals(_client, userdata, msgJson):
    global totalsInitialised
    global totalKWHpulsesI
    global totalKWHpulsesII
    global oldPulses
    global firstPulsesMsg
    global oldPulseTimer

    msgPayload = msgJson.payload.decode()

    print(("on_message_totals: " + msgJson.topic + ": " + msgPayload))

    if msgPayload != '0':
        if not totalsInitialised:  # Name: "Node CEZ meter"
            msg = json.loads(msgPayload)
            # print(msg['oldPulses'])
            totalKWHpulsesI = msg['totalKWHpulsesI']
            totalKWHpulsesII = msg['totalKWHpulsesII']
            oldPulses = msg['oldPulses']
            totalsInitialised = True
            firstPulsesMsg = True
            oldPulseTimer = current_sec_time()
        else:
            print('WARNING: Totals already initialised? !!!')
    else:
        print('Totals not available at HomeLogic (yet)')


def openSerialPort():
    global exitThread
    global openPorts

    # Get a list of to be opened serial ports
    ports = glob.glob(settings.SERIAL_PORT_DEVICE_BASE)

    for port in ports:
        try:
            print("Trying port: %s" % port)
            ser = serial.Serial(port=port,
                                baudrate=settings.SERIAL_PORT_BAUDRATE,
                                parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE,
                                bytesize=serial.EIGHTBITS,
                                timeout=1)  # 1=1sec 0=non-blocking None=Blocked

            if ser.isOpen():
                print(("Connected to serial port %s, now testing which board is connected:" % port))
                found = False
                boardName = ""
                timeoutTimer = current_sec_time()
                while not found:
                    ser.write("0v".encode())
                    serInLine = ser.readline().decode()
                    serInLine = serInLine.rstrip("\r\n")
                    if serInLine != "":
                        msg = serInLine.split(' ')
                        # print(msg)
                        if msg[0][0] == '[':
                            boardName = msg[0]
                            if boardName == '[RF12demo.12]':
                                openPorts[boardName] = ser
                                print(" - JeeLink adapter %s found on device %s" % (boardName, ser.name))
                                found = True
                    if exitThread or (current_sec_time() - timeoutTimer) > 5:
                        print("timeout!")
                        break
                if not found:
                    print(' - Not the correct board found: %s' % boardName)

        # Handle other exceptions and print the error
        except Exception:
            print("Unable to open port %s, trying next one" % port)
            # print("Exception: %s" % str(e))
            # traceback.print_exc()

    nrOfFoundPorts = len(openPorts)
    if nrOfFoundPorts != settings.NR_OF_BOARDS:
        # Report failure to Home Logic system check
        serviceReport.sendFailureToHomeLogic(serviceReport.ACTION_RESTART, 'Not enough GPIO ports found. Found %d and need %d serial ports' % (nrOfFoundPorts, settings.NR_OF_BOARDS))

        # # Suppress restart loops from systemd if something is wrong
        time.sleep(780)  # 13 min
        exitThread = True


def initJeeLink(ser):
    ser.write("1l".encode())  # Led on
    ser.write("1q".encode())  # Quiet mode
    ser.write("8b".encode())  # 868 MHz
    # ser.write("1i")  # nodeId=1
    # ser.write("5g")  # Group=5
    ser.write("0l".encode())  # Led off


def serialPortThread(boardName, dummy):
    global serialPort
    global exitThread
    global totalsInitialised
    global firstPulsesMsg
    global totalKWHpulsesI   # High tarif
    global totalKWHpulsesII  # Low tarif
    global activeLowTarif
    global oldPulseTimer
    global oldPulses
    global raining
    global oldRaining
    global oldMmRain
    global rainTimer
    global firstRainPulsesMsg

    # Waterpomp regeling
    global oldPompAanRelais

    global openPorts

    serialPort = openPorts[boardName]
    mqtt_publish.single("huis/HomeLogic/Get-kWh-Totals/command", 1, qos=1, hostname=settings.MQTT_ServerIP)

    while not exitThread:
        try:
            if serialPort.isOpen():
                serInLine = serialPort.readline().decode()
            else:
                serInLine = ""

            if serInLine != "":
                serInLine = serInLine.rstrip("\r\n")
                # print('JeeLink: %s' % (serInLine))
                msg = serInLine.split(' ')
                # print(msg)
                # JeeLink is started: Init it
                if msg[0] == "[RF12demo.12]":
                    # jeeLinkRSSI = False
                    initJeeLink(serialPort)
                    print("JeeLink[RF12demo.12]: Adapter Initialized!...")

                # JeeLink with rssi software (signal strength) is started: Init it
                if msg[0] == "[RF12demo-rssi.8]":
                    # jeeLinkRSSI = True
                    initJeeLink(serialPort)
                    print("JeeLink [RF12demo-rssi.8]: Adapter Initialized!...")

                # Only handle messages starting with 'OK' from JeeLink
                if msg[0] == 'OK':
                    # Reset the Rx timeout timer
                    serviceReport.systemWatchTimer = current_sec_time()

                    # print 'OK found!'
                    del msg[0]  # remove 'OK' from list

                    nodeId = int(msg[0]) & 0x1F  # NodeId 0..31
                    del msg[0]  # remove NodeId from list

                    # if available, gt the rssi signal strength
                    # if jeeLinkRSSI:
                    #     JeeNode RSSI Signal level can contain: -106, -100, -94, -88, -82, -76, -70
                    #     Domoticz gets signal level 1..7 (7=best)
                    #     signal = jeeNodeRSSIlevels.index(msg[-1])
                    #     print "JeeNode RSSI %s -> Domoticz signal %d" % (msg[-1], signal)

                    # print("NodeId: %d Msg: %s" % (nodeId, msg))

                    # nodeId 3: Waterpomp regeling
                    if nodeId == 3:

                        pompAanTijd = int(float(msg[0]) + float(msg[1]) * 256)
                        pompUitTijd = int(float(msg[2]) + float(msg[3]) * 256)
                        pompAanRelais = int(msg[4])
                        waterDruk = int(msg[5])
                        waterDrukAan = int(msg[6])
                        waterDrukUit = int(msg[7])
                        pompLed = int(msg[8])
                        storingsLed = int(msg[9])
                        pompGeblokkeerd = int(msg[10])
                        # waterdrukMask = int(msg[11])
                        waarschuwingsMelding = int(msg[12])
                        alarmMelding = int(msg[13])

                        waterPompStatus['pompAanTijd'] = pompAanTijd / 10
                        waterPompStatus['pompUitTijd'] = pompUitTijd / 10
                        waterPompStatus['pompAanRelais'] = pompAanRelais
                        waterPompStatus['waterDruk'] = waterDruk
                        waterPompStatus['waterDrukAan'] = waterDrukAan
                        waterPompStatus['waterDrukUit'] = waterDrukUit
                        waterPompStatus['pompLed'] = pompLed
                        waterPompStatus['storingsLed'] = storingsLed
                        waterPompStatus['pompGeblokkeerd'] = pompGeblokkeerd
                        # waterPompStatus['waterdrukMask'] = waterdrukMask
                        waterPompStatus['waarschuwingsMelding'] = waarschuwingsMelding
                        waterPompStatus['alarmMelding'] = alarmMelding

                        # print("Waterpomp: pompAanRelais:%d waterDruk:%d bar orgDrukRegeling:%d drukAan:%d" \
                        #      " drukUit:%d pompLed:%d storingsLed:%d pompGeblokkeerd:%d niemandThuis:%d waarschuwing:%d" \
                        #      " alarm:%d waterdrukMask:%x pompAanTijd:%.1f pompUitTijd:%.1f " %
                        #     (pompAanRelais, waterDruk, origineleDrukRegeling, waterDrukAan, waterDrukUit, pompLed, storingsLed,
                        #      pompGeblokkeerd, niemandThuis, waarschuwingsMelding, alarmMelding, waterdrukMask, pompAanTijd / 10, pompUitTijd / 10))

                        if oldPompAanRelais != pompAanRelais:
                            oldPompAanRelais = pompAanRelais
                            if (pompAanRelais == 0) and (pompAanTijd > 10):
                                # Pomp is uit gegaan, bereken nu aan de hand van de pompAanTijd, de hoeveelheid gebruikt water
                                # Gemeten (handmatig) pompAanTijd = 21,3 sec=14 liter water=0,65727 liter/sec
                                # Gemeten (handmatig) pompAanTijd = 25,2sec -1sec=24,2sec=14,8 liter water=0,61157 liter/sec
                                # Gemeten (handmatig) pompAanTijd = 24,5sec -1sec=23,5sec=14,8 liter water=0,61157 liter/sec
                                # Grundfos sq 2-70 heeft een softstart van 2 sec naar volle druk, dus 1 sec eraf halen
                                litersWater = (pompAanTijd - 10) * 0.629787
                                # print(("Aantal liters gepompt water: %.2f liter (pompAanTijd: %.1f sec)" % (litersWater / 10, pompAanTijd / 10)))
                                # http://192.168.5.248:8080/json.htm?type=command&param=udevice&idx=324&svalue=5

                                fLitersWater = float(litersWater / 10)
                                sensorData = {'Liters': '%1.1f' % fLitersWater}
                                mqtt_publish.single("huis/JeeLink/Water-verbruik/water", json.dumps(sensorData, separators=(', ', ':')), qos=1, hostname=settings.MQTT_ServerIP)
                                waterPompStatus['Liters'] = '%1.1f' % fLitersWater
                            else:
                                # Pomp is aan gegaan, verhoog teller
                                waterPompStatus['Liters'] = 0

                        mqtt_publish.single("huis/JeeLink/Waterpomp/water", json.dumps(waterPompStatus, separators=(', ', ':')), qos=1, hostname=settings.MQTT_ServerIP)

                    # nodeId 29: [JeeNode_OOKrelay868]
                    elif nodeId == 29:
                        # Message from [JeeNode_OOKrelay868]
                        # Message base:
                        # 0: OK
                        # 1: NodeId (& 0x1F) (0-31)
                        # 2: msg[0]: decoderType (& 0x0F), msgLength (& 0xF0)
                        # 3-??: msg[1]...data bytes

                        x = int(msg[0])
                        decoderType = x & 0x0F
                        msgLength = x >> 4
                        sensorType = int(msg[1]) & 0x0F
                        # print("JeeLink: msgLength %d, decoderType:%d, senorType %d" % (msgLength, decoderType, sensorType))

                        # if (decoderType == 3) and (sensorType == 7) and (msgLength == 8):
                        #     # Message from KS300/KS555 Conrad/ELV Weatherstation
                        #
                        #     # Message format:
                        #     # 0-2: Message base
                        #     # 3: 0-msg[1]: sensorType (& 0x0F), tempOutsideSign & 0x80 (1=minus), raining & 0x20(1=raining)
                        #     # 4: 1-msg[2]: tempOutside & 0x0F (*0.1C), tempOutside & 0xF0 (*1C)
                        #     # 5: 2-msg[3]: tempOutside & 0x0F (*10C), humidityOudside & 0xF0 (*1%)
                        #     # 6: 3-msg[4]: humidityOutside & 0x0F (*10%), wind & 0xF0 (*0.1km/h)
                        #     # 7: 4-msg[5]: wind & 0x0F (*1.0km/h), wind & 0xF0 (*10.0km/h)
                        #     # 8: 5-msg[6]: rain & 0x0F (*1 puls), rain & 0xF0 (*16 puls)
                        #     # 9: 6-msg[7]: rain & 0x0F (*256 puls), ??? & 0xF0 (unknown/unused)
                        #
                        #     # Convert outside temperature
                        #     temp = (10 * (int(msg[3]) & 0x0F)) + (int(msg[2]) >> 4) + (0.1 * (int(msg[2]) & 0x0F))
                        #     sensorStat = int(msg[1])
                        #     if (sensorStat & 0x80) == 0x80:
                        #         # Temperature sign: negative temperature
                        #         temp = -temp
                        #
                        #     if (sensorStat & 0x20) == 0x20:
                        #         # Rain dectector
                        #         rainingBit = True
                        #     else:
                        #         rainingBit = False
                        #
                        #     # Convert outside humidity
                        #     humidity = (10 * (int(msg[4]) & 0x0F)) + (int(msg[3]) >> 4)
                        #
                        #     # Convert outside windspeed km/h
                        #     windspeed = (10.0 * (int(msg[5]) >> 4)) + (int(msg[5]) & 0x0F) + (0.1 * (int(msg[4]) >> 4))
                        #     # print("Windspeed rechtstreeks van weerstation: %2.1f" % windspeed)
                        #
                        #     # Convert outside rain mm
                        #     raincounter = (256.0 * (int(msg[7]) & 0x0F)) + (16.0 * (int(msg[6]) >> 4)) + (int(msg[6]) & 0x0F)
                        #
                        #     # (float(RainCounter)*10.0f)*0.253f)
                        #     # rainQuantity   = { "idx": 154, "svalue" : "0;197.0"}
                        #     mmRain = (raincounter * 0.253)
                        #     # print("mmRain: %1.2f" % (mmRain))
                        #     addedMmRain = 0.0
                        #     if firstRainPulsesMsg:
                        #         firstRainPulsesMsg = False
                        #         print("Initialise the rain detection counter, without triggering it's raining")
                        #         oldMmRain = raincounter
                        #         rainTimer = current_sec_time()
                        #         # mqtt_publish.single("huis/HomeLogic/Actief-Regen/bediening", 0, qos=1, hostname="192.168.5.248")
                        #     else:
                        #         if mmRain != oldMmRain:
                        #             addedMmRain = mmRain - oldMmRain
                        #
                        #             # Wrong measurement, more than 10mm in 3-5 minutes
                        #             if addedMmRain >= 100.0:
                        #                 print(('addedMmRain: %1.2f: Wrong measurement, more than 100mm in 3-5 minutes, addedMmRain set to 0' % addedMmRain))
                        #                 addedMmRain = 0.0
                        #
                        #             elif addedMmRain < 0.0:
                        #                 print(('addedMmRain: %1.2f: Wrong measurement, negative value, addedMmRain set to 0' % addedMmRain))
                        #                 addedMmRain = 0.0
                        #             else:
                        #                 # No false measurements, it's raining
                        #                 raining = True
                        #
                        #             oldMmRain = mmRain
                        #             rainTimer = current_sec_time()
                        #         else:
                        #             if raining:
                        #                 # It was raining, but it's maybe dry, wait 20 min
                        #                 if current_sec_time() - rainTimer >= 1200:
                        #                     raining = False
                        #                     # print("KS555: Het is nu droog")
                        #
                        #         # Check if the rainCounter detected rain
                        #         if not raining:
                        #             # If not, check if the rain detector detected rain
                        #             if rainingBit:
                        #                 raining = True
                        #         # else:
                        #             # If rain already detected by the rainCounter, leave this status.
                        #             # Because the rain detector switches off very fast
                        #
                        #         if oldRaining != raining:
                        #             oldRaining = raining
                        #             # It is raining
                        #             if raining:
                        #                 # print "KS555: Het regent"
                        #                 mqtt_publish.single("huis/HomeLogic/Actief-Regen/bediening", 1, qos=1, hostname="192.168.5.248")
                        #             else:
                        #                 mqtt_publish.single("huis/HomeLogic/Actief-Regen/bediening", 0, qos=1, hostname="192.168.5.248")
                        #
                        #     # print("JeeLink: KS300: Outside temp %2.1f C, humidity %d %%" % (temp, humidity))
                        #     # print("JeeLink: KS300: Windspeed %2.1f m/s, rain %d pulses, unused?: %d" % (windspeed, raincounter, int(msg[7]) >> 4))
                        #
                        #     # if ((sensorStat & 0x20) != 0):
                        #     # #Raining bit: but doesn't work very well
                        #     #   print("JeeLink: KS300: It is raining at this moment")
                        #     #
                        #
                        #     if (sensorStat & 0x40) != 0:
                        #         # Low batt signal??
                        #         print("JeeLink: KS300: bit 6 normally 0, but now it is 1. Is this a low-batt bit")
                        #
                        #     if (sensorStat & 0x10) == 0:
                        #         # Low batt signal??
                        #         print("JeeLink: KS300: bit 4 normally 1, but now it is 0. Is this a low-batt bit")
                        #
                        #     sensorData = {'Temperature': '%1.1f' % temp, 'Humidity': '%d' % humidity,
                        #                   'Rain': "%1.2f" % addedMmRain, 'Windspeed': "%1.1f" % windspeed,
                        #                   'Raining': raining, 'RainingBit': rainingBit}
                        #     # sensorData['Temperature'] = temp
                        #
                        #     # sensorData['Battery level'] = batteryLevel
                        #     # sensorData['Signal level'] = signal
                        #
                        #     mqtt_publish.single("huis/JeeLink/Weerstation/weer", json.dumps(sensorData, separators=(', ', ':')), qos=1, hostname="192.168.5.248", retain=True)

                        if (decoderType == 3) and (sensorType == 1) and (msgLength == 5):
                            # Message from Conrad/ELV S300 Temp/humidity

                            sensorStat = int(msg[1])
                            # sensorIndex = (sensorStat & 0x70) >> 4
                            # Message format:
                            # 0-2: Message base
                            # 3: 0-msg[1]: sensorType (& 0x0F), tempSign & 0x80 (1=minus), sensorIndex (0x70) (0..7)
                            # 4: 1-msg[2]: temp & 0x0F (*0.1C), temp & 0xF0 (*1C)
                            # 5: 2-msg[3]: temp & 0x0F (*10C), humidity & 0xF0 (*0,1%)
                            # 6: 3-msg[4]: humidity & 0x0F (*1%), humidity & 0xF0 (*10%)

                            # Convert temperature
                            temp = (10 * (int(msg[3]) & 0x0F)) + (int(msg[2]) >> 4) + (0.1 * (int(msg[2]) & 0x0F))
                            if (sensorStat & 0x80) != 0:
                                # Temperature sign: negative temperature
                                temp = -temp

                            # Convert humidity
                            humidity = (10 * (int(msg[4]) >> 4)) + (int(msg[4]) & 0x0F)

                            # print("JeeLink: S300 Temp %2.1f C, humidity %d %%" % (temp, humidity))

                            sensorData = {'Temperature': temp, 'Humidity': humidity}
                            # sensorData['Battery level'] = batteryLevel
                            # sensorData['Signal level'] = signal

                            mqtt_publish.single("huis/JeeLink/Temp-Vriezer-Werkplaats/temp", json.dumps(sensorData, separators=(', ', ':')), hostname="192.168.5.248", retain=True)
                    else:
                        print(("JeeLink: Received an unknown message from NodeID %d" % nodeId))

            # Check if there is any message to send via JeeLink
            if not sendQueue.empty():
                sendMsg = sendQueue.get_nowait()
                # print(("SendMsg: %s" % sendMsg))
                if sendMsg != "":
                    serialPort.write(sendMsg)

        # In case the message contains unusual data
        except ValueError as arg:
            print(arg)
            traceback.print_exc()
            time.sleep(1)

        # Quit the program by Ctrl-C
        except KeyboardInterrupt:
            print("Program aborted by Ctrl-C")
            exit()

        # Handle other exceptions and print the error
        except Exception as arg:
            print("%s" % str(arg))
            traceback.print_exc()
            time.sleep(120)


def print_time(delay):
    count = 0
    while count < 5:
        time.sleep(delay)
        count += 1
        print("%s" % (time.ctime(time.time())))


# The callback for when the client receives a CONNACK response from the server.
def on_connect(_client, userdata, flags, rc):
    if rc == 0:
        print("MQTT Client connected successfully")
        _client.subscribe([(settings.MQTT_TOPIC_OUT, 1), (settings.MQTT_TOPIC_HOMEASSISTANT_BEDIENING, 1), (settings.MQTT_TOPIC_HOMELOGIC_BEDIENING, 1), (settings.MQTT_TOPIC_TOTALS, 1), (settings.MQTT_TOPIC_CHECK, 1)])
    else:
        print(("ERROR: MQTT Client connected with result code %s " % str(rc)))


###
# Initalisation ####
###
logger.initLogger(settings.LOG_FILENAME)

# Init signal handler, because otherwise Ctrl-C does not work
signal.signal(signal.SIGINT, signal_handler)

# Make the following devices accessable for user
os.system("sudo chmod 666 %s" % settings.SERIAL_PORT_DEVICE_BASE)

# Give Mosquitto and Home_logic the time to startup
time.sleep(6)

# First start the MQTT client
client = mqtt_client.Client()
client.message_callback_add(settings.MQTT_TOPIC_OUT,       on_message_out)
client.message_callback_add(settings.MQTT_TOPIC_HOMEASSISTANT_BEDIENING, on_message_homeassistant_bediening)
client.message_callback_add(settings.MQTT_TOPIC_HOMELOGIC_BEDIENING,     on_message_homelogic_bediening)
client.message_callback_add(settings.MQTT_TOPIC_TOTALS,    on_message_totals)
client.message_callback_add(settings.MQTT_TOPIC_CHECK,     serviceReport.on_message_check)
client.on_connect = on_connect
client.on_message = on_message
client.connect(settings.MQTT_ServerIP, settings.MQTT_ServerPort, 60)
client.loop_start()

# Search for the correct ports and open the ports
openSerialPort()

# Create the serialPortThread
try:
    # thread.start_new_thread( print_time, (60, ) )
    _thread.start_new_thread(serialPortThread, (settings.BOARD_NAME, 0))

except Exception as e:
    print("Error: unable to start the serialPortThread")
    print("Exception: %s" % str(e))
    traceback.print_exc()

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.

while not exitThread:
    time.sleep(30)  # 30s

for board in openPorts:
    print("Closed port %s for board %s" % (openPorts[board].name, board))
    if openPorts[board] is not None:
        openPorts[board].close()

print("Clean exit!")
