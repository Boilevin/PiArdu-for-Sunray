import os
cwd = os.getcwd()
#setting for Raspberry Pi
if (os.name=='posix'):
    print('Linux Platform')
    myComPort = 'COM3'
    myBaudRate = 115200
    myFrameWidth = 800
    myFrameHeight = 430
    myOS="Linux"

    #If PCB1.3 is used with DUE normalu always true
    DueConnectedOnPi=True
    
    #Old code possible to directly connect a GPS to PI for tracking only
    GpsConnectedOnPi=False
    GpsIsM6n=False
    
    #Old code a nano was used to control laser sensor and check env
    NanoConnectedOnPi=False
      
    #the battery cycle can be recorded on each charging cycle see plot folder
    AutoRecordBatCharging=True
    #Possible to remove the in and out NMEA message visualisation in console
    useDebugConsole=False
    
    #On multiple area it's the sender IP, Use to start and stop them over HTTP
    #Sender1AdressIP is the station sender 
    
    #Sender1AdressIP="10.0.0.150"
    #Sender2AdressIP="10.0.0.151"
    #Sender3AdressIP="10.0.0.154"
    Sender1AdressIP="0.0.0.0"
    Sender2AdressIP="0.0.0.0"
    Sender3AdressIP="0.0.0.0"
    
    #if camera is rotate in the chassis
    cameraConnected=True
    myCamAngle = 0 # Possible value = 0, 90, 180, 270
    streamVideoOnPower = False #auto start of the streaming

    #Setting for Mqtt and home automation
    useMqtt=False
    Mqtt_Broker_IP="10.0.0.8"
    Mqtt_Port = 1883
    Mqtt_User = "admin"
    Mqtt_Password = "admin"
    Mqtt_ShowDebug = False
    #delay in second between 2 sends of the DUE loops/sec over mqtt
    Mqtt_IdleFreqency = 5
    # Always first letter uppercase and set the yaml file according Mower is the defaut name
    Mqtt_MowerName = "Denna"

 #Setting for Vision
    useVision=True
    visionDetectMinScore = 0.60
    vision_framerate = 5

    max_map_inUse = 10

















#--------------------------------------------------------------
#Setting for testing with a PC 
#Do not use this parts of setting on normal config

if (os.name=='nt'):
    print('Windows Platform')
    myComPort = 'COM8'
    myFrameWidth = 800
    myFrameHeight = 430
    myBaudRate = 115200
    myOS="Windows"
    # If PCB1.3 is used with DUE normalu always true
    DueConnectedOnPi = True

    # Old code possible to directly connect a GPS to PI for tracking only
    GpsConnectedOnPi = False
    GpsIsM6n = False

    # Old code a nano was used to control laser sensor and check env
    NanoConnectedOnPi = False

    # the battery cycle can be recorded on each charging cycle see plot folder
    AutoRecordBatCharging = True
    # Possible to remove the in and out NMEA message visualisation in console
    useDebugConsole = False

    # On multiple area it's the sender IP, Use to start and stop them over HTTP
    # Sender1AdressIP is the station sender

    # Sender1AdressIP="10.0.0.150"
    # Sender2AdressIP="10.0.0.151"
    # Sender3AdressIP="10.0.0.154"
    Sender1AdressIP = "0.0.0.0"
    Sender2AdressIP = "0.0.0.0"
    Sender3AdressIP = "0.0.0.0"

    # if camera is rotate in the chassis
    cameraConnected = False
    myCamAngle = 0  # Possible value = 0, 90, 180, 270
    streamVideoOnPower = False  # auto start of the streaming

    # Setting for Mqtt and home automation
    useMqtt = False
    Mqtt_Broker_IP = "10.0.0.8"
    Mqtt_Port = 1883
    Mqtt_User = "admin"
    Mqtt_Password = "admin"
    Mqtt_ShowDebug = False
    # delay in second between 2 sends of the DUE loops/sec over mqtt
    Mqtt_IdleFreqency = 5
    # Always first letter uppercase and set the yaml file according Mower is the defaut name
    Mqtt_MowerName = "Denna"

    # Setting for Vision
    useVision = False
    visionDetectMinScore = 0.78
    vision_framerate = 5
    max_map_inUse = 10

    

