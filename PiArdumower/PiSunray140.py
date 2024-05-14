###!/usr/bin/env python3


# WARNING don't work on OS french langage because tkinter fail to manage the , decimal separator on slider or need a dot instead

PiVersion = "140"
from pathlib import Path
import traceback
import sys
import serial
import time
import datetime as dt


from backend.map import map
import networkx as nx


#from backend.map import path

# import csv
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.backend_bases import MouseButton
#import matplotlib as mpl
#mpl.rcParams["keymap.pan"]='p'

#from mpl_point_clicker import clicker
from mpl_interactions import zoom_factory, panhandler


from shapely import geometry


from shapely.ops import *
from shapely.geometry import *

#from shapely.geometry import Polygon,Point,LineString
#     from shapely import union as PolygonUnion
#     polygon2=PolygonUnion(mymower.polygon[7],mymower.polygon[4])
#     polygon3=PolygonUnion(mymower.polygon[2],mymower.polygon[3])
#     polygon9=PolygonUnion(polygon2,polygon3)
#     x,y=polygon9.exterior.xy
#     plt.plot(x,y)
#     plt.show()

import numpy as np
import subprocess
import pickle

import os
from tkinter import ttk

from tkinter import messagebox
# from tkinter import filedialog
import tkinter as tk
# import math


from config import cwd
from config import myOS
# from config import GpsConnectedOnPi
# from config import NanoConnectedOnPi
from config import DueConnectedOnPi
from config import myBaudRate
from config import myComPort
# from config import GpsIsM6n
from config import AutoRecordBatCharging
# from config import useDebugConsole
from config import useMqtt

from config import Mqtt_Broker_IP
from config import Mqtt_Port
from config import Mqtt_IdleFreqency
from config import Mqtt_MowerName
from config import Mqtt_User
from config import Mqtt_Password
from config import Mqtt_ShowDebug

from config import streamVideoOnPower
from config import Sender1AdressIP
from config import Sender2AdressIP
from config import Sender3AdressIP
from config import cameraConnected
from config import useVision
from config import visionDetectMinScore

from config import max_map_inUse

if myOS == "Linux":
    from Ps4remote import PS4Controller
    from libcamera import controls
    from picamera2 import Picamera2
    from gpiozero import CPUTemperature
# Show video in tkinter canvas but very slow
# if(useVision):
from PIL import Image, ImageTk

import threading


# matplotlib navigation vertical bar
class VerticalNavigationToolbar2Tk(NavigationToolbar2Tk):
    def __init__(self, canvas, window):
        super().__init__(canvas, window, pack_toolbar=False)

    # override _Button() to re-pack the toolbar button in vertical direction
    def _Button(self, text, image_file, toggle, command):
        b = super()._Button(text, image_file, toggle, command)
        b.pack(side=tk.TOP)  # re-pack button in vertical direction
        return b

    # override _Spacer() to create vertical separator
    def _Spacer(self):
        s = tk.Frame(self, width=26, relief=tk.RIDGE, bg="DarkGray", padx=2)
        s.pack(side=tk.TOP, pady=5)  # pack in vertical direction
        return s

    # disable showing mouse position in toolbar
    def set_message(self, s):
        pass


"""      bb file     """
from robot import *

if myOS == "Linux":
    sys.path.insert(0, "/home/pi/Documents/PiArdumower")  # add to avoid KST plot error on path


def ButtonFlashDue_click():
    global DueConnectedOnPi
    ButtonClearConsole_click()
    ConsolePage.tkraise()
    consoleInsertText("  KEEP THE POWER BUTTON PUSH DURING THE FLASH PROCESS" + '\n')
    consoleInsertText("  Teensy Loader need to start in new windows" + '\n')
    consoleInsertText("  Select a file and start flash or click on Auto mode" + '\n')
    consoleInsertText("  Teensy reboot after 8 seconds " + '\n')
    fen1.update()
    time.sleep(10)
    message = "AT+U1"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)
    DueConnectedOnPi = False
    Due_Serial.close()
    subprocess.Popen("/home/pi/Documents/PiArdumower/./teensy &", shell=True)
    time.sleep(30)
    os.remove("/home/pi/Documents/PiArdumower/Due_firmware/firmware.hex")
    DueConnectedOnPi = True
    Due_Serial.open()


#################################### CAMERA MANAGEMENT ###############################################
class streamVideo_class(object):
    """class use to start and stop the video stream"""

    def __init__(self):
        self.streamVideo = None

    def start(self, resolution):
        self.stop()

        if resolution == 0:
            self.streamVideo = subprocess.Popen(
                ["/home/pi/Documents/PiArdumower/streamVideo320.py", "shell=True", "stdout=subprocess.PIPE"])
        if resolution == 1:
            self.streamVideo = subprocess.Popen(
                ["/home/pi/Documents/PiArdumower/streamVideo640.py", "shell=True", "stdout=subprocess.PIPE"])

    def stop(self):
        if self.streamVideo:
            self.streamVideo.kill()
            self.streamVideo.wait()
            self.streamVideo = None


myStreamVideo = streamVideo_class()


#################################### KST PLOT MANAGEMENT ###############################################

class PlotterKst_class(object):
    """class use to start and stop the kst plotting prog"""

    def __init__(self):
        self.PlotterKst = None

    def start(self, fileNameKst):
        self.stop()
        self.PlotterKst = subprocess.Popen(
            ["kst2", fileNameKst, "show=maximize", "shell=True", "stdout=subprocess.PIPE"])

    def stop(self):
        if self.PlotterKst:
            self.PlotterKst.kill()
            self.PlotterKst.wait()
            self.PlotterKst = None


#################################### VARIABLE INITIALISATION ###############################################


direction_list = ['LEFT', 'RIGHT']
days_list = ['MONDAY', 'TUESDAY', 'WEDNESDAY', 'THURSDAY', 'FRIDAY', 'SATURDAY', 'SUNDAY']
page_list = ['MAIN', 'AUTO', 'MANUAL', 'SETTING', 'CONSOLE', 'TEST', 'PLOT', 'TIMER', 'VIDEO', 'GPS', 'MAPS']

motPlotterKst = PlotterKst_class()
mowPlotterKst = PlotterKst_class()
periPlotterKst = PlotterKst_class()
batPlotterKst = PlotterKst_class()
ImuPlotterKst = PlotterKst_class()

firstplotMotx = 0
firstplotMowx = 0
firstplotBatx = 0
firstplotPerx = 0
firstplotImux = 0

actualRep = os.getcwd()
dateNow = time.strftime('%d/%m/%y %H:%M:%S', time.localtime())

fen1 = tk.Tk()
"""Variable for check button """
MotVar1 = tk.IntVar()
MotVar2 = tk.IntVar()
PeriVar1 = tk.IntVar()
PeriVar2 = tk.IntVar()
PeriVar3 = tk.IntVar()
PeriVar4 = tk.IntVar()
ImuVar1 = tk.IntVar()
SonVar1 = tk.IntVar()
SonVar2 = tk.IntVar()
SonVar3 = tk.IntVar()
SonVar4 = tk.IntVar()

BatVar1 = tk.IntVar()
MowVar1 = tk.IntVar()
PlotVar1 = tk.IntVar()
CamVar1 = tk.IntVar()

tk_date_Now = tk.StringVar()
tk_time_Now = tk.StringVar()
tk_MainStatusLine = tk.StringVar()
tk_date_hour = tk.IntVar()
tk_date_minute = tk.IntVar()
tk_date_dayOfWeek = tk.IntVar()
tk_date_day = tk.IntVar()
tk_date_month = tk.IntVar()
tk_date_year = tk.IntVar()
tk_mowingPattern = tk.IntVar()

"""variable use into Auto Menu"""
tk_batteryVoltage = tk.DoubleVar()
tk_ImuYaw = tk.DoubleVar()
tk_batSense = tk.DoubleVar()
tk_ImuRoll = tk.DoubleVar()
tk_PiTemp = tk.DoubleVar()
tk_Dht22Humid = tk.DoubleVar()
tk_loopsPerSecond = tk.DoubleVar()
tk_areaToGo = tk.IntVar()
tk_areaToGo.set(1)
tk_ResumeMowing = tk.IntVar()
tk_HouseView = tk.IntVar()
tk_GpsSolution = tk.StringVar()
tk_gpsnumSV = tk.StringVar()
tk_mowPointIdx = tk.StringVar()
tk_labelStopIdx = tk.IntVar()
tk_labelStartIdx = tk.IntVar()
tk_AutoInfoMap = tk.StringVar()

"""variable use into refreh plot"""
tk_millis = tk.IntVar()
tk_motorLeftPower = tk.DoubleVar()
tk_motorRightPower = tk.DoubleVar()
tk_motorLeftPWMCurr = tk.IntVar()
tk_motorRightPWMCurr = tk.IntVar()
tk_motorMowPower = tk.DoubleVar()
tk_motorMowPWMCurr = tk.IntVar()
# tk_batteryVoltage=tk.IntVar()
tk_chgVoltage = tk.DoubleVar()
tk_chgSense = tk.DoubleVar()
tk_perimeterMag = tk.IntVar()
tk_perimeterMagRight = tk.IntVar()
tk_gyroYaw = tk.DoubleVar()
tk_compassYaw = tk.DoubleVar()

ManualKeyboardUse = tk.IntVar()
MainperimeterUse = tk.IntVar()
MainimuUse = tk.IntVar()
MaingpsUse = tk.IntVar()
MainbluetoothUse = tk.IntVar()
MainbumperUse = tk.IntVar()
MainsonarUse = tk.IntVar()
MainDHT22Use = tk.IntVar()
MainlawnSensorUse = tk.IntVar()
tk_MaintimerUse = tk.IntVar()
tk_infoTimer = tk.StringVar()
tk_MainrainUse = tk.IntVar()
tk_useDebugConsole = tk.IntVar()

MaintiltUse = tk.IntVar()

tk_rollDir = tk.StringVar()
tk_laneInUse = tk.IntVar()
tk_YawActual = tk.DoubleVar()
tk_YawCible = tk.DoubleVar()

firstFixFlag = False
firstFixDate = 0

fen1.title('SUNRAY')
fen1.geometry("800x480+0+0")


class datetime:
    def __init__(self):
        datetime.hour = 12
        datetime.minute = 0
        datetime.dayOfWeek = 0
        datetime.day = 1
        datetime.month = 1
        datetime.year = 2017


class mower:
    # char* mowPatternNames[] = {"RAND", "LANE",  "WIRE"};
    def __init__(self):
        self.millis = 0
        self.status = 0
        self.state = 0
        self.odox = 0
        self.odoy = 0
        self.prevYaw = 0
        self.batteryVoltage = 0.00
        self.chgVoltage = 0.00
        self.batSense = 0.00

        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        self.version = 'Unknow'
        self.statsOverride = 0
        self.statsMowTimeMinutesTrip = 0
        self.statsMowTimeHoursTotal = 0
        self.statsBatteryChargingCounterTotal = 0
        self.statsBatteryChargingCapacityTrip = 0
        self.statsBatteryChargingCapacityTotal = 0
        self.statsBatteryChargingCapacityAverage = 0
        # self.motorLeftSenseCurrent=0
        # self.motorRightSenseCurrent=0
        self.motorLeftPower = 0
        self.motorRightPower = 0
        self.motorLeftPWMCurr = 0
        self.motorRightPWMCurr = 0
        self.motorMowPower = 0
        self.motorMowPWMCurr = 0
        self.mowPatternCurr = 0
        self.Dht22Temp = 0
        self.Dht22Humid = 0
        self.rollDir = 0
        self.laneInUse = 0
        self.YawActual = 0
        self.YawCible = 0
        self.loopsPerSecond = 0
        self.useJoystick = False
        self.laserSensor1 = 9000
        self.laserSensor2 = 9000
        self.laserSensor3 = 9000
        self.laserSensor4 = 9000
        self.laserSensor5 = 9000
        self.laserSensor6 = 9000
        self.rainDetect = False
        self.areaInMowing = 1
        self.areaToGo = 1

        self.sigAreaOff = True

        self.timeToStartAreaSignal = 0
        self.focusOnPage = 0
        self.dueSerialReceived = ''
        self.autoRecordBatChargeOn = False

        self.mqtt_message_id = 0
        self.callback_id = 0
        self.timeToSendMqttIdle = time.time() + 20
        self.timeToReconnectMqtt = time.time() + 40
        self.oneMinuteTrigger = time.time() + 10
        self.FiveSecondTrigger = time.time() + 5
        self.OneSecondTrigger = time.time() + 1

        self.cpuFan = False
        self.lastMqttBatteryValue = 0

        self.visionDetect = False
        self.visionDetectAt = time.time() - 3
        self.visionRollRight = 1
        self.visionRunning = False
        self.surfaceDetected = 0
        self.objectDetectedID = 0
        self.userOut2 = False
        self.userOut3 = False
        self.visionScore = 0

        self.House = "00"

        self.mowPointsIdx = 0
        self.startMowPointIdx = 0
        self.stopMowPointIdx = 99999

        self.statex = 0.00
        self.statey = 0.00
        self.laststatex = 0.00
        self.laststatey = 0.00
        self.stateDelta = 0
        self.gpsSolution = 0
        self.opNames = "Idle"
        self.stateSensor = 0
        self.targetPointx = 0.00
        self.targetPointy = 0.00
        self.lasttargetPointx = 0.00
        self.lasttargetPointy = 0.00

        self.gpsAccuracy = 0
        self.gpsnumSV = 0
        self.gpsnumSVdgps = 0
        self.mapCRC = 0  # sum of point coordonnees to be sure map is not false
        self.plotMapCRC = 0  # when import a map check if it's egal to mapcrc,the one send each second
        self.lateralerror = 0

        self.perimeterPointsCount = 0
        self.exclusionPointsCount = 0
        self.dockPointsCount = 0
        self.mowPointsCount = 0
        self.freePointsCount = 0
        self.nbTotalExclusion = 0
        self.fileMapCRC = 0

        self.startPiArduTime = time.time()
        self.map = [0] * 30  # need bigger ??
        self.mapSelected = 1
        self.finishedUploadingMap = False
        self.nbTotalExclusion = 3
        self.ActiveMapX = []
        self.ActiveMapY = []
        self.polygon = [None] * 10
        self.mapNrList = []
        self.newPerimeterx = []
        self.newPerimetery = []
        self.newPerimeter = [[]]#np.zeros([], [])
        self.perimeter = np.zeros([], [])
        self.dockPts = np.zeros([], [])
        self.mowPts = np.zeros([], [])
        self.isSendindMap = False
        self.mapCrcRoundingRange = 50  # map crc is not exactly the same between pi and mower
        self.full_house = 0
        self.actualMowingArea = 0
        self.totalMowingArea = 0

        self.ActualRunningTimer = 99
        self.lastRunningTimer = 99
        self.TimerHouseToStart = 0
        self.TimerMapToStart = 0
        self.startAfterUploadFinish = False

        self.useDebugConsole = False
        self.timerUse = False
        self.rainUse = False
        self.record_perimeter =False

        self.cameraManualPageVideoStreamOn = False
        if cameraConnected:
            self.camera = Picamera2(0)


mymower = mower()
if cameraConnected:
    mymower.camera.close()

myRobot = robot()
myDate = datetime()
last_time_add_point=dt.datetime.now()

if myOS == "Linux":
    myps4 = PS4Controller()
    cpu = CPUTemperature()


def consoleInsertText(texte):
    txtConsoleRecu.insert('1.0', ' ' + texte)
    txtConsoleRecu.insert('1.0', time.strftime('%H:%M:%S', time.localtime()))


#################################### MAIN LOOP ###############################################
last_pos=Point(0,0)
def checkSerial():  # the main loop is here
  
   
    #code to record a new map
    global last_pos
    global last_time_add_point
    if (mymower.record_perimeter) :
                
        if (dt.datetime.now()-last_time_add_point).seconds >= 1:
            
            
            polygon_closed = False #check_if_polygon_is_closed
            point_too_close = False#check_if_point_is_close_to_last_one
                        
            actual_pos=Point(mymower.statex,mymower.statey)
            
            move_dist=Point.distance(actual_pos,last_pos)
            if move_dist <= 0.02 :
                point_too_close=True
                print("Too close pointt" , move_dist)

            if (not bool(polygon_closed)) and (not bool(point_too_close)):
                print("Add Pt : ",actual_pos)
                
                mymower.newPerimeterx.append(mymower.statex)
                mymower.newPerimetery.append(mymower.statey)
                last_pos=actual_pos
                #mymower.newPerimeter.append(actual_pos)
                updateNewMapsPage()


            last_time_add_point=dt.datetime.now()
                



        




    try:
        global DueConnectedOnPi
        if (DueConnectedOnPi and Due_Serial.inWaiting() != 0):
            mymower.dueSerialReceived = Due_Serial.readline()
            if str(mymower.dueSerialReceived) != "b''":
                mymower.dueSerialReceived = mymower.dueSerialReceived.decode('utf-8', errors='ignore')
                if mymower.dueSerialReceived[:1] != '$':  # it is console message because the first digit is not $
                    if (len(mymower.dueSerialReceived)) > 2:
                        # consoleInsertText(mymower.dueSerialReceived)
                        decode_AT_message(mymower.dueSerialReceived)
                else:
                    consoleInsertText("recu sentense starting by $ " + '\n')
                    consoleInsertText(mymower.dueSerialReceived)


    except Exception:
        ConsolePage.tkraise()
        DueConnectedOnPi = False
        exc_type, exc_value, exc_traceback = sys.exc_info()
        # consoleInsertText("*** print_tb:")
        traceback.print_tb(exc_traceback, limit=1, file=sys.stdout)
        traceback.print_exception(exc_type, exc_value, exc_traceback, limit=2, file=sys.stdout)
        consoleInsertText(repr(traceback.print_exc()))
        formatted_lines = traceback.format_exc().splitlines()
        consoleInsertText(formatted_lines[0])
        consoleInsertText(formatted_lines[-1])
        consoleInsertText(repr(traceback.format_exception(exc_type, exc_value, exc_traceback)))
        print("*** extract_tb:")
        print(repr(traceback.extract_tb(exc_traceback)))
        print("*** format_tb:")
        print(repr(traceback.format_tb(exc_traceback)))
        print("*** tb_lineno:", exc_traceback.tb_lineno)
        print("ERROR PLEASE CHECK TRACEBACK INFO")
        consoleInsertText("ERROR PLEASE CHECK TRACEBACK INFO" + '\n')

        if (mymower.visionRunning):
            mymower.visionRunning = False
        if (useMqtt):
            consoleInsertText('Close Mqtt Connection' + '\n')
            Mqqt_DisConnection()
        consoleInsertText('Start to save all Console Data' + '\n')
        ButtonSaveReceived_click()  # save the console txt

    if mymower.useJoystick:
        myps4.listen()
        if myps4.leftClick:
            ButtonLeft_click()
        if myps4.rightClick:
            ButtonRight_click()
        if myps4.upClick:
            send_var_message('w', 'motorSpeedMaxPwm', '' + str(manualSpeedSlider.get()) + '', '0', '0', '0', '0', '0',
                             '0', '0')
            send_pfo_message('nf', '1', '2', '3', '4', '5', '6', )
        if myps4.downClick:
            ButtonStop_click()

        # self.crossClick=False
        # self.roundClick=False
        # self.squareClick=False
        # self.triangle_click=False

        if myps4.triangleClick:
            pass
            # print("triangleClick:")
        if myps4.squareClick:
            buttonBlade_start_click()
            # print("squareClick:")
        if myps4.crossClick:
            button_stop_all_click()
            # print("crossClick:")
        if myps4.roundClick:
            buttonBlade_stop_click()
            # print("roundClick:")

    ##    if (useMqtt):
    ##        start1=time.time()
    ##        Mqqt_client.loop(0.05)
    ##        duration=time.time()-start1
    ##        if duration > 0.06 :
    ##            consoleInsertText("MQTT take more than 60 ms in execution" + '\n')
    ##        if (Mqqt_client.connected_flag):
    ##            if (time.time() > mymower.timeToSendMqttIdle):
    ##                sendMqtt(Mqtt_MowerName + "/Idle",str(mymower.loopsPerSecond))
    ##                mymower.timeToSendMqttIdle=time.time()+Mqtt_IdleFreqency
    ##
    ##        else:
    ##
    ##            if (time.time() > mymower.timeToReconnectMqtt):
    ##                consoleInsertText("MQTT not connected retry each 2 minutes" + '\n')
    ##                Mqqt_Connection()
    ##                mymower.timeToReconnectMqtt=time.time()+120
    ##
    ##    #vision detect something
    ##    if (mymower.visionDetect==True):
    ##
    ##        search_object=mymower.objectDetectedID
    ##        areaThreshold=100
    ##        scoreThreshold=0
    ##
    ##        for i in range(0,len(vision_list)):
    ##
    ##            if (str(vision_list[i][1])== str(search_object)):
    ##                areaThreshold=int(vision_list[i][3])
    ##                scoreThreshold=int(vision_list[i][2])
    ##                #print(vision_list[i])
    ##                break
    ##        if(scoreThreshold==0):
    ##            consoleInsertText("Can't find object ID into setting vision list ID : " + str(search_object) + '\n')
    ##        else:
    ##            consoleInsertText("Detected : " + str(vision_list[i][0]) + '\n')
    ##            consoleInsertText("Detected Score : " + str(int(mymower.visionScore)) + " Treshold setting : " + str(scoreThreshold) + '\n')
    ##            consoleInsertText("Detected Area : " + str(int(mymower.surfaceDetected)) + " Treshold setting : " + str(areaThreshold) + '\n')
    ##
    ##        #only stop if area and score area are OK
    ##        if ((int(mymower.surfaceDetected) >= areaThreshold) and (int(mymower.visionScore) >= scoreThreshold)):
    ##            mymower.visionDetectAt = time.time()
    ##            mymower.userOut2=True
    ##            send_pfo_message('re1','1','2','3','4','5','6',)
    ##
    ##            if(mymower.VisionRollRight == 1):
    ##                send_var_message('w','bumperRight','1','bumperLeft','0','0','0','0','0','0')
    ##            else:
    ##                send_var_message('w','bumperRight','0','bumperLeft','1','0','0','0','0','0')
    ##
    ##
    ##        else:
    ##            consoleInsertText("Object detected but size or score not enough for thresold" + '\n')
    ##
    ##
    ##        mymower.visionDetect=False
    ##
    ##    if((mymower.userOut2==True) and (time.time()>=mymower.visionDetectAt+2)):
    ##            mymower.userOut2=False
    ##            send_pfo_message('re0','1','2','3','4','5','6',)

    # one minute ticker
    if (time.time() > mymower.oneMinuteTrigger):
        if (mymower.isSendindMap):
            return
        # timer part
        if (mymower.startAfterUploadFinish != True):  # do not start mowing if upload in process
            print("timer loop")
            if ((mymower.ActualRunningTimer >= 98) and (mymower.opNames == "DOCK")):  # no timer are already mowing
                checkTimerStart()
            if (mymower.opNames == "Go TO DOCK") or (mymower.opNames == "MOW"):
                checkTimerStop()

        # clean the console page
        txtRecu.delete('2000.0', tk.END)  # keep only  lines
        txtSend.delete('2000.0', tk.END)  # keep only  lines
        txtConsoleRecu.delete('2500.0', tk.END)  # keep only  lines

        # consoleInsertText("PI Température : " + str(cpu.temperature) + '\n')
        # start or stop the Raspberry fan
        if myOS == "Linux":
            if cpu.temperature >= 65.0:
                mymower.cpuFan = True
                ButtonFanStart_click()
            else:
                if mymower.cpuFan == True:
                    mymower.cpuFan = False
                    ButtonFanStop_click()

        if (mymower.visionRunning == True) and (myRobot.statusNames[mymower.status] == "IN_STATION"):
            consoleInsertText("Station detected vision is not needed" + '\n')
            BtnVisionStop_click()
        mymower.oneMinuteTrigger = time.time() + 60

    # five seconde ticker
    if (time.time() > mymower.FiveSecondTrigger):
        if myOS == "Linux":
            tk_PiTemp.set(int(cpu.temperature))
        else:
            tk_PiTemp.set(20)
        # initialisation of auto page
        ##        if ((time.time()-mymower.startPiArduTime) < 9):
        ##            ButtonAuto_click()
        mymower.FiveSecondTrigger = time.time() + 5

    # one seconde ticker
    if (time.time() > mymower.OneSecondTrigger):
        if (mymower.isSendindMap):
            return
        if myOS == "Linux":
            tk_PiTemp.set(int(cpu.temperature))
        else:
            tk_PiTemp.set(20)
        # refresh pi data from mcu
        message = "AT+S"
        message = str(message)
        message = message + '\r'
        send_serial_message(message)
        mymower.OneSecondTrigger = time.time() + 1

    # manual page camera management
    if (cameraConnected):
        if (mymower.focusOnPage != 2) and (mymower.cameraManualPageVideoStreamOn):
            stopManualCamera()
        if (mymower.focusOnPage == 2) and not (mymower.cameraManualPageVideoStreamOn):
            startManualCamera()
        if (mymower.focusOnPage == 2) and (mymower.cameraManualPageVideoStreamOn):
            updateManualCamera()

    fen1.after(10, checkSerial)  # here is the main loop each 10ms


def updateAutoPage():
    
    if (mymower.targetPointx != mymower.lasttargetPointx):
        targetpos1 = [mymower.statex, mymower.targetPointx]
        targetpos2 = [mymower.statey, mymower.targetPointy]
        mymower.lasttargetPointx = mymower.targetPointx
        mymower.lasttargetPointy = mymower.targetPointy

        axLiveMap.plot(targetpos1, targetpos2, color='y', linewidth=1)

    mowerpos1 = [mymower.laststatex, mymower.statex]
    mowerpos2 = [mymower.laststatey, mymower.statey]

    axLiveMap.plot(mowerpos1, mowerpos2, color='g', linewidth=2)
    # axLiveMap.scatter(mymower.statex,mymower.statey, color='g', s=10)
    #axLiveMap.autoscale(False)
    canvasLiveMap.draw()


#################################### END OF MAINLOOP ###############################################
def decode_AT_message(message):  # decode sunray console message
    know_message = False
    if mymower.useDebugConsole:
        txtRecu.insert('1.0', str(message) + '\n')
    if message[:2] == 'W,':
        know_message = True
        print(str(message))
        # consoleInsertText(str(message) + '\n')
        mymower.isSendindMap = False

    if message[:2] == 'S,':  # message statistique 'AT+S'
        know_message = True
        list_recu = (str(message).split(','))

        mymower.batteryVoltage = list_recu[1]
        tk_batteryVoltage.set(mymower.batteryVoltage)
        mymower.laststatex = mymower.statex
        mymower.statex = float(list_recu[2])
        GpsInfoline1.set("StateX : " + str(mymower.statex))
        mymower.laststatey = mymower.statey
        mymower.statey = float(list_recu[3])
        GpsInfoline2.set("StateY : " + str(mymower.statey))
        mymower.stateDelta = list_recu[4]

        GpsInfoline3.set("stateDelta : " + mymower.stateDelta)
        mymower.gpsSolution = list_recu[5]
        GpsInfoline4.set("gpsSolution : " + mymower.gpsSolution)

        tk_GpsSolution.set(myRobot.gpsSolution[int(list_recu[5])])
        timeNow = time.strftime('%H:%M:%S', time.localtime())

        tk_MainStatusLine.set(timeNow + " " + myRobot.opNames[int(list_recu[6])])
        mymower.opNames = str(myRobot.opNames[int(list_recu[6])])
        mymower.mowPointsIdx = int(list_recu[7])
        tk_mowPointIdx.set(str(mymower.mowPointsIdx) + "/" + str(mymower.mowPointsCount))
        if (mymower.mowPointsIdx >= mymower.stopMowPointIdx):
            button_home_click()

        # mymower.inc=list_recu[8] need to check what is it

        mymower.stateSensor = list_recu[9]
        GpsInfoline5.set("stateSensor : " + mymower.stateSensor)
        mymower.targetPointx = float(list_recu[10])
        GpsInfoline6.set("targetPointx : " + str(mymower.targetPointx))
        mymower.targetPointy = float(list_recu[11])
        GpsInfoline7.set("targetPointy : " + str(mymower.targetPointy))
        mymower.gpsAccuracy = list_recu[12]
        GpsInfoline8.set("gpsAccuracy : " + mymower.gpsAccuracy)
        mymower.gpsnumSV = list_recu[13]

        mymower.batSense = float(list_recu[14])
        tk_batSense.set(mymower.batSense)

        mymower.gpsnumSVdgps = list_recu[15]

        GpsInfoline9.set("gpsnumSV : " + mymower.gpsnumSVdgps + "/" + mymower.gpsnumSV)
        tk_gpsnumSV.set(mymower.gpsnumSVdgps + "/" + mymower.gpsnumSV)
        if (mymower.mapCRC != int(list_recu[16])):
            search_map(int(list_recu[16]))
        else:
            mymower.mapCRC = int(list_recu[16])
            
        GpsInfoline11.set("MapCRC : " + str(mymower.mapCRC))

        mymower.lateralerror = list_recu[17]

        GpsInfoline10.set("lateralerror : " + mymower.lateralerror)

        if ((mymower.focusOnPage == 1) and ((mymower.opNames == "MOW") or (mymower.opNames == "Go TO DOCK"))):
            # mymower.ActiveMapX.append(mymower.statex)
            # mymower.ActiveMapY.append(mymower.statey)
            # if mymower.OneSecondTrigger % 2 == 0 : #only each 2 seconde
            updateAutoPage()
            #axLiveMap.set_aspect('equal', 'box')

    ##                if (myRobot.opNames[int(list_recu[6])] == "MOW"):
    ##                    f=open(cwd + "/plot/PlotPos.txt",'a+')
    ##                    f.write("{};{}\n".format(float(mymower.statex) , float(mymower.statey)))
    ##                    f.close()

    if message[:2] == 'Y3':  # message power off
        know_message = True
        mymower.focusOnPage = 4
        ConsolePage.tkraise()
        if (mymower.visionRunning):
            mymower.visionRunning = False
        if (useMqtt):
            consoleInsertText('Close Mqtt Connection' + '\n')
            Mqqt_DisConnection()
        consoleInsertText('Start to save all Console Data' + '\n')
        ButtonSaveReceived_click()  # save the console txt
        consoleInsertText('All Console Data are saved' + '\n')
        consoleInsertText('PI start Shutdown' + '\n')
        time.sleep(3)
        subprocess.Popen('/home/pi/Documents/PiArdumower/PowerOff.py')
        fen1.destroy()
        time.sleep(1)
        sys.exit("PowerOFF ordered by Arduino Due")

    if message[:2] == 'V,':  # message version 'AT+V'
        know_message = True
        list_recu = (str(message).split(','))
        # mymower.developerActive=message.developerActive
        mymower.version = list_recu[1] + " " + list_recu[2]
        Infoline1.set("Firmware : " + mymower.version + "           Pi version : " + PiVersion)
        # mymower.statsOverride=message.statsOverride
        Infoline2.set("??? : " + list_recu[3])
        Infoline3.set("??? : " + list_recu[4])
        Infoline4.set("MCU : " + list_recu[5])
        Infoline5.set("??? : " + list_recu[6])

    if message[:3] == 'RN,':  # message maps main data
        know_message = True
        list_recu = (str(message).split(','))
        # consoleInsertText(str(list_recu) + '\n')
        mymower.perimeterPointsCount = int(list_recu[1])
        mymower.exclusionPointsCount = int(list_recu[2])
        mymower.dockPointsCount = int(list_recu[3])
        mymower.mowPointsCount = int(list_recu[4])
        mymower.freePointsCount = int(list_recu[5])
        message = "AT+RNX"
        message = str(message)
        message = message + '\r'
        send_serial_message(message)

    if message[:3] == 'RNX':  # message maps main data
        know_message = True
        list_recu = (str(message).split(','))
        # consoleInsertText(str(list_recu) + '\n')
        mymower.nbTotalExclusion = int(list_recu[1])
        maindata = np.array([mymower.perimeterPointsCount, mymower.exclusionPointsCount, mymower.dockPointsCount,
                             mymower.mowPointsCount, mymower.freePointsCount, mymower.nbTotalExclusion, mymower.mapCRC])

        fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                   "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/MAIN"
        output_file = Path(fileName)
        output_file.parent.mkdir(exist_ok=True, parents=True)
        np.save(fileName, maindata, allow_pickle=True, fix_imports=True)

        # update the main CRC datalist according to map nr
        fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + "/crcMapList.npy"
        if (os.path.exists(fileName)):
            crcMapList = np.load(fileName)
            print(fileName)
            print(crcMapList)
            find_row = np.where(crcMapList[:, 1] == mymower.mapCRC)
            result = crcMapList[find_row]

            if (result.size != 0):
                if (result[0, 1] == mymower.mapCRC):
                    messagebox.showwarning('warning', "This map already exist ,it's map : " + str(result[0, 0]))
                    MapsPage.select(result[0, 0])
                    mymower.mapSelected = int(MapsPage.index("current"))
                else:
                    messagebox.showwarning('warning', "Error while loading crcMapList")

            else:
                # add the new map
                newrow = [mymower.mapSelected, mymower.mapCRC]
                crcMapList = np.vstack([crcMapList, newrow])
                np.save(fileName, crcMapList, allow_pickle=True, fix_imports=True)
                print(crcMapList)
                message = "AT+RP"
                message = str(message)
                message = message + '\r'
                send_serial_message(message)
        else:
            # create the file for the first time
            crcMapList = np.array([[mymower.mapSelected, mymower.mapCRC]])
            np.save(fileName, crcMapList, allow_pickle=True, fix_imports=True)
            message = "AT+RP"
            message = str(message)
            message = message + '\r'
            send_serial_message(message)

    if message[:2] == 'RP':  # message perimeter list point
        print(message)
        mymower.actualMowingArea = 0
        know_message = True
        pointCount = 0
        list_recu = (str(message).split(','))
        list_recu.pop(0)  # remove first
        list_recu.pop(len(list_recu) - 1)  # remove last
        pointCount = int(len(list_recu) / 2)
        dataArray = np.array(list_recu).reshape(pointCount, 2)
        mymower.map[mymower.mapSelected] = dataArray.astype(float)  # to float data
        fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                   "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/PERIMETER"
        output_file = Path(fileName)
        output_file.parent.mkdir(exist_ok=True, parents=True)
        np.save(fileName, mymower.map[mymower.mapSelected], allow_pickle=True, fix_imports=True)
        consoleInsertText('Perimeter map total point : ' + str(pointCount) + '\n')

        perimeterArray = mymower.map[mymower.mapSelected]
        polygon1 = Polygon(np.squeeze(perimeterArray))
        mymower.actualMowingArea = int(polygon1.area)

        message = "AT+RM"
        message = str(message)
        message = message + '\r'
        send_serial_message(message)

    if message[:2] == 'RM':  # message mow list point
        know_message = True
        pointCount = 0
        list_recu = (str(message).split(','))
        list_recu.pop(0)  # remove first
        list_recu.pop(len(list_recu) - 1)  # remove last
        pointCount = int(len(list_recu) / 2)
        dataArray = np.array(list_recu).reshape(pointCount, 2)
        mymower.map[mymower.mapSelected] = dataArray.astype(float)  # to float data

        fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                   "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/MOW"
        output_file = Path(fileName)
        output_file.parent.mkdir(exist_ok=True, parents=True)
        np.save(fileName, mymower.map[mymower.mapSelected], allow_pickle=True, fix_imports=True)
        consoleInsertText('Mowing map point : ' + str(pointCount) + '\n')
        message = "AT+RD"
        message = str(message)
        message = message + '\r'
        send_serial_message(message)

    if message[:2] == 'RD':  # message dock list point
        know_message = True
        pointCount = 0
        list_recu = (str(message).split(','))
        list_recu.pop(0)  # remove first
        list_recu.pop(len(list_recu) - 1)  # remove last
        pointCount = int(len(list_recu) / 2)
        dataArray = np.array(list_recu).reshape(pointCount, 2)
        mymower.map[mymower.mapSelected] = dataArray.astype(float)  # to float data
        fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                   "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/DOCK"
        output_file = Path(fileName)
        output_file.parent.mkdir(exist_ok=True, parents=True)
        np.save(fileName, mymower.map[mymower.mapSelected], allow_pickle=True, fix_imports=True)
        consoleInsertText('Docking point : ' + str(pointCount) + '\n')

        message = "AT+RF"
        message = str(message)
        message = message + '\r'
        send_serial_message(message)

    if message[:2] == 'RF':  # message free point
        know_message = True
        pointCount = 0
        list_recu = (str(message).split(','))
        list_recu.pop(0)  # remove first
        list_recu.pop(len(list_recu) - 1)  # remove last
        pointCount = int(len(list_recu) / 2)
        dataArray = np.array(list_recu).reshape(pointCount, 2)
        mymower.map[mymower.mapSelected] = dataArray.astype(float)  # to float data
        fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                   "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/FREE"
        output_file = Path(fileName)
        output_file.parent.mkdir(exist_ok=True, parents=True)
        np.save(fileName, mymower.map[mymower.mapSelected], allow_pickle=True, fix_imports=True)
        consoleInsertText('Free point : ' + str(pointCount) + '\n')
        mymower.exclusionNr = 0

        message = "AT+RX," + str(mymower.exclusionNr) + ","
        message = str(message)
        message = message + '\r'
        send_serial_message(message)

        print("   ")
        print(message)

    if message[:2] == 'RX':  # message exclusion list point
        know_message = True

        pointCount = 0
        list_recu = (str(message).split(','))
        list_recu.pop(0)  # remove first
        list_recu.pop(len(list_recu) - 1)  # remove last
        pointCount = int(len(list_recu) / 2)
        dataArray = np.array(list_recu).reshape(pointCount, 2)
        mymower.map[mymower.mapSelected] = dataArray.astype(float)  # to float data

        fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                   "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/EXCLUSION" + "{0:0>2}".format(
            mymower.exclusionNr)
        output_file = Path(fileName)
        output_file.parent.mkdir(exist_ok=True, parents=True)

        np.save(fileName, mymower.map[mymower.mapSelected], allow_pickle=True, fix_imports=True)
        consoleInsertText('Add Exclusion nr : ' + str(mymower.exclusionNr) + ' point : ' + str(pointCount) + '\n')
        print(mymower.exclusionNr, "/", mymower.nbTotalExclusion)
        mymower.exclusionNr = mymower.exclusionNr + 1

        if (mymower.exclusionNr < mymower.nbTotalExclusion):
            consoleInsertText('new exclusion : ' + str(mymower.exclusionNr) + '\n')

            message = "AT+RX," + str(mymower.exclusionNr) + ","
            message = str(message)
            message = message + '\r'
            send_serial_message(message)
        else:
            consoleInsertText('Import Finish ' + '\n')
            consoleInsertText('mapCRC = ' + str(mymower.mapCRC) + '\n')
            # canvas[mymower.mapSelected].draw
            onTabChange(mymower.mapSelected)

    if message[:2] == 'X,':  # message exclusion list point
        know_message = True
        print("message exclusion list point  ")
        print(message)
        consoleInsertText('message exclusion list point ' + '\n')
        consoleInsertText(str(message) + '\n')

    if message[:3] == 'FI,':  # message exclusion list point
        know_message = True
        mymower.finishedUploadingMap = True
        tk_HouseView.set(0)
        initActiveMap(0)
        initialPlotAutoPage(0, 99999)
        print("Map upload Finish and feedback OK  ")
        if (mymower.startAfterUploadFinish):
            print("time to start mowing")
            mymower.startAfterUploadFinish = False
            buttonStartMow_click()

    if message[:4] == 'CON:':
        know_message = True
    ##            if message[:2] == '0:':#a rectifier uniquement pour test
    ##                know_message=True

    if not (know_message):
        consoleInsertText(str(message))


def BtnMowPlotStartRec_click():
    """create an empty txt file to have correct auto legend into the graph """
    f = open(cwd + "/plot/PlotMow.txt", 'w')
    f.write("{};{};{}\n".format("Time", "motorMowPower", "motorMowPWMCurr"))
    f.write("{};{};{}\n".format("0", "0", "0"))
    f.close()
    mowPlotterKst.start('/home/pi/Documents/PiArdumower/plotMow.kst')
    send_req_message('MOW', '' + str(SldMainMowRefresh.get()) + '', '1', '10000', '0', '0',
                     '0', )  # arduino start sending data


def BtnMowPlotStopRec_click():
    global firstplotMowx
    firstplotMowx = 0  # initialise the first time plot for next plot
    mowPlotterKst.stop()  # close the kst prog

    send_req_message('MOW', '1', '0', '0', '0', '0', '0', )  # arduino stop sending data

    filename = cwd + "/plot/Plotmow" + time.strftime("%Y%m%d%H%M") + ".CSV"
    try:  # avoid error if file not exit
        os.rename(cwd + "/plot/PlotMow.txt", filename)  # keep a copy of the plot and clear the last kst file
        messagebox.showinfo('Info', "File " + filename + " created in plot directory")
    except OSError:
        print("Error when rename file ")
        consoleInsertText("Error when rename file PlotMow.txt" + '\n')
        pass

    """recreate an empty txt file to have correct auto legend into the graph 
    f=open(cwd + "/plot/PlotMow.txt",'w')
    f.write("{};{};{}\n".format("Time","motorMowPower","motorMowPWMCurr"))
    f.write("{};{};{}\n".format("0","0","0"))
    f.close()
    """


def BtnPeriPlotStartRec_click():
    """create an empty txt file to have correct auto legend into the graph """
    f = open(cwd + "/plot/PlotPeri.txt", 'w')
    f.write("{};{};{}\n".format("Time", "perimeterMag", "perimeterMagRight"))
    f.write("{};{};{}\n".format("0", "0", "0"))
    f.close()
    periPlotterKst.start('/home/pi/Documents/PiArdumower/plotPeri.kst')
    send_req_message('PERI', '' + str(SldMainPeriRefresh.get()) + '', '1', '10000', '0', '0', '0', )


def BtnPeriPlotStopRec_click():
    global firstplotPerx
    firstplotPerx = 0
    periPlotterKst.stop()  # close the kst prog

    send_req_message('PERI', '1', '0', '0', '0', '0', '0', )

    filename = cwd + "/plot/PlotPeri" + time.strftime("%Y%m%d%H%M") + ".CSV"
    try:
        os.rename(cwd + "/plot/PlotPeri.txt", filename)  # keep a copy of the plot and clear the last kst file
        messagebox.showinfo('Info', "File " + filename + " created in plot directory")
    except OSError:
        print("Error when rename file /plot/PlotPeri.txt")
        consoleInsertText("Error when rename file /plot/PlotPeri.txt" + '\n')
        pass


def BtnBatPlotStartRec_click():
    # create an empty txt file to have correct auto legend into the graph """
    f = open(cwd + "/plot/PlotBat.txt", 'w')
    f.write("{};{};{};{}\n".format("Time", "chgVoltage", "chgSense", "batteryVoltage"))
    f.write("{};{};{};{}\n".format("0", "0", "0", "0"))
    f.close()
    # if(mymower.autoRecordBatChargeOn!=True):#it's not the auto record so need to start KST
    batPlotterKst.start('/home/pi/Documents/PiArdumower/plotBat.kst')
    send_req_message('BAT', '' + str(SldMainBatRefresh.get()) + '', '1', '10000', '0', '0', '0', )


def BtnBatPlotStopRec_click():
    global firstplotBatx
    firstplotBatx = 0
    batPlotterKst.stop()  # close the kst prog

    send_req_message('BAT', '1', '0', '0', '0', '0', '0', )

    filename = cwd + "/plot/PlotBattery" + time.strftime("%Y%m%d%H%M") + ".CSV"
    try:
        os.rename(cwd + "/plot/PlotBat.txt", filename)  # keep a copy of the plot and clear the last kst file
        # //bber17
        # if(mymower.autoRecordBatChargeOn==False): #it's not the auto record so send a message
        # messagebox.showinfo('Info',"File " + filename + " created in plot directory")
    except OSError:
        print("Error when rename file /plot/PlotBat.txt")
        consoleInsertText("Error when rename file /plot/PlotBat.txt" + '\n')


def BtnImuPlotStartRec_click():
    """create an empty txt file to have correct auto legend into the graph """
    f = open(cwd + "/plot/PlotImu.txt", 'w')
    f.write("{};{};{}\n".format("Time", "GyroYaw", "CompassYaw"))
    f.write("{};{};{}\n".format("0", "0", "0"))
    f.close()
    ImuPlotterKst.start('/home/pi/Documents/PiArdumower/plotImu.kst')
    send_req_message('IMU', '' + str(SldMainImuRefresh.get()) + '', '1', '10000', '0', '0', '0', )


def BtnImuPlotStopRec_click():
    global firstplotImux
    firstplotImux = 0
    ImuPlotterKst.stop()  # close the kst prog

    send_req_message('IMU', '1', '0', '0', '0', '0', '0', )

    filename = cwd + "/plot/PlotImu" + time.strftime("%Y%m%d%H%M") + ".CSV"
    try:
        os.rename(cwd + "/plot/PlotImu.txt", filename)  # keep a copy of the plot and clear the last kst file
        messagebox.showinfo('Info', "File " + filename + " created in plot directory")
    except OSError:
        print("Error when rename file /plot/PlotImu.txt")
        consoleInsertText("Error when rename file /plot/PlotImu.txt" + '\n')
        pass


def BtnBylaneStartRec_click():
    send_req_message('BYL', '3', '1', '6000', '0', '0', '0', )


def BtnBylaneStopRec_click():
    send_req_message('BYL', '1', '0', '0', '0', '0', '0', )


def BtnMotPlotStartRec_click():
    """create an empty txt file to have correct auto legend into the graph """
    f = open(cwd + "/plot/PlotMot.txt", 'w')
    f.write(
        "{};{};{};{};{}\n".format("Time", "motorLeftPower", "motorRightPower", "motorLeftPWMCurr", "motorRightPWMCurr"))
    f.write("{};{};{};{};{}\n".format("0", "0", "0", "0", "0"))
    f.close()
    motPlotterKst.start('/home/pi/Documents/PiArdumower/plotMot.kst')
    send_req_message('MOT', '' + str(SldMainWheelRefresh.get()) + '', '1', '10000', '0', '0', '0', )


def BtnMotPlotStopRec_click():
    global firstplotMotx
    firstplotMotx = 0
    motPlotterKst.stop()  # close the kst prog
    send_req_message('MOT', '1', '0', '0', '0', '0', '0', )
    filename = cwd + "/plot/PlotMotor" + time.strftime("%Y%m%d%H%M") + ".CSV"
    try:
        os.rename(cwd + "/plot/PlotMot.txt", filename)  # keep a copy of the plot and clear the last kst file
        messagebox.showinfo('Info', "File " + filename + " created in plot directory")
    except OSError:
        print("Error when rename file /plot/PlotMot.txt")
        consoleInsertText("Error when rename file /plot/PlotMot.txt" + '\n')
        pass


def BtnMotPlotStopAll_click():
    BtnMotPlotStopRec_click()
    BtnBatPlotStopRec_click()
    BtnPeriPlotStopRec_click()
    BtnMowPlotStopRec_click()
    BtnImuPlotStopRec_click()


def refreshByLaneSettingPage():
    sliderDistBetweenLane.set(myRobot.DistBetweenLane)
    slidermaxLenghtByLane.set(myRobot.maxLenghtByLane)
    slideryawSet1.set(myRobot.yawSet1)
    slideryawOppositeLane1RollRight.set(myRobot.yawOppositeLane1RollRight)
    slideryawOppositeLane1RollLeft.set(myRobot.yawOppositeLane1RollLeft)
    slideryawSet2.set(myRobot.yawSet2)
    slideryawOppositeLane2RollRight.set(myRobot.yawOppositeLane2RollRight)
    slideryawOppositeLane2RollLeft.set(myRobot.yawOppositeLane2RollLeft)
    slideryawSet3.set(myRobot.yawSet3)
    slideryawOppositeLane3RollRight.set(myRobot.yawOppositeLane3RollRight)
    slideryawOppositeLane3RollLeft.set(myRobot.yawOppositeLane3RollLeft)


def refreshMowMotorSettingPage():
    # slidermowPatternDurationMax.set(myRobot.mowPatternDurationMax)
    slidermotorMowSpeedMaxPwm.set(myRobot.motorMowSpeedMaxPwm)
    slidermotorMowSpeedMinPwm.set(myRobot.motorMowSpeedMinPwm)
    slidermotorMowfaultcurrent.set(myRobot.motorMowfaultcurrent)
    slidermotorMowOverloadCurrent.set(myRobot.motorMowOverloadCurrent)


##    slidermotorMowSenseScale.set(myRobot.motorMowSenseScale)
##    slidermotorMowPID_Kp.set(myRobot.motorMowPID_Kp)
##    slidermotorMowPID_Ki.set(myRobot.motorMowPID_Ki)
##    slidermotorMowPID_Kd.set(myRobot.motorMowPID_Kd)
##
##    ChkBtnmotorMowForceOff.deselect()
##    if myRobot.motorMowForceOff=='1':
##        ChkBtnmotorMowForceOff.select()

def refreshBatterySettingPage():
    sliderbatGoHomeIfBelow.set(myRobot.batGoHomeIfBelow)
    sliderbatSwitchOffIfBelow.set(myRobot.batSwitchOffIfBelow)
    sliderbatSwitchOffIfIdle.set(myRobot.batSwitchOffIfIdle)
    sliderstartChargingIfBelow.set(myRobot.startChargingIfBelow)

    sliderbatFullCurrent.set(myRobot.batFullCurrent)


##    sliderbatFactor.set(myRobot.batFactor)

##    sliderbatChgFactor.set(myRobot.batChgFactor)
##    sliderbatSenseFactor.set(myRobot.batSenseFactor)
##
##    ChkBtnbatMonitor.deselect()
##    if myRobot.batMonitor=='1':
##        ChkBtnbatMonitor.select()


def refreshImuSettingPage():
    sliderimuDirPID_Kp.set(myRobot.imuDirPID_Kp)
    sliderimuDirPID_Ki.set(myRobot.imuDirPID_Ki)
    sliderimuDirPID_Kd.set(myRobot.imuDirPID_Kd)

    sliderdelayBetweenTwoDmpAutocalib.set(myRobot.delayBetweenTwoDmpAutocalib)
    slidermaxDurationDmpAutocalib.set(myRobot.maxDurationDmpAutocalib)
    slidermaxDriftPerSecond.set(myRobot.maxDriftPerSecond)

    ChkBtnstopMotorDuringCalib.deselect()
    if myRobot.stopMotorDuringCalib == '1':
        ChkBtnstopMotorDuringCalib.select()


##def refreshTimerSettingPage():
##
##
##    for i in range(5):
##
##        tk_timerActive[i].set(myRobot.Timeractive[i])
##        tk_timerStartTimehour[i].set(myRobot.TimerstartTime_hour[i])
##        tk_timerStartTimeMinute[i].set(myRobot.TimerstartTime_minute[i])
##        tk_timerStopTimehour[i].set(myRobot.TimerstopTime_hour[i])
##        tk_timerStopTimeMinute[i].set(myRobot.TimerstopTime_minute[i])
##        tk_timerHouse[i].set(myRobot.TimerstartHouse[i])
##        tk_TimerstartMap[i].set(myRobot.TimerstartMap[i])
##        tk_timerStartNrLane[i].set(myRobot.TimerstartNrLane[i])
##        tk_timerStartRollDir[i].set(myRobot.TimerstartRollDir[i])
##        tk_timerStartLaneMaxlengh[i].set(myRobot.TimerstartLaneMaxlengh[i])
##        tk_timerStartArea[i].set(myRobot.TimerstartArea[i])
##
##        for j in range(7):
##            result=[bool((myRobot.TimerdaysOfWeek[i]) & (1<<n)) for n in range(8)]
##            tk_timerDayVar[i][j].set(result[j])
##
##


def refreshMotorSettingPage():
    # manualSpeedSlider.set(myRobot.motorSpeedMaxPwm)
    sliderPowerMax.set(myRobot.motorPowerMax)


##    sliderSpeedRpmMax.set(myRobot.motorSpeedMaxRpm)
##    sliderSpeedPwmMax.set(myRobot.motorSpeedMaxPwm)
##    sliderAccel.set(myRobot.motorAccel)
##    sliderPowerIgnoreTime.set(myRobot.motorPowerIgnoreTime)
##    sliderRollDegMax.set(myRobot.motorRollDegMax)
##    sliderRollDegMin.set(myRobot.motorRollDegMin)
##    sliderRevDist.set(myRobot.DistPeriOutRev)
##    sliderStopDist.set(myRobot.DistPeriOutStop)
##    sliderPidP.set(myRobot.motorLeftPID_Kp)
##    sliderPidI.set(myRobot.motorLeftPID_Ki)
##    sliderPidD.set(myRobot.motorLeftPID_Kd)
##    ChkBtnMotorSwapLeftDir.deselect()
##    ChkBtnMotorSwapRightDir.deselect()
##    if myRobot.motorLeftSwapDir=='1':
##        ChkBtnMotorSwapLeftDir.select()
##    if myRobot.motorRightSwapDir=='1':
##        ChkBtnMotorSwapRightDir.select()
##    sliderRightFwOffset.set(myRobot.motorRightOffsetFwd)
##    sliderRightRevOffset.set(myRobot.motorRightOffsetRev)
##    sliderSpeedOdoMin.set(myRobot.SpeedOdoMin)
##    sliderSpeedOdoMax.set(myRobot.SpeedOdoMax)
##    sliderLeftSense.set(myRobot.motorSenseLeftScale)
##    sliderRightSense.set(myRobot.motorSenseRightScale)
# ButtonSendSettingToDue_click

def refreshMainSettingPage():
    ChkBtntimerUse.deselect()
    tk_MaintimerUse.set(0)
    if (mymower.timerUse):
        tk_MaintimerUse.set(1)
        ChkBtntimerUse.select()
    ChkBtnrainUse.deselect()
    tk_MainrainUse.set(0)
    if (mymower.rainUse):
        tk_MainrainUse.set(1)
        ChkBtnrainUse.select()
    ChkBtnuseDebugConsole.deselect()
    tk_useDebugConsole.set(0)
    if (mymower.useDebugConsole):
        ChkBtnuseDebugConsole.select()
        tk_useDebugConsole.set(1)


def ButtonSaveReceived_click():
    fileName = cwd + "/log/" + time.strftime("%Y%m%d%H%M") + "_Received.txt"
    with open(fileName, "w") as f:
        f.write(txtRecu.get('1.0', 'end'))
    fileName = cwd + "/log/" + time.strftime("%Y%m%d%H%M") + "_Send.txt"
    with open(fileName, "w") as f:
        f.write(txtSend.get('1.0', 'end'))
    fileName = cwd + "/log/" + time.strftime("%Y%m%d%H%M") + "_Console.txt"
    with open(fileName, "w") as f:
        f.write(txtConsoleRecu.get('1.0', 'end'))

    consoleInsertText('All Console file are saved' + '\n')


def button_stop_all_click():
    mymower.startAfterUploadFinish = False
    mymower.lastRunningTimer = mymower.ActualRunningTimer
    mymower.ActualRunningTimer = 99
    message = "AT+C,-1,0,-1,-1,-1,-1,-1,-1"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


def ButtonInfo_click():
    message = "AT+V"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)
    mymower.focusOnPage = 11
    InfoPage.tkraise()

def NewMapsPage_click():
    mymower.focusOnPage = 12
    NewMapsPage.tkraise()


def ButtonCamera_click():
    mymower.focusOnPage = 8
    StreamVideoPage.tkraise()


##def ButtonGps_click():
##    mymower.focusOnPage=9
##    GpsPage.tkraise()

def ButtonMaps_click():
    mymower.focusOnPage = 9

    MapsPage.tkraise()
    MapsPage.select(0)
    # MapsPage.select(mymower.mapSelected)


def ButtonSchedule_click():
    mymower.focusOnPage = 7
    TabTimer.tkraise()


def ButtonManual_click():
    manualSpeedSlider.set(myRobot.motorSpeedMaxRpm)
    mymower.focusOnPage = 2
    ManualPage.tkraise()


def ButtonPlot_click():
    mymower.focusOnPage = 6
    TabPlot.tkraise()


def ButtonSetting_click():
    mymower.focusOnPage = 3
    TabSetting.tkraise()


def ButtonBackToMain_click():
    mymower.focusOnPage = 0
    MainPage.tkraise()


def ButtonConsole_click():
    mymower.focusOnPage = 4
    ConsolePage.tkraise()


def ButtonTest_click():
    mymower.focusOnPage = 5
    TestPage.tkraise()
    
def search_map(map_crc_ToFind):
    # try to find the map crc in the raspberry pi and init eveything
    # check if the map locate into mower have changed
    mymower.mapCRC = map_crc_ToFind
    map_find = False
    for search_house in range(10):
        print("search into house nr : ",search_house)

        fileName = cwd + "/House" + "{0:0>2}".format(search_house) + "/crcMapList.npy"
        if ((os.path.exists(fileName)) & (mymower.mapCRC != 0)):
            crcMapList = np.load(fileName)
            for ip in range(int(len(crcMapList))):
##                print("ctrl")
##                print(crcMapList[ip,1])
##                print(mymower.mapCRC)
                ctrl = crcMapList[ip, 1] - map_crc_ToFind
                # print(ctrl)
                if (abs(ctrl) < mymower.mapCrcRoundingRange):
                    mymower.mapSelected = int(crcMapList[ip, 0])
                    mymower.House=int(search_house)
                    print("House : ",mymower.House," Map selected : ",mymower.mapSelected)
                    map_find = True
                    if (mymower.full_house == 0):
                        initActiveMap(0)
                        initialPlotAutoPage(mymower.startMowPointIdx, mymower.stopMowPointIdx)
                    else:
                        rebuildHouseMap()
                        # initActiveMap(1)
                        # initialPlotAutoPageFullHouse()
                    return

    if (map_find == False):
        mymower.mapCRC=0
        messagebox.showwarning('warning', "Map unknow (try to upload, see map page)" + str(mymower.mapCRC))



def ButtonAuto_click():
    if (mymower.full_house == 0):
        initActiveMap(0)
        initialPlotAutoPage(mymower.startMowPointIdx, mymower.stopMowPointIdx)
    else:
        rebuildHouseMap()
        # initActiveMap(1)
        # initialPlotAutoPageFullHouse()
      
    
    mymower.focusOnPage = 1
    AutoPage.tkraise()

    

##        initActiveMap(1)
##        initialPlotAutoPageFullHouse()


def ButtonSendSettingByLaneToDue_click():
    myRobot.yawSet1 = slideryawSet1.get()
    myRobot.yawSet2 = slideryawSet2.get()
    myRobot.yawSet3 = slideryawSet3.get()


def ButtonSendSettingDateTimeToDue_click():
    Send_reqSetting_message('Time', 'w', '1', '' + str(tk_date_hour.get()) + \
                            '', '' + str(tk_date_minute.get()) + \
                            '', '' + str(tk_date_dayOfWeek.get()) + \
                            '', '' + str(tk_date_day.get()) + \
                            '', '' + str(tk_date_month.get()) + \
                            '', '' + str(tk_date_year.get()) + \
                            '', '' + str(0) + \
                            '', '' + str(0) + \
                            '', '' + str(0) + \
                            '', '' + str(0) + '', )


def ButtonSendSettingToDue_click():
    consoleInsertText("All Setting are change into the Due but not save for the moment" + '\n')


def read_all_setting():
    Send_reqSetting_message('All', 'r', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0')


def read_time_setting():
    Send_reqSetting_message('Time', 'r', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0')


def send_req_message(val1, val2, val3, val4, val5, val6, val7):
    send_serial_message(message)


def send_var_message(val1, val2, val3, val4, val5, val6, val7, val8, val9, val10):
    send_serial_message(message)


def send_cmd_message(val1, val2, val3, val4, val5):
    send_serial_message(message)


def send_pfo_message(val1, val2, val3, val4, val5, val6, val7):
    message = pynmea2.PFO('RM', 'PFO', (val1, val2, val3, val4, val5, val6, val7))
    message = str(message)
    message = message + '\r' + '\n'
    send_serial_message(message)


def Send_reqSetting_message(val1, val2, val3, val4, val5, val6, val7, val8, val9, val10, val11, val12, val13):
    send_serial_message(message)


def send_serial_message(message1):
    try:
        if DueConnectedOnPi:
            # checkSerial()
            Due_Serial.flush()
            Due_Serial.write(bytes(message1, 'utf-8'))

            if mymower.useDebugConsole:
                txtSend.insert('1.0', message1)

    except:
        print("ERREUR while transfert")
        time.sleep(2)


""" ------------------- connecting the the PCB  ------------ """
try:

    if DueConnectedOnPi:
        if myOS == "Linux":
            if os.path.exists('/dev/ttyACM0') == True:
                Due_Serial = serial.Serial('/dev/ttyACM0', 115200, timeout=2, write_timeout=2)
                Due_Serial.flushInput()
                Due_Serial.flushOutput()  # clear the output buffer
                print("Find Serial on ttyACM0")

            if os.path.exists('/dev/ttyACM1') == True:
                Due_Serial = serial.Serial('/dev/ttyACM1', 115200, timeout=10, write_timeout=10)
                Due_Serial.flushInput()
                Due_Serial.flushOutput()  # clear the output buffer
                print("Find Serial on ttyACM1")
        else:
            Due_Serial = serial.Serial(myComPort, myBaudRate, timeout=0)





except:
    print(" ")
    print("************************************")
    print("ERREUR DE CONNECTION WITH PCB1.3 DUE")
    print("************************************")
    print(" ")
    #consoleInsertText("ERREUR DE CONNECTION WITH PCB"  + '\n')

    time.sleep(1)
    DueConnectedOnPi=False
    # sys.exit("Impossible de continuer")

"""-------------------ICONS LOADING---------------------"""
imgHome = tk.PhotoImage(file=cwd + "/icons/home.png")
imgTrack = tk.PhotoImage(file=cwd + "/icons/track.png")
imgStopAll = tk.PhotoImage(file=cwd + "/icons/stop all.png")
imgstartMow = tk.PhotoImage(file=cwd + "/icons/startmow.png")
imgBack = tk.PhotoImage(file=cwd + "/icons/back.png")
imgBladeStop = tk.PhotoImage(file=cwd + "/icons/bladeoff.png")
imgBladeStart = tk.PhotoImage(file=cwd + "/icons/bladeon.png")
imgForward = tk.PhotoImage(file=cwd + "/icons/forward.png")
imgForwardLeft = tk.PhotoImage(file=cwd + "/icons/forLeft.png")
imgForwardRight = tk.PhotoImage(file=cwd + "/icons/forRight.png")

imgReverse = tk.PhotoImage(file=cwd + "/icons/reverse.png")
imgLeft = tk.PhotoImage(file=cwd + "/icons/left.png")
imgRight = tk.PhotoImage(file=cwd + "/icons/right.png")
imgStop = tk.PhotoImage(file=cwd + "/icons/stop.png")
imgArdumower = tk.PhotoImage(file=cwd + "/icons/ardumower.png")
imgManual = tk.PhotoImage(file=cwd + "/icons/manual.png")
imgAuto = tk.PhotoImage(file=cwd + "/icons/auto.png")
imgTest = tk.PhotoImage(file=cwd + "/icons/test.png")
imgConsole = tk.PhotoImage(file=cwd + "/icons/console.png")
imgSetting = tk.PhotoImage(file=cwd + "/icons/setting.png")
imgPowerOff = tk.PhotoImage(file=cwd + "/icons/off.png")
imgPlot = tk.PhotoImage(file=cwd + "/icons/plot.png")
imgSchedule = tk.PhotoImage(file=cwd + "/icons/schedule.png")
imgCamera = tk.PhotoImage(file=cwd + "/icons/camera.png")
imgGps = tk.PhotoImage(file=cwd + "/icons/gps.png")
imgJoystickON = tk.PhotoImage(file=cwd + "/icons/joystick_on.png")
imgJoystickOFF = tk.PhotoImage(file=cwd + "/icons/joystick_off.png")
imgJoystick = tk.PhotoImage(file=cwd + "/icons/joystick.png")
imgMaps = tk.PhotoImage(file=cwd + "/icons/map.png")

""" THE SETTING PAGE ****************************************************"""


def ButtonSaveSettingToFile_click():
    setting_list = []

    ButtonSetOdometryApply_click()
    setting_list.append(myRobot.odometryTicksPerRevolution)
    setting_list.append(myRobot.odometryWheelDiameter)
    setting_list.append(myRobot.odometryWheelBaseCm)

    ButtonSetMainApply_click()
    setting_list.append(LinearSpeedSlider.get())
    setting_list.append(DockingSpeedSlider.get())
    setting_list.append(mymower.timerUse)
    setting_list.append(mymower.rainUse)

    ButtonSetMotApply_click()
    setting_list.append(sliderPowerMax.get())

    ButtonSetMowMotorApply_click()
    setting_list.append(myRobot.motorMowSpeedMaxPwm)
    setting_list.append(myRobot.motorMowSpeedMinPwm)
    setting_list.append(myRobot.motorMowfaultcurrent)

    ButtonSetBatteryApply_click()
    setting_list.append(myRobot.batGoHomeIfBelow)
    setting_list.append(myRobot.batSwitchOffIfBelow)
    setting_list.append(myRobot.batSwitchOffIfIdle)
    setting_list.append(myRobot.startChargingIfBelow)
    setting_list.append(myRobot.batFullCurrent)
    setting_list.append(mymower.useDebugConsole)

    print(setting_list)

    with open("setting_list.bin", "wb") as fp:
        pickle.dump(setting_list, fp)
        print("setting file saved")


def ButtonReadSettingFromFile_click():
    setting_list = []
    print("read setting file")
    with open("setting_list.bin", "rb") as fp:
        setting_list = pickle.load(fp)
    myRobot.odometryTicksPerRevolution = setting_list[0]
    myRobot.odometryWheelDiameter = setting_list[1]
    myRobot.odometryWheelBaseCm = setting_list[2]
    refreshOdometrySettingPage()

    LinearSpeedSlider.set(setting_list[3])
    DockingSpeedSlider.set(setting_list[4])
    mymower.timerUse = setting_list[5]
    mymower.rainUse = setting_list[6]

    myRobot.motorPowerMax = setting_list[7]
    refreshMotorSettingPage()

    myRobot.motorMowSpeedMaxPwm = setting_list[8]
    myRobot.motorMowSpeedMinPwm = setting_list[9]
    myRobot.motorMowfaultcurrent = setting_list[10]
    refreshMowMotorSettingPage()

    myRobot.batGoHomeIfBelow = setting_list[11]
    myRobot.batSwitchOffIfBelow = setting_list[12]
    myRobot.batSwitchOffIfIdle = setting_list[13]
    myRobot.startChargingIfBelow = setting_list[14]
    myRobot.batFullCurrent = setting_list[15]
    refreshBatterySettingPage()

    mymower.useDebugConsole = setting_list[16]
    # print("rr",mymower.useDebugConsole)
    refreshMainSettingPage()


def refreshOdometrySettingPage():
    sliderodometryTicksPerRevolution.set(myRobot.odometryTicksPerRevolution)
    sliderodometryWheelDiameter.set(myRobot.odometryWheelDiameter)
    sliderodometryWheelBaseCm.set(myRobot.odometryWheelBaseCm)


def refreshAllSettingPage():
    refreshMotorSettingPage()
    refreshPerimeterSettingPage()
    refreshMainSettingPage()
    refreshImuSettingPage()

    refreshBatterySettingPage()
    refreshOdometrySettingPage()
    refreshMowMotorSettingPage()
    refreshByLaneSettingPage()
    # refreshTimerSettingPage()


def ButtonSetMowMotorApply_click():
    myRobot.motorMowSpeedMaxPwm = slidermotorMowSpeedMaxPwm.get()
    myRobot.motorMowSpeedMinPwm = slidermotorMowSpeedMinPwm.get()
    myRobot.motorMowfaultcurrent = slidermotorMowfaultcurrent.get()
    myRobot.motorMowOverloadCurrent = slidermotorMowOverloadCurrent.get()


def ButtonSetMotApply_click():
    myRobot.motorPowerMax = sliderPowerMax.get()


def ButtonSetBatteryApply_click():
    myRobot.batGoHomeIfBelow = sliderbatGoHomeIfBelow.get()
    myRobot.batSwitchOffIfBelow = sliderbatSwitchOffIfBelow.get()
    myRobot.batSwitchOffIfIdle = sliderbatSwitchOffIfIdle.get()
    myRobot.startChargingIfBelow = sliderstartChargingIfBelow.get()
    myRobot.batFullCurrent = sliderbatFullCurrent.get()


def ButtonSetOdometryApply_click():
    myRobot.odometryTicksPerRevolution = sliderodometryTicksPerRevolution.get()
    myRobot.odometryWheelDiameter = sliderodometryWheelDiameter.get()
    myRobot.odometryWheelBaseCm = sliderodometryWheelBaseCm.get()


def ButtonSetImuApply_click():
    myRobot.imuDirPID_Kp = sliderimuDirPID_Kp.get()
    myRobot.imuDirPID_Ki = sliderimuDirPID_Ki.get()
    myRobot.imuDirPID_Kd = sliderimuDirPID_Kd.get()
    myRobot.delayBetweenTwoDmpAutocalib = sliderdelayBetweenTwoDmpAutocalib.get()
    myRobot.maxDurationDmpAutocalib = slidermaxDurationDmpAutocalib.get()
    myRobot.maxDriftPerSecond = slidermaxDriftPerSecond.get()

    myRobot.stopMotorDuringCalib = '0'
    if ImuVar1.get() == 1:
        myRobot.stopMotorDuringCalib = '1'
    ButtonSendSettingToDue_click()


def ButtonSetMainApply_click():
    mymower.timerUse = False
    if tk_MaintimerUse.get() == 1:
        mymower.timerUse = True
    mymower.rainUse = False
    if tk_MainrainUse.get() == 1:
        mymower.rainUse = True
    mymower.useDebugConsole = False
    if tk_useDebugConsole.get() == 1:
        mymower.useDebugConsole = True


TabSetting = ttk.Notebook(fen1)
TabSetting.place(x=0, y=0)

# TabConsole=ttk.Notebook(fen1)
# img1=tk.PhotoImage(file="/pi/Ardumawer/img/setting1.png")
tabMain = tk.Frame(TabSetting, width=800, height=380)
tabWheelMotor = tk.Frame(TabSetting, width=800, height=380)
tabMowMotor = tk.Frame(TabSetting, width=800, height=380)
# tabPerimeter=tk.Frame(TabSetting,width=800,height=380)
# tabImu=tk.Frame(TabSetting,width=800,height=380)
# tabSonar=tk.Frame(TabSetting,width=800,height=380)
tabBattery = tk.Frame(TabSetting, width=800, height=380)
tabOdometry = tk.Frame(TabSetting, width=800, height=380)
# tabDateTime=tk.Frame(TabSetting,width=800,height=380)
# tabByLane=tk.Frame(TabSetting,width=800,height=380)
tabVision = tk.Frame(TabSetting, width=800, height=380)

TabSetting.add(tabMain, text="Main")
TabSetting.add(tabWheelMotor, text="Wheels Motor")
TabSetting.add(tabMowMotor, text="Mow Motor")
# TabSetting.add(tabPerimeter,text="Perimeter")
# TabSetting.add(tabImu,text="Imu")
# TabSetting.add(tabSonar,text="Sonar")
TabSetting.add(tabBattery, text="Battery")
TabSetting.add(tabOdometry, text="Odometry")
# TabSetting.add(tabDateTime,text="Date Time")
# TabSetting.add(tabByLane,text="ByLane")
TabSetting.add(tabVision, text="Vision")

"""************* Main setting *****************************"""


def updateChkBtnrainUse(event):
    ButtonSetMainApply_click()


def updateBtnuseDebugConsole(event):
    ButtonSetMainApply_click()


def updateChkBtntimerUse(event):
    ButtonSetMainApply_click()


def updateDockingSpeedSlider(event):
    print(str(DockingSpeedSlider.get()))


##   message="AT+CT,14," + str(DockingSpeedSlider.get()) +",0"
##   message=str(message)
##   message=message + '\r'
##   send_serial_message(message)


ButtonSaveSettingToFile = tk.Button(tabMain)
ButtonSaveSettingToFile.place(x=30, y=15, height=25, width=120)
ButtonSaveSettingToFile.configure(command=ButtonSaveSettingToFile_click)
ButtonSaveSettingToFile.configure(text="Save")

ButtonReadSettingFromFile = tk.Button(tabMain)
ButtonReadSettingFromFile.place(x=30, y=65, height=25, width=120)
ButtonReadSettingFromFile.configure(command=ButtonReadSettingFromFile_click)
ButtonReadSettingFromFile.configure(text="Read")

ButtonFlashDue = tk.Button(tabMain)
ButtonFlashDue.place(x=0, y=175, height=30, width=200)
ButtonFlashDue.configure(command=ButtonFlashDue_click)
ButtonFlashDue.configure(text="Update ROBOT Firmware")

##def ButtonReboot_click():
##    #01/09/2023 actualy nothing dev here because reboot can power off PCB and pi
##    print("reboot")
##ButtonReboot = tk.Button(tabMain,text="Reboot All",  command = ButtonReboot_click)
##ButtonReboot.place(x=30,y=115, height=40, width=100)

LinearSpeedSlider = tk.Scale(tabMain, orient='horizontal', label='Linear SPEED', relief=tk.SOLID, resolution=0.1,
                             from_=0.0, to=0.8)
LinearSpeedSlider.place(x=270, y=10, width=200, height=55)
LinearSpeedSlider.set(0.4)

DockingSpeedSlider = tk.Scale(tabMain, orient='horizontal', label='Docking SPEED', relief=tk.SOLID, resolution=0.1,
                              from_=0.0, to=0.8)
DockingSpeedSlider.place(x=270, y=65, width=200, height=55)
DockingSpeedSlider.set(0.2)
DockingSpeedSlider.bind("<ButtonRelease-1>", updateDockingSpeedSlider)

ChkBtntimerUse = tk.Checkbutton(tabMain, text="Use Timer", relief=tk.SOLID, variable=tk_MaintimerUse, onvalue=1,
                                offvalue=0, anchor='nw')
ChkBtntimerUse.place(x=530, y=40, width=250, height=20)
ChkBtntimerUse.bind("<ButtonRelease-1>", updateChkBtntimerUse)

ChkBtnrainUse = tk.Checkbutton(tabMain, text="Use Rain Sensor", relief=tk.SOLID, variable=tk_MainrainUse, onvalue=1,
                               offvalue=0, anchor='nw')
ChkBtnrainUse.place(x=530, y=70, width=250, height=20)
ChkBtnrainUse.bind("<ButtonRelease-1>", updateChkBtnrainUse)

ChkBtnuseDebugConsole = tk.Checkbutton(tabMain, text="Debug console", relief=tk.SOLID, variable=tk_useDebugConsole,
                                       onvalue=1, offvalue=0, anchor='nw')
ChkBtnuseDebugConsole.place(x=530, y=100, width=250, height=20)
ChkBtnuseDebugConsole.bind("<ButtonRelease-1>", updateBtnuseDebugConsole)

# ButtonSetMainApply = tk.Button(tabMain)
# ButtonSetMainApply.place(x=300,y=350, height=25, width=100)
# ButtonSetMainApply.configure(command = ButtonSetMainApply_click,text="Send To Mower")


ButtonBackHome = tk.Button(TabSetting, image=imgBack, command=ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=120, width=120)

"""************* Mow motor setting *****************************"""


def updateslidermotorMowSpeedMaxPwm(event):
    message = "AT+CT,20," + str(slidermotorMowSpeedMaxPwm.get()) + ",0"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


def updateslidermotorMowSpeedMinPwm(event):
    message = "AT+CT,21," + str(slidermotorMowSpeedMinPwm.get()) + ",0"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


def updateslidermotorMowfaultcurrent(event):
    message = "AT+CT,22," + str(slidermotorMowfaultcurrent.get()) + ",0"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


def updateslidermotorMowOverloadCurrent(event):
    message = "AT+CT,23," + str(slidermotorMowOverloadCurrent.get()) + ",0"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


##
##
##    case 21:
##              motor.motorMowfaultcurrent = floatValue;
##              break;
##            case 22:
##              motor.motorMowOverloadCurrent = floatValue;
##              break;


slidermotorMowSpeedMaxPwm = tk.Scale(tabMowMotor, from_=100, to=255, label='Max PWM Speed', relief=tk.SOLID,
                                     orient='horizontal')
slidermotorMowSpeedMaxPwm.place(x=10, y=10, width=250, height=50)
slidermotorMowSpeedMaxPwm.bind("<ButtonRelease-1>", updateslidermotorMowSpeedMaxPwm)

slidermotorMowSpeedMinPwm = tk.Scale(tabMowMotor, from_=100, to=255, label='Min PWM Speed', relief=tk.SOLID,
                                     orient='horizontal')
slidermotorMowSpeedMinPwm.place(x=10, y=60, width=250, height=50)
slidermotorMowSpeedMinPwm.bind("<ButtonRelease-1>", updateslidermotorMowSpeedMinPwm)

slidermotorMowfaultcurrent = tk.Scale(tabMowMotor, from_=1, to=15, label='Max Current in Amp', relief=tk.SOLID,
                                      orient='horizontal')
slidermotorMowfaultcurrent.place(x=270, y=10, width=250, height=50)
slidermotorMowfaultcurrent.bind("<ButtonRelease-1>", updateslidermotorMowfaultcurrent)

slidermotorMowOverloadCurrent = tk.Scale(tabMowMotor, from_=1, to=15, label='Overload Current in Amp', relief=tk.SOLID,
                                         orient='horizontal')
slidermotorMowOverloadCurrent.place(x=270, y=60, width=250, height=50)
slidermotorMowOverloadCurrent.bind("<ButtonRelease-1>", updateslidermotorMowOverloadCurrent)

"""************* Odometry setting *****************************"""


def updatesliderodometryTicksPerRevolution(event):
    message = "AT+CT,4," + str(sliderodometryTicksPerRevolution.get()) + ",0"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


def updatesliderodometryWheelBaseCm(event):
    message = "AT+CT,5," + str(sliderodometryWheelBaseCm.get()) + ",0"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


def updatesliderodometryWheelDiameter(event):
    message = "AT+CT,6," + str(sliderodometryWheelDiameter.get()) + ",0"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


sliderodometryTicksPerRevolution = tk.Scale(tabOdometry, from_=500, to=2000, label='Ticks for one full revolution',
                                            relief=tk.SOLID, orient='horizontal')
sliderodometryTicksPerRevolution.place(x=10, y=10, width=300, height=50)
sliderodometryTicksPerRevolution.bind("<ButtonRelease-1>", updatesliderodometryTicksPerRevolution)
sliderodometryWheelDiameter = tk.Scale(tabOdometry, from_=100, to=350, label='Wheels diameters in mm  ',
                                       relief=tk.SOLID, orient='horizontal')
sliderodometryWheelDiameter.place(x=10, y=60, width=300, height=50)
sliderodometryWheelDiameter.bind("<ButtonRelease-1>", updatesliderodometryWheelDiameter)
sliderodometryWheelBaseCm = tk.Scale(tabOdometry, from_=0, to=50, label='Distance between the 2 Wheels Motor (CM)',
                                     relief=tk.SOLID, orient='horizontal')
sliderodometryWheelBaseCm.place(x=10, y=110, width=300, height=50)
sliderodometryWheelBaseCm.bind("<ButtonRelease-1>", updatesliderodometryWheelBaseCm)

"""************* Battery setting *****************************"""


def updatesliderbatGoHomeIfBelow(event):
    message = "AT+CT,11," + str(sliderbatGoHomeIfBelow.get()) + ",0"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


def updatesliderbatSwitchOffIfBelow(event):
    message = "AT+CT,12," + str(sliderbatSwitchOffIfBelow.get()) + ",0"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


def updatesliderbatSwitchOffIfIdle(event):
    message = "AT+CT,9," + str(60*sliderbatSwitchOffIfIdle.get()) + ",0"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


def updatesliderstartChargingIfBelow(event):
    message = "AT+CT,8," + str(sliderstartChargingIfBelow.get()) + ",0"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


def updatesliderbatFullCurrent(event):
    message = "AT+CT,10," + str(sliderbatFullCurrent.get()) + ",0"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


sliderbatGoHomeIfBelow = tk.Scale(tabBattery, from_=20, to=30, resolution=0.1, label='Go Home if Voltage Below ',
                                  relief=tk.SOLID, orient='horizontal')
# sliderbatGoHomeIfBelow = ttk.Scale(tabBattery, from_=20.0, to=30.0)
sliderbatGoHomeIfBelow.place(x=10, y=10, width=250, height=50)
sliderbatGoHomeIfBelow.bind("<ButtonRelease-1>", updatesliderbatGoHomeIfBelow)

sliderbatSwitchOffIfBelow = tk.Scale(tabBattery, from_=20, to=30, resolution=0.1, label='All OFF if Voltage Below ',
                                     relief=tk.SOLID, orient='horizontal')
sliderbatSwitchOffIfBelow.place(x=10, y=60, width=250, height=50)
sliderbatSwitchOffIfBelow.bind("<ButtonRelease-1>", updatesliderbatSwitchOffIfBelow)

sliderbatSwitchOffIfIdle = tk.Scale(tabBattery, from_=5, to=60, label='All OFF After this Delay in Minute',
                                    relief=tk.SOLID, orient='horizontal')
sliderbatSwitchOffIfIdle.place(x=10, y=110, width=250, height=50)
sliderbatSwitchOffIfIdle.bind("<ButtonRelease-1>", updatesliderbatSwitchOffIfIdle)

sliderstartChargingIfBelow = tk.Scale(tabBattery, from_=20, to=30, resolution=0.1,
                                      label='Start Charging if Voltage Below ', relief=tk.SOLID, orient='horizontal')
sliderstartChargingIfBelow.place(x=10, y=160, width=250, height=50)
sliderstartChargingIfBelow.bind("<ButtonRelease-1>", updatesliderstartChargingIfBelow)

sliderbatFullCurrent = tk.Scale(tabBattery, from_=0, to=2, resolution=0.1,
                                label='Start mowing if Charge Current Below ', relief=tk.SOLID, orient='horizontal')
sliderbatFullCurrent.place(x=10, y=210, width=250, height=50)
sliderbatFullCurrent.bind("<ButtonRelease-1>", updatesliderbatFullCurrent)

"""************* SONAR setting *****************************"""

##ChkBtnsonarCenterUse=tk.Checkbutton(tabSonar, text="Use Sonar center",relief=tk.SOLID,variable=SonVar1,anchor='nw')
##ChkBtnsonarCenterUse.place(x=10,y=10,width=250, height=20)
##ChkBtnsonarLeftUse=tk.Checkbutton(tabSonar, text="Use Sonar Left",relief=tk.SOLID,variable=SonVar2,anchor='nw')
##ChkBtnsonarLeftUse.place(x=10,y=40,width=250, height=20)
##ChkBtnsonarRightUse=tk.Checkbutton(tabSonar, text="Use Sonar Right",relief=tk.SOLID,variable=SonVar3,anchor='nw')
##ChkBtnsonarRightUse.place(x=10,y=70,width=250, height=20)
##
##ChkBtnsonarLikeBumper=tk.Checkbutton(tabSonar, text="Sonar like a Bumper",relief=tk.SOLID,variable=SonVar4,anchor='nw')
##ChkBtnsonarLikeBumper.place(x=10,y=100,width=250, height=20)
##
##slidersonarTriggerBelow = tk.Scale(tabSonar,orient='horizontal',relief=tk.SOLID, from_=20, to=150, label='Detect Below in CM ')
##slidersonarTriggerBelow.place(x=10,y=130,width=250, height=50)
##
##slidersonarToFront = tk.Scale(tabSonar,orient='horizontal',relief=tk.SOLID, from_=0, to=100, label='Sonar to mower Front in CM ')
##slidersonarToFront.place(x=10,y=200,width=250, height=50)


"""************* Motor setting *****************************"""

sliderPowerMax = tk.Scale(tabWheelMotor, orient='horizontal', relief=tk.SOLID, from_=0, to=100,
                          label='Power Max in Watt (0 to 100)')
sliderPowerMax.place(x=5, y=10, width=250, height=50)

"""************* Vision setting *****************************"""

# exemple of vision_list=[['person',1,80,15],['chair',52,85,18,]]
# vision_list=[['person',1,80,15],['chair',52,85,18]]
# Read the file and create the list
with open("vision_list.bin", "rb") as fp:
    vision_list = pickle.load(fp)
    print("Vision file OK")


def OnClick_vision(app):
    try:
        itemVision = treeVision.selection()[0]
        txtVisionObject.delete('1.0', 'end')
        txtVisionObject.insert('1.0', vision_list[treeVision.item(itemVision, "text")][0])
        txtVisionID.delete('1.0', 'end')
        txtVisionID.insert('1.0', vision_list[treeVision.item(itemVision, "text")][1])
        txtVisionScore.delete('1.0', 'end')
        txtVisionScore.insert('1.0', vision_list[treeVision.item(itemVision, "text")][2])
        txtVisionArea.delete('1.0', 'end')
        txtVisionArea.insert('1.0', vision_list[treeVision.item(itemVision, "text")][3])

    except:
        print("Please click on correct line")


# Building the treeVisionView
treeVision = ttk.Treeview(tabVision)
scrollbarVision1 = ttk.Scrollbar(treeVision, orient="vertical", command=treeVision.yview)
scrollbarVision1.pack(side=tk.RIGHT, fill=tk.Y)
treeVision.configure(yscrollcommand=scrollbarVision1.set)

minwidth = treeVision.column('#0', option='minwidth')
treeVision.column('#0', width=minwidth)

treeVision["columns"] = ("0", "1", "2", "3")
treeVision.column("0", width=150)
treeVision.column("1", width=60, anchor='center')
treeVision.column("2", width=60, anchor='center')
treeVision.column("3", width=60, anchor='center')

treeVision.heading("0", text="Name")
treeVision.heading("1", text="Object ID ")
treeVision.heading("2", text=" % Score")
treeVision.heading("3", text=" % Image Area")

treeVision.bind("<ButtonRelease-1>", OnClick_vision)
treeVision.place(x=0, y=0, height=300, width=520)


def rebuild_treeVisionview():
    for i in treeVision.get_children():
        treeVision.delete(i)
    for i in range(0, len(vision_list)):
        treeVision.insert("", 'end', text=i,
                          values=(vision_list[i][0], vision_list[i][1], vision_list[i][2], vision_list[i][3]))


def save_VisionList():
    with open("vision_list.bin", "wb") as fp:
        pickle.dump(vision_list, fp)


def del_vision_line():
    curr = treeVision.focus()
    if '' == curr: return
    search_object = txtVisionObject.get("1.0", 'end-1c')

    for i in range(0, len(vision_list)):
        if (str(vision_list[i][0]) == str(search_object)):
            for value in vision_list[:]:
                if (value[0] == search_object):
                    vision_list.remove(value)
                    print("remov ok")

            break

    rebuild_treeVisionview()


def add_vision_line():
    maVisionList = [txtVisionObject.get("1.0", 'end-1c'), txtVisionID.get("1.0", 'end-1c'),
                    txtVisionScore.get("1.0", 'end-1c'), txtVisionArea.get("1.0", 'end-1c')]
    vision_list.append(maVisionList)
    rebuild_treeVisionview()


def vison_test():
    search_code = (b'dog')
    for i in range(0, len(vision_list)):
        if (str("b'" + vision_list[i][0] + "'") == str(search_code)):
            print("trouv")
            print(vision_list[i])


tk.Label(tabVision, text="Name:", font=("Arial", 10), fg='green').place(x=530, y=5, height=20, width=80)
tk.Label(tabVision, text="ID:", font=("Arial", 10), fg='green').place(x=530, y=25, height=20, width=80)
tk.Label(tabVision, text="Score:", font=("Arial", 10), fg='green').place(x=530, y=45, height=20, width=80)
tk.Label(tabVision, text="Area:", font=("Arial", 10), fg='green').place(x=530, y=65, height=20, width=80)

txtVisionObject = tk.Text(tabVision)
txtVisionObject.place(x=630, y=5, width=120, height=20)
txtVisionObject.delete('1.0', 'end')
txtVisionObject.insert('1.0', vision_list[0][0])
txtVisionID = tk.Text(tabVision)
txtVisionID.place(x=630, y=25, width=120, height=20)
txtVisionID.delete('1.0', 'end')
txtVisionID.insert('1.0', vision_list[0][1])
txtVisionScore = tk.Text(tabVision)
txtVisionScore.place(x=630, y=45, width=120, height=20)
txtVisionScore.delete('1.0', 'end')
txtVisionScore.insert('1.0', vision_list[0][2])
txtVisionArea = tk.Text(tabVision)
txtVisionArea.place(x=630, y=65, width=120, height=20)
txtVisionArea.delete('1.0', 'end')
txtVisionArea.insert('1.0', vision_list[0][3])

ButtonVisionAdd = tk.Button(tabVision, text="Add ", command=add_vision_line)
ButtonVisionAdd.place(x=530, y=170, height=30, width=110)
ButtonVisionDel = tk.Button(tabVision, text="Delete ", command=del_vision_line)
ButtonVisionDel.place(x=660, y=170, height=30, width=100)
ButtonVisionSave = tk.Button(tabVision, text="Save to File", command=save_VisionList)
ButtonVisionSave.place(x=530, y=210, height=60, width=140)

# ButtonVisionTest = tk.Button(tabVision, text="seek test",command = vison_test)
# ButtonVisionTest.place(x=530, y=260, height=60, width=140)

rebuild_treeVisionview()

""" THE AUTO PAGE ***************************************************"""
from matplotlib.artist import Artist
# def onPickPoint(event):
#     ind = event.ind
#     print('onpickPoint scatter:', ind )
    


 
# def onPickPoint(event):
#     global selectedPoint
    
#     line = event.artist
#     xdata, ydata = line.get_data()
#     ind = event.ind
#     print('on pick line: ',ind," " , np.array([xdata[ind], ydata[ind]]))
#     selectedPoint = event.artist
#     if selectedPoint != None:
#         props = { 'color' : "Red" }
#         #Artist.update(event.ind, props)
#         canvasLiveMap.draw()   
    

def BtnTextHouseNrLeft_click():
    mymower.House = int(mymower.House) - 1
    if (mymower.House <= 0):
        mymower.House = 0
    textHouseNr.configure(text=mymower.House)
    rebuildHouseMap()


def BtnTextHouseNrRight_click():
    mymower.House = int(mymower.House) + 1
    if (mymower.House >= 10):
        mymower.House = 10
    textHouseNr.configure(text=mymower.House)
    rebuildHouseMap()


def rebuildHouseMap():
    mymower.full_house = 1
    tk_HouseView.set(1)
    initialPlotAutoPageFullHouse()


def button_home_click():
    message = "AT+C,-1,4,-1,-1,-1,-1,-1,1"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)
    mymower.lastRunningTimer = mymower.ActualRunningTimer
    mymower.ActualRunningTimer = 99


def buttonStartMow_click():
    # mow en ,op,speed,timeout,finishandrestart,mowpercent,skipNextMowingPoint,sonar.enabled

    message = "AT+C,-1,1," + str(LinearSpeedSlider.get()) + ",100,0," + str(AutoSliderStart.get() / 100) + ",0,0,208"
    message = str(message)
    message = message + '\r'
    print(message)
    send_serial_message(message)


def updatesAutoSliderStart(event):
    mymower.startMowPointIdx = int(mymower.mowPointsCount * AutoSliderStart.get() / 100)
    if (mymower.startMowPointIdx < 0):
        mymower.startMowPointIdx = 0
    if (int(AutoSliderStart.get()) >= int(AutoSliderStop.get())):
        AutoSliderStop.set(AutoSliderStart.get())
        mymower.stopMowPointIdx = mymower.startMowPointIdx + 1
        tk_labelStopIdx.set(mymower.stopMowPointIdx)

    tk_labelStartIdx.set(mymower.startMowPointIdx)
    initialPlotAutoPage(mymower.startMowPointIdx, mymower.stopMowPointIdx)


def updatesAutoSliderStop(event):
    mymower.stopMowPointIdx = int(mymower.mowPointsCount * int(AutoSliderStop.get()) / 100)
    tk_labelStopIdx.set(mymower.stopMowPointIdx)
    initialPlotAutoPage(mymower.startMowPointIdx, mymower.stopMowPointIdx)


def updatesRdBtn_HouseView():
    
    if tk_HouseView.get() == 1:
        mymower.full_house = 1
        print("house view")
        initialPlotAutoPageFullHouse()

    else:
        mymower.full_house = 0
        print("Map view")
        initialPlotAutoPage(mymower.startMowPointIdx, mymower.stopMowPointIdx)


def updatesRdBtn_Resume():
    if tk_ResumeMowing.get() == 1:
        AutoSliderStart.place_forget()
        AutoPercentLabel.place_forget()
        AutoPercentLabel.text = ''
        AutoSliderStart.set(-1)
        AutoSliderStop.set(100)



    else:
        AutoSliderStart.place(x=430, y=175, width=40, height=170)
        AutoPercentLabel.place(x=445, y=155)
    mymower.startMowPointIdx = mymower.mowPointsIdx
    initialPlotAutoPage(mymower.startMowPointIdx, mymower.stopMowPointIdx)
    tk_labelStartIdx.set(mymower.mowPointsIdx)
    tk_labelStopIdx.set(mymower.mowPointsCount)


AutoPage = tk.Frame(fen1)
AutoPage.place(x=0, y=0, height=400, width=800)

batteryFrame = tk.Frame(AutoPage)
batteryFrame.place(x=430, y=10, height=80, width=100)
batteryFrame.configure(borderwidth="3", relief=tk.GROOVE, background="#d9d9d9", highlightbackground="#d9d9d9",
                       highlightcolor="black")

tk.Label(batteryFrame, text="BATTERY", fg='green', font=("Arial", 12)).pack(side='top', anchor='n')
tk.Label(batteryFrame, textvariable=tk_batSense, fg='red', font=("Arial", 12)).pack(side='bottom', anchor='n')
tk.Label(batteryFrame, textvariable=tk_batteryVoltage, fg='red', font=("Arial", 20)).pack(side='bottom', anchor='n')

temperatureFrame = tk.Frame(AutoPage)
temperatureFrame.place(x=430, y=95, height=60, width=100)
temperatureFrame.configure(borderwidth="3", relief=tk.GROOVE, background="#d9d9d9", highlightbackground="#d9d9d9",
                           highlightcolor="black")
tk.Label(temperatureFrame, text="TEMPERATURE", fg='green').pack(side='top', anchor='n')
tk.Label(temperatureFrame, textvariable=tk_PiTemp, fg='red', font=("Arial", 20)).pack(side='bottom', anchor='n')

GpsFrame = tk.Frame(AutoPage)
GpsFrame.place(x=570, y=145, width=100, height=80)
GpsFrame.configure(borderwidth="3", relief=tk.GROOVE, background="#d9d9d9", highlightbackground="#d9d9d9",
                   highlightcolor="black")
tk.Label(GpsFrame, text="GPS", fg='green', font=("Arial", 12)).place(x=25, y=1)
tk.Label(GpsFrame, textvariable=tk_GpsSolution, fg='red', font=("Arial", 20)).pack(side='bottom', anchor='center')
tk.Label(GpsFrame, textvariable=tk_gpsnumSV, fg='black', font=("Arial", 10)).pack(side='bottom', anchor='center')

PosFrame = tk.Frame(AutoPage)
PosFrame.place(x=570, y=228, width=100, height=50)
PosFrame.configure(borderwidth="3", relief=tk.GROOVE, background="#d9d9d9", highlightbackground="#d9d9d9",
                   highlightcolor="black")
tk.Label(PosFrame, text="POS", fg='green', font=("Arial", 12)).place(x=25, y=1)
tk.Label(PosFrame, textvariable=tk_mowPointIdx, fg='black', font=("Arial", 10)).pack(side='bottom', anchor='center')

# creation of the canvas for map view
FrameLiveMap = tk.Frame(AutoPage, borderwidth="1", relief=tk.SOLID)
FrameLiveMap.place(x=5, y=5, height=330, width=360)

figLiveMap, axLiveMap = plt.subplots()
#bb100
#zoom_factory(axLiveMap)
#ph = panhandler(figLiveMap, button=2)
#axLiveMap.autoscale(False)

canvasLiveMap = FigureCanvasTkAgg(figLiveMap, master=FrameLiveMap)
canvasLiveMap.get_tk_widget().place(x=0, y=0, width=360, height=330)

axLiveMap.plot(mymower.ActiveMapX, mymower.ActiveMapY, color='r', linewidth=0.4, marker='.', markersize=2)

canvasLiveMap.draw()

#cid2 = canvasLiveMap.mpl_connect('pick_event', onPickPoint)

# fig, ax = plt.subplots()
# ax.plot(np.random.rand(10))

def onMapclick(event):
    
    print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
          ('double' if event.dblclick else 'single', event.button,
           event.x, event.y, event.xdata, event.ydata))

    Point_X = event.xdata
    Point_Y = event.ydata

   #axLiveMap.reset_position()
   # axLiveMap.drag_pan(1, event.key, event.x, event.y)






   
    #axLiveMap.set_position([Point_X,Point_Y,20,20])

    if (mymower.full_house == 1) & (event.dblclick):
        point = geometry.Point(Point_X, Point_Y)

        for idx in range(int(len(mymower.mapNrList))):
            mapNr = int(mymower.mapNrList[idx])
            if (mymower.polygon[mapNr].contains(point)):
                print(mapNr)
                returnval = messagebox.askyesno('Info', "Upload map " + str(mapNr) + " to robot")
                if (returnval):
                    mymower.mapSelected = mapNr
                    export_map_to_mower(mymower.House, mymower.mapSelected)

                    tk_HouseView.set(0)
                    initActiveMap(0)
                    initialPlotAutoPage(0, 99999)
                    return

    # figLiveMap.canvas.mpl_disconnect(cid)


cid = figLiveMap.canvas.mpl_connect('button_press_event', onMapclick)

toolbarLiveMap = VerticalNavigationToolbar2Tk(canvasLiveMap, AutoPage)
toolbarLiveMap.children['!button2'].pack_forget()
toolbarLiveMap.children['!button3'].pack_forget()
toolbarLiveMap.children['!button4'].pack_forget()

toolbarLiveMap.update()
toolbarLiveMap.place(x=370, y=5, height=150, width=30)


def initActiveMap(full_house):
    # main data

    if (full_house == 1):
        print("init full map " + str(mymower.full_house))

    fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
               "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/MAIN.npy"

    if (os.path.exists(fileName)):
        maindata = np.load(fileName)
        mymower.perimeterPointsCount = int(maindata[0])
        mymower.exclusionPointsCount = int(maindata[1])
        mymower.dockPointsCount = int(maindata[2])
        mymower.mowPointsCount = int(maindata[3])
        mymower.freePointsCount = int(maindata[4])
        mymower.nbTotalExclusion = int(maindata[5])
        mymower.fileMapCRC = int(maindata[6])
    else:
        messagebox.showwarning('warning', "No main data for this map ???")

    fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
               "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/PERIMETER.npy"
    if (os.path.exists(fileName)):
        mymower.perimeter = np.load(fileName)
    else:
        messagebox.showwarning('warning', "No perimeter for this map ???")

    fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
               "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/MOW.npy"
    if (os.path.exists(fileName)):
        mymower.mowPts = np.load(fileName)
    else:
        messagebox.showwarning('warning', "No mowing points for this map ???")

    fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
               "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/DOCK.npy"
    if (os.path.exists(fileName)):
        mymower.dockPts = np.load(fileName)
    textHouseNr.config(text=mymower.House)
    tk_AutoInfoMap.set("House " + "{0:0>2}".format(mymower.House) + " Total Area " + str(mymower.totalMowingArea) + " m2")
    

def initialPlotAutoPageFullHouse():
    
    if (mymower.mapSelected == 0):
        return
    mymower.mapNrList.clear
    axLiveMap.clear()

    fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + "/crcMapList.npy"
    mymower.totalMowingArea = 0
    if (os.path.exists(fileName)):
        crcMapList = np.load(fileName)
        # print(crcMapList)
        for idx in range(int(len(crcMapList))):
            mapNr = int(crcMapList[idx, 0])
            mymower.mapNrList.append(mapNr)
            fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                       "/maps" + "{0:0>2}".format(mapNr) + "/PERIMETER.npy"
            if (os.path.exists(fileName)):
                perimeterArray = np.load(fileName)
                polygon1 = Polygon(np.squeeze(perimeterArray))

                mymower.polygon[mapNr] = polygon1  # keep the polygon for later search

                # pp3 = plt.Polygon(np.squeeze(perimeterArray))
                # axLiveMap.add_patch(pp3)

                # print(str(mapNr)+ " " +str(int(polygon1.area)))
                mymower.totalMowingArea = mymower.totalMowingArea + int(
                    polygon1.area)  # print(polygon1.centroid.coords[0][0]) center of polygon
                # draw perimeter
                x = np.zeros(int(len(perimeterArray) + 1))
                y = np.zeros(int(len(perimeterArray) + 1))
                for idx1 in range(int(len(perimeterArray))):
                    x[idx1] = perimeterArray[idx1][0]
                    y[idx1] = perimeterArray[idx1][1]
                    # close the drawing
                x[idx1 + 1] = perimeterArray[0][0]
                y[idx1 + 1] = perimeterArray[0][1]
                axLiveMap.text(polygon1.centroid.coords[0][0], polygon1.centroid.coords[0][1], mapNr, fontsize=8,
                               bbox=dict(facecolor='yellow', alpha=0.4))
                axLiveMap.plot(x, y, color='r', linewidth=0.4)  # ,marker='.',markersize=2)

            else:
                print("no perimeter data for this map " + str(mapNr))

            if (mapNr == mymower.mapSelected):
                fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                           "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/MOW.npy"

                if (os.path.exists(fileName)):
                    mowPts = np.load(fileName)
                    x = np.zeros(int(len(mowPts)))
                    y = np.zeros(int(len(mowPts)))
                    for ip in range(int(len(mowPts))):
                        x[ip] = mowPts[ip][0]
                        y[ip] = mowPts[ip][1]

                    axLiveMap.plot(x, y, color='black', linewidth=0.2)

                else:
                    messagebox.showwarning('warning', "No mowing points for this map ?????")
        textHouseNr.config(text=mymower.House)
        tk_AutoInfoMap.set(
            "House " + "{0:0>2}".format(mymower.House) + " Total Area " + str(mymower.totalMowingArea) + " m2")

    else:
        textHouseNr.config(text=mymower.House)
        tk_AutoInfoMap.set("House " + "{0:0>2}".format(mymower.House) + " NO MAP ")

        print("error no crcMapList.npy for this house")

    
    canvasLiveMap.draw()

    # print(mymower.totalMowingArea)


def initialPlotAutoPage(start, stop):
   
    if (mymower.mapSelected == 0):
        print("no map selected into initialPlotAutoPage")
        return

    axLiveMap.clear()
    if (mymower.opNames == "MOW"):
        mymower.startMowPointIdx = mymower.mowPointsIdx
    # reduce len of the array according start and stop
    # bug here
    array_size = int(len(mymower.mowPts) - mymower.startMowPointIdx)  # -(len(mymower.mowPts)-mymower.stopMowPointIdx)
    # print(array_size)
    x_lon = np.zeros(array_size)
    y_lat = np.zeros(array_size)
    idx_tab = 0

    for ip in range(int(len(mymower.mowPts))):
        if ((ip >= mymower.startMowPointIdx) and (ip <= mymower.stopMowPointIdx)):
            x_lon[idx_tab] = float(mymower.mowPts[ip][0])
            y_lat[idx_tab] = float(mymower.mowPts[ip][1])
            idx_tab = idx_tab + 1
    x_lon[x_lon != 0.00]  # try to remove 0 value ???????????
    y_lat[y_lat != 0.00]
    # print(x_lon)
    # print(y_lat)
    axLiveMap.plot(x_lon, y_lat, color='black', linewidth=0.2)#,picker=True,pickradius=5)


    
    # draw perimeter
    x_lon = np.zeros(int(len(mymower.perimeter) + 1))
    y_lat = np.zeros(int(len(mymower.perimeter) + 1))
    for ip in range(int(len(mymower.perimeter))):
        x_lon[ip] = mymower.perimeter[ip][0]
        y_lat[ip] = mymower.perimeter[ip][1]
        # close the drawing
    x_lon[ip + 1] = mymower.perimeter[0][0]
    y_lat[ip + 1] = mymower.perimeter[0][1]

    # perimeterArray=mymower.map[mymower.mapSelected]
    polygon1 = Polygon(np.squeeze(mymower.perimeter))
    mymower.actualMowingArea = int(polygon1.area)

    axLiveMap.plot(x_lon, y_lat, color='r', linewidth=0.6,picker=True,pickradius=5)  # ,marker='.',markersize=2)
    #figLiveMap.subplots_adjust(left=0, right=1, top=1, bottom=0)

    # draw dock
    #print("dock point ",mymower.dockPts.T)
    x_lon = np.zeros(int(len(mymower.dockPts)))
    y_lat = np.zeros(int(len(mymower.dockPts)))
    for ip in range(int(len(mymower.dockPts))):
        x_lon[ip] = mymower.dockPts[ip][0]
        y_lat[ip] = mymower.dockPts[ip][1]
    axLiveMap.plot(x_lon, y_lat, color='b', linewidth=0.4,picker=True,pickradius=3,marker='.',markersize=2)
    # print(str(x_lon[ip])+' station '+str(y_lat[ip]) )
    # print(str(mymower.statex)+' pos     '+str(mymower.statey) )

    axLiveMap.scatter(x_lon[ip], y_lat[ip], color='r', s=20)
    axLiveMap.scatter(float(mymower.statex), float(mymower.statey), color='g', s=10)

    tk_AutoInfoMap.set(
        "House " + "{0:0>2}".format(mymower.House) + " Map " + "{0:0>2}".format(mymower.mapSelected) + " Area " + str(
            mymower.actualMowingArea) + " m2")
    
    
    
    

    figLiveMap.subplots_adjust(left=0, right=1, top=1, bottom=0)
    
    # axLiveMap.
    canvasLiveMap.draw()
   


tk_AutoInfoMap.set("No Map")
AutoInfoMapLabel = tk.Label(AutoPage, textvariable=tk_AutoInfoMap, font=("Arial", 10))
AutoInfoMapLabel.place(x=150, y=335)

tk_infoTimer.set("")
AutoInfoMapTimer = tk.Label(AutoPage, textvariable=tk_infoTimer, font=("Arial", 10))
AutoInfoMapTimer.place(x=150, y=360)

AutoSliderStart = tk.Scale(AutoPage, orient='vertical', resolution=1, font=("Arial", 8), from_=-1, to=100)
AutoSliderStart.place(x=430, y=175, width=40, height=170)
AutoSliderStart.set(0)
AutoLabelStartIdx = tk.Label(AutoPage, textvariable=tk_labelStartIdx, font=("Arial", 8)).place(x=453, y=342)

AutoSliderStop = tk.Scale(AutoPage, orient='vertical', resolution=1, font=("Arial", 8), from_=0, to=100)
AutoSliderStop.place(x=470, y=175, width=40, height=170)
AutoSliderStop.set(100)
AutoLabelStopIdx = tk.Label(AutoPage, textvariable=tk_labelStopIdx, font=("Arial", 8)).place(x=493, y=342)

AutoSliderStart.bind("<ButtonRelease-1>", updatesAutoSliderStart)
AutoSliderStop.bind("<ButtonRelease-1>", updatesAutoSliderStop)
AutoPercentLabel = tk.Label(AutoPage, text='Start  Stop', fg='green')
AutoPercentLabel.place(x=445, y=155)

RdBtn_Resume = tk.Checkbutton(AutoPage, text="Resume last session", font=("Arial", 10), variable=tk_ResumeMowing,command=updatesRdBtn_Resume)
RdBtn_Resume.place(x=515, y=290, width=150, height=20)
#RdBtn_Resume.bind("<ButtonRelease-1>", updatesRdBtn_Resume)


RdBtn_HouseView = tk.Checkbutton(AutoPage, text="House view", font=("Arial", 10), variable=tk_HouseView,command=updatesRdBtn_HouseView)
RdBtn_HouseView.place(x=5, y=335, width=90, height=15)
#RdBtn_HouseView.bind("<ButtonRelease-1>", updatesRdBtn_HouseView)

textHouseNr = tk.Label(AutoPage, borderwidth=1, relief="solid", text=int(mymower.House), font=("Arial", 12), fg='green')
textHouseNr.place(x=34, y=355, height=30, width=40)
BtnTextHouseNrLeft = tk.Button(AutoPage, font=("Arial", 18, 'bold'), text="<", command=BtnTextHouseNrLeft_click)
BtnTextHouseNrLeft.place(x=5, y=355, height=30, width=30)
BtnTextHouseNrRight = tk.Button(AutoPage, font=("Arial", 18, 'bold'), text=">", command=BtnTextHouseNrRight_click)
BtnTextHouseNrRight.place(x=73, y=355, height=30, width=30)

ButtonStartMow = tk.Button(AutoPage, image=imgstartMow, command=buttonStartMow_click)
ButtonStartMow.place(x=570, y=10, width=100, height=130)
Buttonhome = tk.Button(AutoPage, image=imgHome, command=button_home_click)
Buttonhome.place(x=680, y=145, width=100, height=130)
ButtonStopAllAuto = tk.Button(AutoPage, image=imgStopAll, command=button_stop_all_click)
ButtonStopAllAuto.place(x=680, y=10, width=100, height=130)



ButtonBackHome = tk.Button(AutoPage, image=imgBack, command=ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=120, width=120)

"""**************************THE MANUAL PAGE  **********************************************"""

def ButtonJoystickON_click():
    subprocess.call(["rfkill", "unblock", "bluetooth"])
    print("BlueTooth Start")
    returnval = messagebox.askyesno('Info', "Power ON the joystick now and wait until the led is fix")
    if returnval:
        try:
            # if not pygame.joystick.get_init():
            # print(myps4.get_init())
            myps4.init()
            myps4.listen()
            mymower.useJoystick = True
            # print(myps4.get_init())
        except:
            subprocess.call(["rfkill", "block", "bluetooth"])
            returnval = messagebox.showerror('Error',
                                             "The joystick is not found: Please try to connect under PI GUI First")


def ButtonJoystickOFF_click():
    subprocess.call(["rfkill", "block", "bluetooth"])
    print("BlueTooth Stop")
    mymower.useJoystick = False
    messagebox.showinfo('Info', "Bluetooth and joystick are OFF")


def buttonBlade_stop_click():
    message = "AT+C,0,0,-1,-1,-1,-1,-1,-1"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


def buttonBlade_start_click():
    message = "AT+C,1,0,-1,-1,-1,-1,-1,-1"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


def ButtonForward_click():
    ButtonReverse.configure(state='disabled')
    message = "AT+M," + str(manualSpeedSlider.get()) + ",0,0"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


def ButtonForwardLeft_click():
    ButtonReverse.configure(state='disabled')
    message = "AT+M," + str(manualSpeedSlider.get()) + "," + str(manualSpeedSlider.get()) + ",0"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


def ButtonForwardRight_click():
    ButtonReverse.configure(state='disabled')
    message = "AT+M," + str(manualSpeedSlider.get()) + "," + str(-1 * manualSpeedSlider.get()) + ",0"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


def ButtonRight_click():
    ButtonReverse.configure(state='disabled')
    message = "AT+M," + str(0.3 * manualSpeedSlider.get()) + "," + str(-1 * manualSpeedSlider.get()) + ",0"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


def ButtonLeft_click():
    ButtonReverse.configure(state='disabled')
    message = "AT+M," + str(0.3 * manualSpeedSlider.get()) + "," + str(manualSpeedSlider.get()) + ",0"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


def ButtonReverse_click():
    message = "AT+M,-" + str(manualSpeedSlider.get()) + ",0"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


def ButtonStop_click():
    ButtonReverse.configure(state='active')
    message = "AT+M,0,0"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)

def startManualCamera():
    mymower.cameraManualPageVideoStreamOn = True
    mymower.camera = Picamera2(0)
    frame_rate = 15
    lowresSize = (320, 240)
    config = mymower.camera.create_preview_configuration({'format': 'RGB888', "size": lowresSize})
    mymower.camera.configure(config)
    mymower.camera.set_controls({"FrameRate": frame_rate})
    # time.sleep(2)
    mymower.camera.start(show_preview=False)

def updateManualCamera():
    frame = mymower.camera.capture_array('main')
    img_update = ImageTk.PhotoImage(Image.fromarray(frame))
    ManualLabelStreamVideo.configure(image=img_update)
    ManualLabelStreamVideo.image = img_update
    ManualLabelStreamVideo.update()

def stopManualCamera():
    mymower.cameraManualPageVideoStreamOn = False
    mymower.camera.stop()
    mymower.camera.close()


ManualPage = tk.Frame(fen1)
ManualPage.place(x=0, y=0, height=400, width=800)
Frame1 = tk.Frame(ManualPage)
Frame1.place(x=0, y=0, height=300, width=300)

#Buttontest2 = tk.Button(ManualPage, text="Test2", wraplength=80, command=test2_click)
#Buttontest2.place(x=320, y=00, height=30, width=30)

ManualFrameStreamVideo = tk.Frame(ManualPage, borderwidth="1", relief=tk.SOLID)
ManualFrameStreamVideo.place(x=325, y=30, width=320, height=240)

ManualLabelStreamVideo = tk.Label(ManualFrameStreamVideo)  # ,image=img)
ManualLabelStreamVideo.grid(row=0, column=0, pady=0, padx=0)

ButtonForward = tk.Button(Frame1, image=imgForward, command=ButtonForward_click, repeatdelay=500, repeatinterval=500)
ButtonForward.place(x=100, y=0, height=100, width=100)
ButtonForwardLeft = tk.Button(Frame1, image=imgForwardLeft, command=ButtonForwardLeft_click, repeatdelay=500,repeatinterval=500)
ButtonForwardLeft.place(x=0, y=0, height=100, width=100)
ButtonForwardRight = tk.Button(Frame1, image=imgForwardRight, command=ButtonForwardRight_click, repeatdelay=500,repeatinterval=500)
ButtonForwardRight.place(x=200, y=0, height=100, width=100)
ButtonStop = tk.Button(Frame1, image=imgStop, command=ButtonStop_click)
ButtonStop.place(x=100, y=100, height=100, width=100)
ButtonRight = tk.Button(Frame1, image=imgRight, command=ButtonRight_click, repeatdelay=500, repeatinterval=500)
ButtonRight.place(x=200, y=100, height=100, width=100)
ButtonLeft = tk.Button(Frame1, image=imgLeft, command=ButtonLeft_click, repeatdelay=500, repeatinterval=500)
ButtonLeft.place(x=0, y=100, height=100, width=100)
ButtonReverse = tk.Button(Frame1, image=imgReverse, command=ButtonReverse_click)
ButtonReverse.place(x=100, y=200, height=100, width=100)

ButtonBladeStart = tk.Button(ManualPage, image=imgBladeStart, command=buttonBlade_start_click)
ButtonBladeStart.place(x=680, y=5, width=100, height=50)
ButtonBladeStop = tk.Button(ManualPage, image=imgBladeStop, command=buttonBlade_stop_click)
ButtonBladeStop.place(x=680, y=55, width=100, height=80)

ButtonStopAllManual = tk.Button(ManualPage, image=imgStopAll, command=button_stop_all_click)
ButtonStopAllManual.place(x=680, y=140, width=100, height=130)

tk.Label(ManualPage, image=imgJoystick).place(x=400, y=325)
ButtonJoystickON = tk.Button(ManualPage, image=imgJoystickON, command=ButtonJoystickON_click)
ButtonJoystickON.place(x=500, y=340, width=50, height=50)
ButtonJoystickOFF = tk.Button(ManualPage, image=imgJoystickOFF, command=ButtonJoystickOFF_click)
ButtonJoystickOFF.place(x=350, y=340, width=50, height=50)

RdBtn_keyboard = tk.Checkbutton(ManualPage, text="Use Keyboard to drive the mower", variable=ManualKeyboardUse).place(
    x=0, y=365)

manualSpeedSlider = tk.Scale(ManualPage, orient='horizontal', resolution=0.1, from_=0, to=0.8)
manualSpeedSlider.place(x=0, y=300, width=300, height=60)
manualSpeedSlider.set(0.2)
tk.Label(ManualPage, text='SPEED', fg='green').place(x=0, y=300)

"""
slider1 = tk.Scale(orient='horizontal', from_=0, to=350)
slider1.place(x=50,y=50,anchor='nw',width=300, height=50)
slider2 = tk.Scale(orient='horizontal', from_=0, to=100)
slider2.place(x=50,y=100,anchor='nw',width=300, height=50)
"""
ButtonBackHome = tk.Button(ManualPage, image=imgBack, command=ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=120, width=120)

def ButtonTesting_click():
    fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
               "/maps02" + "/DOCK.npy"
    ##    if (os.path.exists(fileName)):
    ##            crc_mow=0.0
    ##            mowPts=np.load(fileName)
    ##            mowPts_count=int(len(mowPts))
    ##            print(mowPts)
    ##
    ##            for ip in range(int(len(mowPts))):
    ##
    ##                crc_mow=(crc_mow) + (100*float(mowPts[ip][0])) + (100*float(mowPts[ip][1]))
    ##
    ##            print(crc_mow)
    ##            crc_mow=0.0
    ##            for ip in range(int(len(mowPts1))):
    ##
    ##                crc_mow=(crc_mow) + (100*(mowPts1[ip][0])) + (100*(mowPts1[ip][1]))
    ##
    ##            print(crc_mow)

    if (os.path.exists(fileName)):
        crc_mow = 0.0
        mowPts = np.load(fileName)
        mowPts_count = int(len(mowPts))
        print(mowPts)

    fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
               "/maps04" + "/DOCK.npy"
    if (os.path.exists(fileName)):
        crc_mow = 0.0
        mowPts1 = np.load(fileName)
        mowPts1_count = int(len(mowPts1))
        print(mowPts1)

    for ip in range(int(len(mowPts))):
        # print(mowPts[ip][0], " / " , mowPts1[ip][0])

        if ((100 * float(mowPts[ip][0])) != (100 * float(mowPts1[ip][0]))):
            print("erreur x :", ip, (mowPts[ip][0]), (mowPts1[ip][0]))

        if ((100 * float(mowPts[ip][1])) != (100 * float(mowPts1[ip][1]))):
            print("erreur y :", ip, 100 * float(mowPts[ip][1]), 100 * float(mowPts1[ip][1]))
    print("fini")




def ButtonClearConsole_click():
    txtRecu.delete('1.0', tk.END)
    txtSend.delete('1.0', tk.END)
    txtConsoleRecu.delete('1.0', tk.END)


ConsolePage = tk.Frame(fen1)
ConsolePage.place(x=0, y=0, height=400, width=800)

txtRecu = tk.Text(ConsolePage)
ScrollTxtRecu = tk.Scrollbar(txtRecu)
ScrollTxtRecu.pack(side=tk.RIGHT, fill=tk.Y)
txtRecu.pack(side=tk.LEFT, fill=tk.Y)
ScrollTxtRecu.config(command=txtRecu.yview)
txtRecu.config(yscrollcommand=ScrollTxtRecu.set)
txtRecu.place(x=0, y=300, anchor='nw', width=480, height=100)

txtSend = tk.Text(ConsolePage)
ScrollTxtSend = tk.Scrollbar(txtSend)
ScrollTxtSend.pack(side=tk.RIGHT, fill=tk.Y)
txtSend.pack(side=tk.LEFT, fill=tk.Y)
ScrollTxtSend.config(command=txtSend.yview)
txtSend.config(yscrollcommand=ScrollTxtSend.set)
txtSend.place(x=490, y=300, anchor='nw', width=300, height=100)

txtConsoleRecu = tk.Text(ConsolePage)
ScrolltxtConsoleRecu = tk.Scrollbar(txtConsoleRecu)
ScrolltxtConsoleRecu.pack(side=tk.RIGHT, fill=tk.Y)
txtConsoleRecu.pack(side=tk.LEFT, fill=tk.Y)
ScrolltxtConsoleRecu.config(command=txtConsoleRecu.yview)
txtConsoleRecu.config(yscrollcommand=ScrolltxtConsoleRecu.set)
txtConsoleRecu.place(x=0, y=5, anchor='nw', width=800, height=290)


ButtonClearConsole = tk.Button(ConsolePage)
ButtonClearConsole.place(x=660, y=75, height=35, width=120)
ButtonClearConsole.configure(command=ButtonClearConsole_click, text="Clear Console")

ButtonSaveReceived = tk.Button(ConsolePage)
ButtonSaveReceived.place(x=660, y=120, height=35, width=120)
ButtonSaveReceived.configure(command=ButtonSaveReceived_click, text="Save To File")

ButtonBackHome = tk.Button(ConsolePage, image=imgBack, command=ButtonBackToMain_click)
ButtonBackHome.place(x=660, y=160, height=120, width=120)




""" THE NEW MAPS PAGE ***************************************************"""
from backend.data.cfgdata import PathPlannerCfg
from dataclasses import dataclass, field, asdict
from shapely.geometry import LinearRing
from shapely.geometry import MultiLineString
from pathfinder import pathfinder
class Perimeter:
    import pandas as pd
    import networkx as nx
    name: str = ''
    angle_offset: int = 0
    perimeter: pd.DataFrame = field(default_factory=lambda: pd.DataFrame())
    perimeter_polygon: Polygon = Polygon()
    selected_perimeter: Polygon = Polygon()
    perimeter_for_plot: pd.DataFrame = field(default_factory=lambda: pd.DataFrame())
    perimeter_points: MultiPoint = MultiPoint()
    search_wire: LineString = LineString()
    search_wire_points: MultiPoint = MultiPoint()
    gotopoints: pd.DataFrame = field(default_factory=lambda: pd.DataFrame())
    gotopoint: pd.DataFrame = field(default_factory=lambda: pd.DataFrame())
    mowpath: pd.DataFrame = field(default_factory=lambda: pd.DataFrame())
    preview: pd.DataFrame = field(default_factory=lambda: pd.DataFrame())
    obstacles: pd.DataFrame = field(default_factory=lambda: pd.DataFrame())
    #obstacle_img: Image = field(default_factory = lambda: 
     #                           Image.open(os.path.dirname(__file__).replace('/backend/data', '/assets/icons/obstacle.png')))
   
    obstacle_img: pd.DataFrame = field(default_factory=lambda: pd.DataFrame())
    
    astar_graph: nx.Graph = nx.Graph()
    areatomow: float = 0
    distancetogo: float = 0
    map_crc: int = None
    current_perimeter_file: str = ''
    plotgotopoints: bool = False
    # Mow progress
    finished_distance = 0
    distance = 0
    distance_perc = 0
    finished_idx = 0
    idx = 0
    idx_perc = 0 
    # Progress bar
    calculating: bool = False
    calculated_progress: int = 0
    total_progress: int = 0
    task_progress: int = 0
    total_tasks: int = 0

current_map=Perimeter()

def check_direct_way(border: Polygon, start: list, end: list) -> bool:
    way = LineString([start, end])
    direct_way_possible = way.within(border)
    return direct_way_possible

def Cutedge_create_route(perimeter: Polygon, mowborder: str, mowexclusion: bool, 
                 mowborderccw: bool, last_coord: list, border: Polygon,
                 figure: str) -> list:
    route_tmp = []
    edges_pol = []
    if mowborder == 'yes':
        route_tmp = list(perimeter.exterior.coords)
        ###Remove double values###
        route_tmp = list(dict.fromkeys(route_tmp))
        ring = LinearRing(route_tmp)
        ###Check is counter clock wise###
        if not ring.is_ccw and mowborderccw == True:
            route_tmp.reverse()
        ###Look for shortest way from start point###
        first_coords = [min(route_tmp, key=lambda coord: (coord[0]-last_coord[0])**2 + (coord[1]-last_coord[1])**2)]
        first_coords_nr = route_tmp.index(first_coords[0])
        route_tmp = route_tmp[first_coords_nr:]+route_tmp[:first_coords_nr]
        route_tmp.append(route_tmp[0])
        if figure == 'MultiPolygon':
            direct_way_possible = check_direct_way(border, last_coord, route_tmp[0])
            if not direct_way_possible:
                print('Coverage path planner (planing route for cut to edge): No direct way for cut to edge possible, figure saved as to do for path planner')
                edges_pol.append(Polygon(perimeter.exterior.coords))
                route_tmp = []
    if mowexclusion == True:
        for i in range(len(perimeter.interiors)):
            edges_pol.append(Polygon(perimeter.interiors[i].coords))
    return route_tmp, edges_pol

def Lines_create_route(perimeter: Polygon, mowborder: str, mowexclusion: bool, 
                 mowborderccw: bool, last_coord: list, border: Polygon,
                 figure: str) -> list:
    
    route_tmp = []
    edges_pol = []
    if mowborder == 'yes':
        route_tmp = list(perimeter.exterior.coords)
        ###Remove double values###
        route_tmp = list(dict.fromkeys(route_tmp))
        ring = LinearRing(route_tmp)
        ###Check is counter clock wise###
        if not ring.is_ccw and mowborderccw == True:
            route_tmp.reverse()
        ###Look for shortest way from start point###
        first_coords = [min(route_tmp, key=lambda coord: (coord[0]-last_coord[0])**2 + (coord[1]-last_coord[1])**2)]
        first_coords_nr = route_tmp.index(first_coords[0])
        route_tmp = route_tmp[first_coords_nr:]+route_tmp[:first_coords_nr]
        route_tmp.append(route_tmp[0])
        if figure == 'MultiPolygon':
            direct_way_possible = check_direct_way(border, last_coord, route_tmp[0])
            if not direct_way_possible:
                print('Coverage path planner (planing route for cut to edge): No direct way for cut to edge possible, figure saved as to do for path planner')
                edges_pol.append(Polygon(perimeter.exterior.coords))
                route_tmp = []
    if mowexclusion == True:
        for i in range(len(perimeter.interiors)):
            edges_pol.append(Polygon(perimeter.interiors[i].coords))
    return route_tmp, edges_pol

def Ring_create_polygons(current_polygon: Polygon, width: float) -> list:
    polygons = []
    perimeter_coords = list(current_polygon.exterior.coords)
    polygons.append(Polygon((perimeter_coords)))
    #Check if perimeter is last polygon and add a centroid
    last_polygon = current_polygon.buffer(width, resolution=16, join_style=2, mitre_limit=1, single_sided=True)
    last_polygon = last_polygon.simplify(0.02, preserve_topology=False)
    if last_polygon.is_empty:
        centroid = current_polygon.centroid
        polygons.append(centroid)
    for exclusion in current_polygon.interiors:
        exclusion_coords = exclusion.coords
        polygons.append(Polygon((exclusion_coords)))
    return polygons

def Ring_split_multipolygons(current_polygons: MultiPolygon, width: float) -> list:
    polygons = []
    for polygon in current_polygons.geoms:
        polygons.extend(Ring_create_polygons(polygon, width))
    return polygons

import pandas as pd

def Ring_shortest_path_to_point(border: Polygon, points_to_check: list, route: list) -> list:
    end_of_route = route[-1]
    way_nr = None
    route_tmp = None
    current_shortest_way_coord = [end_of_route, list(points_to_check[0].coords)[0]]
    current_shortest_way = LineString(current_shortest_way_coord)
    current_shortest_way_length = current_shortest_way.length
    #Handle shapely problem in case if linestring has length. Within perimeter delivers False
    if current_shortest_way_length == 0:
        current_shortest_way = Point((current_shortest_way_coord[0]))
    for i, point in enumerate(points_to_check):
        shortest_way_coord = [end_of_route, list(point.coords)[0]]
        shortest_way = LineString((shortest_way_coord))
        shortest_way_length = shortest_way.length
        #Handle shapely problem in case if linestring has length. Within perimeter delivers False
        if shortest_way_length == 0:
            shortest_way = Point((shortest_way_coord[0]))
        if (shortest_way_length <= current_shortest_way_length and shortest_way.within(border)) or (shortest_way_length == 0 and shortest_way.touches(border)):
            current_shortest_way_length = shortest_way_length
            route_tmp = list(point.coords)
            way_nr = i
    return route_tmp, way_nr, current_shortest_way_length

def Ring_shortest_path_to_exclusion(border: Polygon, edges_to_check: list, route: list) -> list:
    end_of_route = route[-1]
    way_nr = None
    route_tmp = None
    current_shortest_way_coord = nearest_points(Point((end_of_route)), MultiPoint(edges_to_check[0].exterior.coords))
    current_shortest_way = LineString((current_shortest_way_coord))
    current_shortest_way_length = current_shortest_way.length
    #Handle shapely problem in case if linestring has length. Within perimeter delivers False
    if current_shortest_way_length <= 0.01:
        current_shortest_way_length = 0
        current_shortest_way = Point((current_shortest_way_coord[1]))
    for i, edge in enumerate(edges_to_check):
        shortest_way_coord = nearest_points(Point((end_of_route)), MultiPoint(edge.exterior.coords))
        shortest_way = LineString((shortest_way_coord))
        shortest_way_length = shortest_way.length
        #Handle shapely problem in case if linestring has length. Within perimeter delivers False
        if shortest_way_length <= 0.01:
            shortest_way_length = 0
            shortest_way = Point((shortest_way_coord[1]))
        if (shortest_way_length <= current_shortest_way_length and shortest_way.within(border)) or (shortest_way_length == 0 and shortest_way.touches(border)):
            current_shortest_way_length = shortest_way_length
            route_tmp = list(edge.exterior.coords)
            route_tmp.pop(-1)
            first_coords_nr = route_tmp.index(list(shortest_way_coord[-1].coords)[0])
            route_tmp = route_tmp[first_coords_nr:]+route_tmp[:first_coords_nr]
            route_tmp.append(route_tmp[0])
            #Check for cw or ccw, which way shorter
            route_1 = [route[-1]]
            route_1.extend(route_tmp)
            route_2 = [route[-1]]
            route_tmp.reverse()
            route_2.extend(route_tmp)
            route1_length = LineString((route_1)).length
            route2_length = LineString((route_2)).length
            if route1_length < route2_length:
                route_tmp.reverse()
            way_nr = i
    return route_tmp, way_nr, current_shortest_way_length

def Ring_calcroute(areatomow, border, route, parameters):
    print('Coverage path planner (rings): Start coverage path planner')
    mowccw = parameters.mowborderccw
    print(parameters)
    pathfinder.create()
    pathfinder.angle = 0
    if len(route) == 1:
        print('Coverage path planner (rings): Route is just start point, check if the point within perimeter')
        if not Point((route[-1])).within(border) or not Point((route[-1])).touches(border):
            print('Coverage path planner (rings): Rover is outside perimeter, interpolate to the border')
            route_tmp = border.exterior.coords
            route = [min(route_tmp, key=lambda coord: (coord[0]-route[-1][0])**2 + (coord[1]-route[-1][1])**2)]
            print('Coverage path planner (rings): New start point: '+str(route))
    
    print('Coverage path planner (rings): Create polygons')
    areatomow_tmp = areatomow
    polygons = []
    while True:
        if areatomow_tmp.is_empty:
            print('Coverage path planner (rings): Calculation done')
            break
        if areatomow_tmp.geom_type == 'Polygon':
            print("area to mow  is single polygon")
            polygons.extend(Ring_create_polygons(areatomow_tmp, -parameters.width))
        elif areatomow_tmp.geom_type == 'MultiPolygon':
            print("area to mow  is multipolygon")
            polygons.extend(Ring_split_multipolygons(areatomow_tmp, -parameters.width))
        else:
            print('Unknown figure')
            break
        areatomow_tmp = areatomow_tmp.buffer(-parameters.width, resolution=16, join_style=2, mitre_limit=1, single_sided=True)
        areatomow_tmp = areatomow_tmp.simplify(0.02, preserve_topology=False)
    print('Coverage path planner (rings): Polygons created. Polygons to calculate: '+str(len(polygons)))

    print('Coverage path planner (calc rings): Create ways')
    ways_to_go = pd.DataFrame()
    for i, polygon in enumerate(polygons):
        if polygon.geom_type == 'Polygon':
            pol_df = pd.DataFrame({'name': 'polygon'+str(i), 'shapely': polygon, 'coords': [list(polygon.exterior.coords)], 'type': 'polygon', 'gone': False, 'take into account': True})
        else:
            pol_df = pd.DataFrame({'name': 'polygon'+str(i), 'shapely': polygon, 'coords': [list(polygon.coords)], 'type': 'point', 'gone': False, 'take into account': True})
        ways_to_go = pd.concat([ways_to_go, pol_df], ignore_index=True)
    
    print('Coverage path planner (calc rings): Starting loop')
    while True:
        gone_way_pol = None
        gone_way_pt = None
        ways = ways_to_go[ways_to_go['gone'] == False]
        if ways.empty:
            print('Coverage path planner (calc rings): Calculation done')
            break
        ways_polygons = ways_to_go[(ways_to_go['type'] == 'polygon') & (ways_to_go['gone'] == False) & (ways_to_go['take into account'] == True)]
        ways_points = ways_to_go[(ways_to_go['type'] == 'point') & (ways_to_go['gone'] == False) & (ways_to_go['take into account'] == True)]
        print('Check for polygons to cut in range')
        if not ways_polygons.empty:
            possible_pol = ways_polygons
            possible_pol = possible_pol.reset_index(drop=True)
            route_pol_way, gone_way_pol, length_to_pol = Ring_shortest_path_to_exclusion(border, possible_pol['shapely'].to_list(), route)  
        else:
            print('No direct way to a polygon found')
        if not ways_points.empty:
            possible_pt = ways_points
            possible_pt = possible_pt.reset_index(drop=True)
            route_pt_way, gone_way_pt, length_to_pt = Ring_shortest_path_to_point(border, possible_pt['shapely'].to_list(), route)
        #Decide for a shortest way
        if gone_way_pol != None and gone_way_pt != None:
            if length_to_pol < length_to_pt:
                index = ways_to_go[ways_to_go['coords'].apply(lambda x: x==list(possible_pol.loc[gone_way_pol, 'shapely'].exterior.coords))].index.array[0] 
                ways_to_go.at[index, 'gone'] = True
                print('Found way to a polygon: Finished: '+str(len(ways_to_go[ways_to_go['gone']==True]))+'/'+str(len(ways_to_go)))  
                route.extend(route_pol_way)  
                # Progress-bar data
                current_map.total_progress = len(ways_to_go)
                current_map.calculated_progress = len(ways_to_go[ways_to_go['gone']==True])
            else:
                index = ways_to_go[ways_to_go['coords'].apply(lambda x: x==list(possible_pt.loc[gone_way_pt, 'shapely'].coords))].index.array[0]
                ways_to_go.at[index, 'gone'] = True
                print('Found way to a point: Finished: '+str(len(ways_to_go[ways_to_go['gone']==True]))+'/'+str(len(ways_to_go)))  
                route.extend(route_pt_way)
                # Progress-bar data
                current_map.total_progress = len(ways_to_go)
                current_map.calculated_progress = len(ways_to_go[ways_to_go['gone']==True])
        elif gone_way_pol != None:
            index = ways_to_go[ways_to_go['coords'].apply(lambda x: x==list(possible_pol.loc[gone_way_pol, 'shapely'].exterior.coords))].index.array[0] 
            ways_to_go.at[index, 'gone'] = True
            print('Found way to a polygon: Finished: '+str(len(ways_to_go[ways_to_go['gone']==True]))+'/'+str(len(ways_to_go)))  
            route.extend(route_pol_way)
            # Progress-bar data
            current_map.total_progress = len(ways_to_go)
            current_map.calculated_progress = len(ways_to_go[ways_to_go['gone']==True])
        elif gone_way_pt != None:
            index = ways_to_go[ways_to_go['coords'].apply(lambda x: x==list(possible_pt.loc[gone_way_pt, 'shapely'].coords))].index.array[0]
            ways_to_go.at[index, 'gone'] = True
            print('Found way to a point: Finished: '+str(len(ways_to_go[ways_to_go['gone']==True]))+'/'+str(len(ways_to_go)))  
            route.extend(route_pt_way)
            # Progress-bar data
            current_map.total_progress = len(ways_to_go)
            current_map.calculated_progress = len(ways_to_go[ways_to_go['gone']==True])
        else:
            print('No point for start over direct way found. Starting A* pathfinder') 
            possible_goals = ways_to_go[ways_to_go['gone'] == False]
            for i in range(len(possible_goals)):
                if not possible_goals.empty:
                    if possible_goals.iloc[i]['type'] == 'polygon':
                        goal = nearest_points(Point(route[-1]), MultiPoint((list(possible_goals.iloc[i]['shapely'].exterior.coords))))[1]
                    else:
                        goal = nearest_points(Point(route[-1]), Point((list(possible_goals.iloc[i]['shapely'].coords))))[1]
                    goal = list(goal.coords)
                route_astar = pathfinder.find_way(route[-1], goal)
                if route_astar != []:
                    index = possible_goals.index.values[0]
                    route.extend(route_astar)
                    break
            if route_astar == []:
                print('Coverage patha planner (rings): Could not finish calculation')
                break
           
    return route


def Cutedge_calcroute(area_to_mow, parameters, start):
    print('Backend: Calc route for cutedge')
    mowoffs = -parameters.width
    if parameters.mowborder != 0:
        mowborder = 'yes'
    else:
        mowborder = 'no'
    mowexclusion = parameters.mowexclusion
    mowborderccw = parameters.mowborderccw
    rounds = parameters.mowborder
    num_edge_per = min(parameters.distancetoborder, 2)
    start_coords = start
    route = []
    route_tmp = []
    edges_pol = []

    area_to_mow_tmp = area_to_mow
    last_coord = start[0]
    for i in range(rounds):
        if area_to_mow_tmp.is_empty:
            print('Coverage path planner (planing route for cut to edge): Could not finished distancetoborderloop, please check your settings. Max value: '+str(i))
            break
        elif area_to_mow_tmp.geom_type == 'MultiPolygon':
            print('Coverage path planner (planing route for cut to edge): MultiPolygon detected, creating loop')
            for single_polygon in area_to_mow_tmp.geoms:
                route_tmp, edges_pol_tmp = Cutedge_create_route(single_polygon, mowborder, mowexclusion, mowborderccw, last_coord, area_to_mow, 'MultiPolygon')
                print('Coverage path planner (planing route for cut to edge): Loop call delivered route: '+str(route_tmp))
                print('Coverage path planner (planing route for cut to edge): Loop call delivered figures to mow: '+str(len(edges_pol_tmp)))
                if route_tmp != []:
                    route.extend(route_tmp)
                    last_coord = route[-1]
                edges_pol.extend(edges_pol_tmp)
        elif area_to_mow_tmp.geom_type == 'Polygon':
            print('Coverage path planner (planing route for cut to edge): Polygon detected')
            route_tmp, edges_pol_tmp = Cutedge_create_route(area_to_mow_tmp, mowborder, mowexclusion, mowborderccw, last_coord, area_to_mow, 'Polygon')
            print('Coverage path planner (planing route for cut to edge): Call delivered route: '+str(route_tmp))
            print('Coverage path planner (planing route for cut to edge): Call delivered figures to mow: '+str(len(edges_pol_tmp)))
            if route_tmp != []:
                route.extend(route_tmp)
                last_coord = route[-1]
            edges_pol.extend(edges_pol_tmp)
        else:
            print('Coverage path planner (planing route for cut to edge): Unknown figure, calculation incomplete. Shapely: '+area_to_mow_tmp.geom_type)
            break
    
        area_to_mow_tmp = area_to_mow_tmp.buffer(mowoffs, resolution=16, join_style=2, mitre_limit=1, single_sided=True)
        area_to_mow_tmp = area_to_mow_tmp.simplify(0.05, preserve_topology=False)

    if route == []:
        route.extend(start)
    
    print('Coverage path planner (planing route for cut to edge): Calculation finished')
    print('Coverage path planner (planing route for cut to edge): Route '+str(len(route))+' points; Exclusions to calculate: '+str(len(edges_pol)))
    return route, edges_pol
def Lines_check_prio_lines(ways_to_go: pd.DataFrame, border: Polygon, current_level: int, route: list, angle: int) -> list:
    possible_start = []
    gone_way = None
    #Standard call, look for lines: same level, level under, level over
    if current_level != None:
        #Check for prio lines 
        ways_area = ways_to_go[(ways_to_go['type'] == 'area') & (ways_to_go['gone'] == False) & (ways_to_go['take into account'] == True) & (ways_to_go['level_min'] <= current_level+1) & (ways_to_go['level_min']>=current_level-1)]
        if not ways_area.empty:
            #possible_start = list(ways_area['coords'])
            possible_start1 = min(ways_area['coords'], key=lambda coord: (coord[0][0]-route[-1][0])**2 + (coord[0][1]-route[-1][1])**2)
            possible_start2 = min(ways_area['coords'], key=lambda coord: (coord[1][0]-route[-1][0])**2 + (coord[1][1]-route[-1][1])**2)
            possible_start = [possible_start1, possible_start2]
            route_line_way, gone_way, length_to_line = Lines_shortest_path(border, possible_start, route, angle)
            return route_line_way, gone_way, length_to_line, possible_start
        else:
            #Check for all lines and pick the nearest two 
            ways_area = ways_to_go[(ways_to_go['type'] == 'area') & (ways_to_go['gone'] == False) & (ways_to_go['take into account'] == True)]
            if not ways_area.empty:
                possible_start1 = min(ways_area['coords'], key=lambda coord: (coord[0][0]-route[-1][0])**2 + (coord[0][1]-route[-1][1])**2)
                possible_start2 = min(ways_area['coords'], key=lambda coord: (coord[1][0]-route[-1][0])**2 + (coord[1][1]-route[-1][1])**2)
                possible_start = [possible_start1, possible_start2]
                route_line_way, gone_way, length_to_line = Lines_shortest_path(border, possible_start, route, angle)
                return route_line_way, gone_way, length_to_line, possible_start
            else:
                return None, None, None, None
    #If curren_level == None, then it is first call or no prio lines accessable
    if current_level == None or gone_way == None:
        ways_area = ways_to_go[(ways_to_go['type'] == 'area') & (ways_to_go['gone'] == False) & (ways_to_go['take into account'] == True)]
        if not ways_area.empty:
            possible_start1 = min(ways_area['coords'], key=lambda coord: (coord[0][0]-route[-1][0])**2 + (coord[0][1]-route[-1][1])**2)
            possible_start2 = min(ways_area['coords'], key=lambda coord: (coord[1][0]-route[-1][0])**2 + (coord[1][1]-route[-1][1])**2)
            possible_start = [possible_start1, possible_start2]
            route_line_way, gone_way, length_to_line = Lines_shortest_path(border, possible_start, route, angle)
            return route_line_way, gone_way, length_to_line, possible_start
        else:
            return None, None, None, None
    
def Lines_shortest_path(border: Polygon, ways_to_check: list, route: list, angle: int) -> list:
    possible_line = ways_to_check[0]
    end_of_route = route[-1]
    current_shortest_way = LineString((end_of_route, possible_line[0])).length
    way_nr = None
    route_tmp = None
    for i, way in enumerate(ways_to_check):
        if way_nr != None and current_shortest_way < 2*0.18:
            break
        line_coord = way
        way_rev = way.copy()
        way_rev.reverse()
        line_coord_rev = way_rev
        way_to_line = LineString((end_of_route, line_coord[0]))
        length_of_way = way_to_line.length
        #Handle shapely problem in case if linestring has length. Within perimeter delivers False
        if length_of_way == 0:
            way_to_line = Point((end_of_route))
            length_of_way = 0
        way_to_line_rev = LineString((end_of_route, line_coord_rev[0]))
        length_of_rev_way = way_to_line_rev.length
        if length_of_rev_way == 0:
            way_to_line_rev = Point((end_of_route))
            length_of_rev_way = 0
        #Now check the shortest path, direct or reverse line
        if (length_of_way <= current_shortest_way and way_to_line.within(border)) or (length_of_way == 0 and way_to_line.touches(border)):
            current_shortest_way = length_of_way
            possible_line = line_coord
            way_nr = i
        if (length_of_rev_way < current_shortest_way and way_to_line_rev.within(border)) or (length_of_rev_way == 0 and way_to_line_rev.touches(border)):
            current_shortest_way = length_of_rev_way
            possible_line = line_coord_rev
            way_nr = i
        #No direct way for standar or reverse line, check A* distance
        if (not way_to_line.within(border) or not way_to_line_rev.within(border)):
            way_coord, length_of_astar_way, reverse_line = Lines_check_astar_distance(border, way, angle, route, current_map.perimeter_points)
            if (length_of_astar_way != None and length_of_astar_way < current_shortest_way) or (length_of_astar_way != None and way_nr == None):
                current_shortest_way = length_of_astar_way
                if reverse_line:
                    way_coord.extend(line_coord_rev)
                    possible_line = way_coord
                else:
                    way_coord.extend(line_coord)
                    possible_line = way_coord
                way_nr = i
    if way_nr != None:
        route_tmp = possible_line
    return route_tmp, way_nr, current_shortest_way
from shapely import affinity
def Lines_check_astar_distance(border: Polygon, possible_start: list, angle: int, route: list, perimeter_points) -> list:
    route_tmp = []
    reverse_line = None
    #Create start point  
    astar_start_tmp = Point((route[-1]))
    astar_start_tmp = affinity.rotate(astar_start_tmp, angle, origin=(0, 0))
    astar_start_tmp = nearest_points(astar_start_tmp, perimeter_points)
    astar_start = list(astar_start_tmp[1].coords)
    #Create end points
    #Create first end point
    coords_tmp1 = possible_start[0]
    coords_tmp1 = Point((coords_tmp1))
    coords_tmp1 = affinity.rotate(coords_tmp1, angle, origin=(0, 0))
    astar_end_tmp1 = nearest_points(perimeter_points, coords_tmp1)
    astar_end1 = list(astar_end_tmp1[0].coords)
    #Create second end point
    coords_tmp2 = possible_start[1]
    coords_tmp2 = Point((coords_tmp2))
    coords_tmp2 = affinity.rotate(coords_tmp2, angle, origin=(0, 0))
    astar_end_tmp2 = nearest_points(perimeter_points, coords_tmp2)
    astar_end2 = list(astar_end_tmp2[0].coords)
    try:
        astar_path1 = nx.astar_path(current_map.astar_graph, astar_start[0], astar_end1[0], heuristic=None, weight='weight')
        astar_path1 = turn_coords(astar_path1, -angle)
        way1 = [route[-1]]
        way1.extend(astar_path1)
        way1.extend([possible_start[0]])
        way1 = LineString((way1))
        way1_length = way1.length
        if not way1.within(border):
            way1 = None

        astar_path2 = nx.astar_path(current_map.astar_graph, astar_start[0], astar_end2[0], heuristic=None, weight='weight')
        astar_path2 = turn_coords(astar_path2, -angle)
        way2 = [route[-1]]
        way2.extend(astar_path2)
        way2.extend([possible_start[1]])
        way2 = LineString((way2))
        way2_length = way2.length
        if not way2.within(border):
            way2 = None
    except Exception as e:
        logger.warning('A* pathfinder delivered unexpexted result')
        logger.debug(str(e))
        return None, None, None
    if (way1 != None and way2 != None and way1_length <= way2_length) or (way1 != None and way2 == None):
        current_shortest_way_length = way1_length
        route_tmp = astar_path1
        reverse_line = False
    elif (way1 != None and way2 != None and way1_length > way2_length) or (way2 != None and way1 == None):
        current_shortest_way_length = way2_length
        route_tmp = astar_path2
        reverse_line = True
    else:
        return None, None, None
    return route_tmp, current_shortest_way_length, reverse_line
def Lines_calcroute(areatomow, border, line_mask, edges_pol, route, parameters, angle):
    print('Coverage path planner (lines): Start coverage path planner')
    print(parameters)

    pathfinder.create()
    pathfinder.angle = angle

    if len(route) == 1:
        print('Coverage path planner (lines): Route is just start point, check if the point within perimeter')
        if not Point((route[-1])).within(border) and not Point((route[-1])).touches(border):
            print('Coverage path planner (lines): Rover is outside perimeter, interpolate to the border')
            route_tmp = border.exterior.coords
            route = [min(route_tmp, key=lambda coord: (coord[0]-route[-1][0])**2 + (coord[1]-route[-1][1])**2)]
            print('Coverage path planner (lines): New start point: '+str(route))

    ways_to_go = pd.DataFrame()
    perimeter_points = current_map.perimeter_points

    tosimplify = False
    if parameters.distancetoborder == 0:
        print('Distance to border selected to 0, increase boundary')
        border = border.buffer(0.05, resolution=16, join_style=2, mitre_limit=1, single_sided=True)
    else:
        print('Distance to border is not equal to 0, use original boundary')
    #Extract y-coordinate and sort data
    line_mask_coords = []
    #Check for data type MultiLineString or GeometryCollection as expected?
    print('Extract lines coordinates and sort data')
    try: 
        for line in line_mask.geoms:
            #Check if line is a point and create a virtual line
            if len(list(line.coords)) == 1:
                print('At least one line is a point, create a virtual line, and set simplify flag')
                tosimplify = True
                coords = list(line.coords)
                coords.extend(coords)
                line_mask_coords.append(coords)
            #Check if line is a line (as expected)
            else:
                line_mask_coords.append(list(line.coords))
    #There is no MultiLineString and no GeometryCollection. Selection to small?
    except:
        print('Line mask is no MultiLineString. Selection to small?')
        if not line_mask.is_empty:
            line = line_mask
            #Check if line is a point and create a virtual line
            if len(list(line.coords)) == 1:
                print('At least one line is a point, create a virtual line, and set simplify flag')
                tosimplify = True
                coords = list(line.coords)
                coords.extend(coords)
                line_mask_coords.append(coords)
             #Check if line is a line (as expected)
            else:
                line_mask_coords.append(list(line.coords))
        #Exctraction not possible
        else:
            print('Extraction not possible. Line mask is empty')
            line_mask_coords = []
    #Sort coordinates and create new MultiLineString
    line_mask_coords.sort(key=lambda x: x[0][1])
    result_lines = MultiLineString(line_mask_coords)
    print('Lines to calculate: '+str(len(result_lines.geoms)))

    #Add additional informations to edges to cut about y-position on the map
    print('Sort border cuts and exclusion cuts')
    print('Figures to sort: '+str(len(edges_pol)))
    border_bounds = border.bounds
    print('Border bounds: '+str(border_bounds))
    for i, pol in enumerate(edges_pol):
        bounds = pol.bounds
        print('Excl/borger to cut bounds: '+str(bounds))
        level_min = round((bounds[1] - border_bounds[1])/parameters.width)
        level_max = round((bounds[3]-border_bounds[1])/parameters.width)
        print('Add levels to excl/border to cut. Min: '+str(level_min)+' Max: '+str(level_max))
        edge = pd.DataFrame({'name': 'edge'+str(i),'shapely': pol, 'level_min': level_min, 'level_max': level_max, 'coords': [list(pol.exterior.coords)], 'type': 'edge', 'gone': False, 'take into account': True})
        ways_to_go = pd.concat([ways_to_go, edge], ignore_index=True)
    
    #Add additionl informations to the lines about y-position on the map
    print('Sort lines to cut')
    lines_to_go = []
    line_level = 0
    current_level = None
    if not result_lines.is_empty:
        coord = list(result_lines.geoms[0].coords)
        coord_y_old = coord[0][1]
        for i in range(len(result_lines.geoms)):
            coord = list(result_lines.geoms[i].coords)
            if coord[0][1] > coord_y_old:
                line_level += 1
            line = pd.DataFrame({'name': 'area'+str(i),'shapely': result_lines.geoms[i], 'level_min': line_level, 'level_max': line_level, 'coords': [coord], 'type': 'area', 'gone': False, 'take into account': True})
            ways_to_go = pd.concat([ways_to_go, line], ignore_index=True)
            coord_y_old = coord[0][1]
    else:
        print('No lines to sort found')
        line = pd.DataFrame({'name': 'area1','shapely': LineString(), 'level_min': None, 'level_max': None, 'coords': [None], 'type': 'area', 'gone': True, 'take into account': True})
        ways_to_go = pd.concat([ways_to_go, line], ignore_index= True)

    #Starting coverage path planner
    print('Coverage path planner (calc lines): Starting loop')
    astar_path = []
    astar_last_way = []
    while True:
        gone_way = None
        gone_way_edge = None
        ways_to_go_filtered = ways_to_go[ways_to_go['gone'] == False]
        if ways_to_go_filtered.empty:
            print('Coverage path planner (calc lines): No more way to calculate, ending loop')
            break

        #Check for ways to lines
        print('Check for prio lines')
        route_line_way, gone_way, length_to_line, possible_start = Lines_check_prio_lines(ways_to_go, border, current_level, route, angle)
        if gone_way != None:
            print('Found way to line, distance: '+str(length_to_line))

        #Check for possible ways to edges
        print('Check for edges to cut in range')
        #First call or after pathfinder, current_level = None
        if current_level == None:
            ways_edge = ways_to_go[(ways_to_go['type'] == 'edge') & (ways_to_go['gone'] == False) & (ways_to_go['take into account'] == True)]
        else:
            ways_edge = ways_to_go[(ways_to_go['type'] == 'edge') & (ways_to_go['gone'] == False) & (ways_to_go['take into account'] == True) & (ways_to_go['level_min'] <= current_level) & (ways_to_go['level_max'] >= current_level)]
        if not ways_edge.empty:
            possible_edges = ways_edge.reset_index(drop=True)
            route_edge_way, gone_way_edge, length_to_edge = shortest_path_to_exclusion(border, possible_edges['shapely'].to_list(), route)
            if gone_way_edge != None:
                print('Found edge(s) to cut in range, distance: '+str(length_to_edge))
            else:
                print('No direct way to a edge found')
        else:
            print('No edges in range found')
        
        #Decide for a shortest way
        if gone_way != None and gone_way_edge != None:
            if length_to_line < 0.5*length_to_edge:
                index = ways_to_go[ways_to_go['coords'].apply(lambda x: x==possible_start[gone_way])].index.array[0]
                ways_to_go.at[index, 'gone'] = True
                current_level = ways_to_go.at[index, 'level_min']
                print('Take way to a line, current level: '+str(ways_to_go.at[index, 'level_min'])+' Finished: '+str(len(ways_to_go[ways_to_go['gone']==True]))+'/'+str(len(ways_to_go)))  
                astar_last_way = []
                route.extend(route_line_way)   
                # Progress-bar data
                current_map.total_progress = len(ways_to_go)
                current_map.calculated_progress = len(ways_to_go[ways_to_go['gone']==True])
            else:
                index = ways_to_go[ways_to_go['coords'].apply(lambda x: x==list(possible_edges.loc[gone_way_edge, 'shapely'].exterior.coords))].index.array[0] 
                ways_to_go.at[index, 'gone'] = True
                print('Take way to a edge, current level: '+str(ways_to_go.at[index, 'level_min'])+' Finished: '+str(len(ways_to_go[ways_to_go['gone']==True]))+'/'+str(len(ways_to_go)))  
                astar_last_way = []
                route.extend(route_edge_way)
                # Progress-bar data
                current_map.total_progress = len(ways_to_go)
                current_map.calculated_progress = len(ways_to_go[ways_to_go['gone']==True])
        elif gone_way != None:
            index = ways_to_go[ways_to_go['coords'].apply(lambda x: x==possible_start[gone_way])].index.array[0]
            ways_to_go.at[index, 'gone'] = True
            current_level = ways_to_go.at[index, 'level_min']
            print('Take way to a line, current level: '+str(ways_to_go.at[index, 'level_min'])+' Finished: '+str(len(ways_to_go[ways_to_go['gone']==True]))+'/'+str(len(ways_to_go)))  
            astar_last_way = []
            route.extend(route_line_way) 
            # Progress-bar data
            current_map.total_progress = len(ways_to_go)
            current_map.calculated_progress = len(ways_to_go[ways_to_go['gone']==True])
        elif gone_way_edge != None:
            index = ways_to_go[ways_to_go['coords'].apply(lambda x: x==list(possible_edges.loc[gone_way_edge, 'shapely'].exterior.coords))].index.array[0] 
            ways_to_go.at[index, 'gone'] = True
            print('Take way to a edge, current level: '+str(ways_to_go.at[index, 'level_min'])+' Finished: '+str(len(ways_to_go[ways_to_go['gone']==True]))+'/'+str(len(ways_to_go)))  
            astar_last_way = []
            route.extend(route_edge_way)
            # Progress-bar data
            current_map.total_progress = len(ways_to_go)
            current_map.calculated_progress = len(ways_to_go[ways_to_go['gone']==True])
        else:
            print('No point for start over direct way found. Starting A* pathfinder')
            possible_goals = ways_to_go[ways_to_go['gone'] == False]
            for i in range(len(possible_goals)):
                coords = possible_goals.iloc[i]['coords']
                goal = nearest_points(Point(route[-1]), MultiPoint(coords))
                goal = list(goal[1].coords)
                route_astar = pathfinder.find_way(route[-1], goal)
                if route_astar != []:
                    index = possible_goals.index.values[i]
                    route.extend(route_astar)
                    current_level = None
                    break
            if route_astar == []:
                print('Coverage patha planner (lines): Could not finish calculation')
                break
        
    route_shapely = LineString(route)
    return route_shapely

def calc(selected_perimeter: Polygon, parameters: PathPlannerCfg, start_pos: list) -> list:
    import math
    import random
    

    if selected_perimeter.is_empty or (not parameters.mowarea and parameters.mowborder==0 and not parameters.mowexclusion):
        print(f"Coverage path planner parameters are not valid. Calculation aborted.")
        print(parameters)
        return []
    print('Backend: Planning route:')
    print(parameters)
    print('Rover start position: '+str(start_pos))
    start_pos = Point(start_pos)
    #check if random angle
    if parameters.angle == None or math.isnan(float(parameters.angle)):
        angle = random.randrange(start=359) 
        print(f'Coverage path planner uses random angle: {angle}Deg')
    else:
        angle = parameters.angle

    if parameters.pattern == 'lines' or parameters.pattern == 'squares':
        start_pos = map.turn(start_pos, angle)
        selected_area_turned = map.turn(selected_perimeter, angle)
        #border = map.turn(current_map.perimeter_polygon, angle)
        border = map.turn(selected_perimeter, angle)

        area_to_mow = map.areatomow(selected_area_turned, parameters.distancetoborder, parameters.width)
       
        route, edge_polygons = Cutedge_calcroute(selected_area_turned, parameters, list(start_pos.coords))
        if parameters.mowarea:
            line_mask = map.linemask(area_to_mow, parameters.width)
            x, y = line_mask.exterior.xy
            ax[4].plot(x,y,color='blue', linewidth=0.4,picker=True,marker='.')

        else:
            line_mask = MultiLineString()
        route = Lines_calcroute(area_to_mow, border, line_mask, edge_polygons, route, parameters, angle)
        route = map.turn(route, -angle)
        route = list(route.coords)
        # Clear progress bar
        #if parameters.pattern == 'lines' or (parameters.pattern == 'squares' and parameters.mowarea != True):
            
            #current_map.total_progress = current_map.calculated_progress = 0

    if parameters.pattern == 'squares' and parameters.mowarea == True:
        last_coord = route[-1]
        last_coord = Point(last_coord)
        last_coord = map.turn(last_coord, angle+90)
        selected_area_turned = map.turn(selected_perimeter, angle+90)
        #border = map.turn(current_map.perimeter_polygon, angle)
        border = map.turn(selected_perimeter, angle)
        area_to_mow = map.areatomow(selected_area_turned, parameters.distancetoborder, parameters.width)
        if parameters.mowarea:
            line_mask = map.linemask(area_to_mow, parameters.width)
        else:
            line_mask = MultiLineString()
        route2 = Lines_calcroute(area_to_mow, border, line_mask, [], list(last_coord.coords), parameters, angle+90)
        route2 = map.turn(route2, -angle-90)
        route.extend(list(route2.coords))
        # Clear progress bar
        #current_map.total_progress = current_map.calculated_progress = 0
    
    if parameters.pattern == 'rings':
        border = selected_perimeter
        area_to_mow = map.areatomow(selected_perimeter, parameters.distancetoborder, parameters.width)
        route, edge_polygons = Cutedge_calcroute(selected_perimeter, parameters, list(start_pos.coords))
        route = Ring_calcroute(area_to_mow, border, route, parameters)
        # Clear progress bar
        #current_map.total_progress = current_map.calculated_progress = 0
    #print(str(route))
    return route

NewMapsPage = tk.Frame(fen1)
NewMapsPage.place(x=0, y=0, height=400, width=800)
FrameNMP1 = tk.Frame(NewMapsPage)
FrameNMP1.place(x=0, y=0, height=300, width=300)
ButtonForwardNMP = tk.Button(FrameNMP1, image=imgForward, command=ButtonForward_click, repeatdelay=500, repeatinterval=500)
ButtonForwardNMP.place(x=100, y=0, height=100, width=100)
ButtonForwardLeftNMP = tk.Button(FrameNMP1, image=imgForwardLeft, command=ButtonForwardLeft_click, repeatdelay=500,repeatinterval=500)
ButtonForwardLeftNMP.place(x=0, y=0, height=100, width=100)
ButtonForwardRightNMP = tk.Button(FrameNMP1, image=imgForwardRight, command=ButtonForwardRight_click, repeatdelay=500,repeatinterval=500)
ButtonForwardRightNMP.place(x=200, y=0, height=100, width=100)
ButtonStopNMP = tk.Button(FrameNMP1, image=imgStop, command=ButtonStop_click)
ButtonStopNMP.place(x=100, y=100, height=100, width=100)
ButtonRightNMP = tk.Button(FrameNMP1, image=imgRight, command=ButtonRight_click, repeatdelay=500, repeatinterval=500)
ButtonRightNMP.place(x=200, y=100, height=100, width=100)
ButtonLeftNMP = tk.Button(FrameNMP1, image=imgLeft, command=ButtonLeft_click, repeatdelay=500, repeatinterval=500)
ButtonLeftNMP.place(x=0, y=100, height=100, width=100)
ButtonReverseNMP = tk.Button(FrameNMP1, image=imgReverse, command=ButtonReverse_click)
ButtonReverseNMP.place(x=100, y=200, height=100, width=100)


# creation of the canvas for map view
FrameLiveMapNMP = tk.Frame(NewMapsPage, borderwidth="1", relief=tk.SOLID)
FrameLiveMapNMP.place(x=305, y=5, width=360,height=330)
figLiveMapNMP, axLiveMapNMP = plt.subplots()
canvasLiveMapNMP = FigureCanvasTkAgg(figLiveMapNMP, master=FrameLiveMapNMP)
canvasLiveMapNMP.get_tk_widget().place(x=0, y=0, width=360, height=330)

axLiveMapNMP.plot(mymower.ActiveMapX, mymower.ActiveMapY, color='r', linewidth=0.4, marker='.', markersize=2)
#axLiveMapNMP.set_xlim(-50, 50)
#axLiveMapNMP.set_ylim(-50, 50)
canvasLiveMapNMP.draw()

def BtnRecordMapStop_click():
    mymower.record_perimeter=False
    
def BtnRecordMapStart_click():
    mymower.record_perimeter=True

def updateNewMapsPage():     
    mowerpos1 = [mymower.laststatex, mymower.statex]
    mowerpos2 = [mymower.laststatey, mymower.statey]

    axLiveMapNMP.plot(mowerpos1, mowerpos2, color='g', linewidth=2)
    axLiveMapNMP.scatter(mymower.statex,mymower.statey, color='g', s=10)
    #axLiveMap.autoscale(False)
    canvasLiveMapNMP.draw()


def BtnSaveMap_click():
    mymower.newPerimeter=np.array(list(zip(mymower.newPerimeterx,mymower.newPerimetery)))
    fileName = cwd + "/House" + "{0:0>2}".format(VarHouseNrNMP.get()) + \
                           "/maps" + "{0:0>2}".format(VarMapNrNMP.get()) + "/perimeter_raw.npy"
    if os.path.exists(fileName):
        messagebox.showwarning('warning', "file already exist try at other location")
    else:
        output_file = Path(fileName)
        output_file.parent.mkdir(exist_ok=True, parents=True)
        np.save(fileName, mymower.newPerimeter, allow_pickle=True, fix_imports=True)
        print(mymower.newPerimeter)
        fileNamePeri = cwd + "/House" + "{0:0>2}".format(VarHouseNrNMP.get()) + \
                           "/maps" + "{0:0>2}".format(VarMapNrNMP.get()) + "/perimeter.npy"
        polygon1 = Polygon(np.squeeze(mymower.newPerimeter))
        from shapely.validation import explain_validity
        print(explain_validity(polygon1))
        print("valid  polygon init " ,polygon1.is_valid)
        new_polygon = polygon1.buffer(0)
        #border = border.buffer(0.05, resolution=16, join_style=2, mitre_limit=1, single_sided=True)
        print("valid  polygon2 " ,new_polygon.is_valid)
        polygon2=new_polygon.geoms[0].simplify(tolerance=0.03, preserve_topology=True) #tolerance is 2 cm here
        a, b = polygon2.exterior.xy
        print(len(a),len(mymower.newPerimeter))
        axLiveMapNMP.plot(a,b,color='g', linewidth=0.4,picker=True,marker='.')
        
        


BtnRecordMapStop = tk.Button(NewMapsPage, command=BtnRecordMapStop_click, text="Stop Record")
BtnRecordMapStop.place(x=0, y=370, height=25, width=80)
BtnRecordMapStart = tk.Button(NewMapsPage, command=BtnRecordMapStart_click, text="Start Record")
BtnRecordMapStart.place(x=90, y=370, height=25, width=80)

BtnSaveMap = tk.Button(NewMapsPage, command=BtnSaveMap_click, text="Save Map")
BtnSaveMap.place(x=690, y=220, height=25, width=80)

#style = ttk.Style()
#style.theme_use('default')
#style.configure('My.TSpinbox', font=('Arial', 30),arrowsize=35)
from tkinter.font import Font

tk.Label(NewMapsPage, text="House",font=Font(family='Arial', size=16)).place(x=690, y=0)
VarHouseNrNMP = tk.IntVar()
VarHouseNrNMP.set(9)  # set the initial value
HouseNrNMPspinbox = tk.Spinbox(NewMapsPage, from_=0, to=9, textvariable=VarHouseNrNMP,font=Font(family='Arial', size=36))
HouseNrNMPspinbox.place(x=690, y=30, height=70, width=85)

tk.Label(NewMapsPage, text="Map",font=Font(family='Arial', size=16)).place(x=690, y=110)
VarMapNrNMP = tk.IntVar()
VarMapNrNMP.set(9)  # set the initial value
MapNrNMPspinbox = tk.Spinbox(NewMapsPage, from_=1, to=9, textvariable=VarMapNrNMP,font=Font(family='Arial', size=36))
MapNrNMPspinbox.place(x=690, y=140, height=70, width=85)
Small_imgBack=imgBack.subsample(2, 2)
ButtonBackHome = tk.Button(NewMapsPage, image=Small_imgBack, command=ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=60, width=60)






""" THE PLOT PAGE ***************************************************"""

TabPlot = ttk.Notebook(fen1)
tabPlotMain = tk.Frame(TabPlot, width=800, height=400)
tabPlotWheelMotor = tk.Frame(TabPlot, width=800, height=200)
tabPlotMowMotor = tk.Frame(TabPlot, width=800, height=200)
tabPlotPerimeter = tk.Frame(TabPlot, width=800, height=200)
tabPlotBattery = tk.Frame(TabPlot, width=800, height=200)
tabPlotImu = tk.Frame(TabPlot, width=800, height=200)

TabPlot.add(tabPlotMain, text="Main")
TabPlot.add(tabPlotWheelMotor, text="Wheels Motor")
TabPlot.add(tabPlotMowMotor, text="Mow Motor")
TabPlot.add(tabPlotPerimeter, text="Perimeter")
TabPlot.add(tabPlotBattery, text="Battery")
TabPlot.add(tabPlotImu, text="Imu")

TabPlot.place(x=0, y=0, height=400, width=800)

# 'Main
Frame11 = tk.Frame(tabPlotMain, relief=tk.GROOVE, borderwidth="3")
Frame11.place(x=10, y=20, height=80, width=680)

BtnMotPlotStopAll = tk.Button(Frame11, command=BtnMotPlotStopAll_click, text="Stop ALL the Data send by DUE")
BtnMotPlotStopAll.place(x=10, y=10, height=25, width=250)

# 'Wheel Motor
tk.Label(tabPlotWheelMotor, text="Mower Millis : ").place(x=300, y=0)
tk.Label(tabPlotWheelMotor, textvariable=tk_millis).place(x=400, y=0)
Frame12 = tk.Frame(tabPlotWheelMotor, relief=tk.GROOVE, borderwidth="3")
Frame12.place(x=10, y=20, height=80, width=680)
BtnMotPlotStartRec = tk.Button(Frame12, command=BtnMotPlotStartRec_click, text="Start")
BtnMotPlotStartRec.place(x=0, y=0, height=25, width=60)
BtnMotPlotStopRec = tk.Button(Frame12, command=BtnMotPlotStopRec_click, text="Stop")
BtnMotPlotStopRec.place(x=0, y=25, height=25, width=60)
SldMainWheelRefresh = tk.Scale(Frame12, from_=1, to=10, label='Refresh Rate per second', relief=tk.SOLID,
                               orient='horizontal')
SldMainWheelRefresh.place(x=70, y=0, width=250, height=50)

tk.Label(Frame12, text='Power', fg='green').place(x=400, y=0)
tk.Label(Frame12, text='Left', fg='green').place(x=350, y=15)
tk.Label(Frame12, textvariable=tk_motorLeftPower).place(x=400, y=15)
tk.Label(Frame12, text='Right', fg='green').place(x=350, y=35)
tk.Label(Frame12, textvariable=tk_motorRightPower).place(x=400, y=35)
tk.Label(Frame12, text='PWM', fg='green').place(x=550, y=0)
tk.Label(Frame12, textvariable=tk_motorLeftPWMCurr).place(x=550, y=15)
tk.Label(Frame12, textvariable=tk_motorRightPWMCurr).place(x=550, y=35)

# 'Mow Motor
tk.Label(tabPlotMowMotor, text="Mower Millis : ").place(x=300, y=0)
tk.Label(tabPlotMowMotor, textvariable=tk_millis).place(x=400, y=0)
Frame13 = tk.Frame(tabPlotMowMotor, relief=tk.GROOVE, borderwidth="3")
Frame13.place(x=10, y=20, height=80, width=680)
BtnMowPlotStartRec = tk.Button(Frame13, command=BtnMowPlotStartRec_click, text="Start")
BtnMowPlotStartRec.place(x=0, y=0, height=25, width=60)
BtnMowPlotStopRec = tk.Button(Frame13, command=BtnMowPlotStopRec_click, text="Stop")
BtnMowPlotStopRec.place(x=0, y=25, height=25, width=60)
SldMainMowRefresh = tk.Scale(Frame13, from_=1, to=10, label='Refresh Rate per second', relief=tk.SOLID,
                             orient='horizontal')
SldMainMowRefresh.place(x=70, y=0, width=250, height=50)

tk.Label(Frame13, text='Power', fg='green').place(x=400, y=0)
tk.Label(Frame13, textvariable=tk_motorMowPower).place(x=400, y=15)
tk.Label(Frame13, text='PWM', fg='green').place(x=550, y=0)
tk.Label(Frame13, textvariable=tk_motorMowPWMCurr).place(x=550, y=15)

# 'Battery'
tk.Label(tabPlotBattery, text="Mower Millis : ").place(x=300, y=0)
tk.Label(tabPlotBattery, textvariable=tk_millis).place(x=400, y=0)
Frame14 = tk.Frame(tabPlotBattery, relief=tk.GROOVE, borderwidth="3")
Frame14.place(x=10, y=20, height=80, width=680)
BtnBatPlotStartRec = tk.Button(Frame14, command=BtnBatPlotStartRec_click, text="Start")
BtnBatPlotStartRec.place(x=0, y=0, height=25, width=60)
BtnBatPlotStopRec = tk.Button(Frame14, command=BtnBatPlotStopRec_click, text="Stop")
BtnBatPlotStopRec.place(x=0, y=25, height=25, width=60)
SldMainBatRefresh = tk.Scale(Frame14, from_=1, to=100, label='Refresh Rate per minute', relief=tk.SOLID,
                             orient='horizontal')
SldMainBatRefresh.place(x=70, y=0, width=250, height=50)

tk.Label(Frame14, text='Charge', fg='green').place(x=350, y=15)
tk.Label(Frame14, text='Battery', fg='green').place(x=350, y=35)
tk.Label(Frame14, text='Sense', fg='green').place(x=400, y=0)
tk.Label(Frame14, textvariable=tk_chgSense).place(x=400, y=15)
tk.Label(Frame14, text='Voltage', fg='green').place(x=550, y=0)
tk.Label(Frame14, textvariable=tk_chgVoltage).place(x=550, y=15)
tk.Label(Frame14, textvariable=tk_batteryVoltage).place(x=550, y=35)

# 'Perimeter'
tk.Label(tabPlotPerimeter, text="Mower Millis : ").place(x=300, y=0)
tk.Label(tabPlotPerimeter, textvariable=tk_millis).place(x=400, y=0)
Frame15 = tk.Frame(tabPlotPerimeter, relief=tk.GROOVE, borderwidth="3")
Frame15.place(x=10, y=20, height=80, width=680)
BtnPeriPlotStartRec = tk.Button(Frame15, command=BtnPeriPlotStartRec_click, text="Start")
BtnPeriPlotStartRec.place(x=0, y=0, height=25, width=60)
BtnPeriPlotStopRec = tk.Button(Frame15, command=BtnPeriPlotStopRec_click, text="Stop")
BtnPeriPlotStopRec.place(x=0, y=25, height=25, width=60)
SldMainPeriRefresh = tk.Scale(Frame15, from_=1, to=10, label='Refresh Rate per second', relief=tk.SOLID,
                              orient='horizontal')
SldMainPeriRefresh.place(x=70, y=0, width=250, height=50)

tk.Label(Frame15, text='Mag', fg='green').place(x=400, y=0)
tk.Label(Frame15, text='Left', fg='green').place(x=350, y=15)
tk.Label(Frame15, textvariable=tk_perimeterMag).place(x=400, y=15)
tk.Label(Frame15, text='Right', fg='green').place(x=350, y=35)
tk.Label(Frame15, textvariable=tk_perimeterMagRight).place(x=400, y=35)

# 'Imu'
tk.Label(tabPlotImu, text="Mower Millis : ").place(x=0, y=200)
tk.Label(tabPlotImu, textvariable=tk_millis).place(x=100, y=200)
Frame16 = tk.Frame(tabPlotImu, relief=tk.GROOVE, borderwidth="3")
Frame16.place(x=5, y=5, height=155, width=390)
BtnImuPlotStartRec = tk.Button(Frame16, command=BtnImuPlotStartRec_click, text="Start")
BtnImuPlotStartRec.place(x=0, y=0, height=25, width=60)
BtnImuPlotStopRec = tk.Button(Frame16, command=BtnImuPlotStopRec_click, text="Stop")
BtnImuPlotStopRec.place(x=0, y=25, height=25, width=60)
SldMainImuRefresh = tk.Scale(Frame16, from_=1, to=10, label='Refresh Rate per seconde', relief=tk.SOLID,
                             orient='horizontal')
SldMainImuRefresh.place(x=70, y=0, width=250, height=50)

tk.Label(Frame16, text='Gyro', fg='green').place(x=50, y=75)
tk.Label(Frame16, text='Compass', fg='green').place(x=50, y=95)
tk.Label(Frame16, text='Value', fg='green').place(x=160, y=60)
tk.Label(Frame16, textvariable=tk_gyroYaw).place(x=160, y=75)
tk.Label(Frame16, textvariable=tk_compassYaw).place(x=160, y=95)

ButtonBackHome = tk.Button(TabPlot, image=imgBack, command=ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=120, width=120)

"""
 THE INFO PAGE ***************************************************
"""
InfoPage = tk.Frame(fen1)
InfoPage.place(x=0, y=0, height=400, width=800)
Infoline1 = tk.StringVar()
LabInfoline = tk.Label(InfoPage, textvariable=Infoline1)
LabInfoline.place(x=10, y=10, height=25, width=450)
Infoline2 = tk.IntVar()
LabInfoline = tk.Label(InfoPage, textvariable=Infoline2)
LabInfoline.place(x=10, y=40, height=25, width=300)
Infoline3 = tk.IntVar()
LabInfoline = tk.Label(InfoPage, textvariable=Infoline3)
LabInfoline.place(x=10, y=70, height=25, width=300)
Infoline4 = tk.IntVar()
LabInfoline = tk.Label(InfoPage, textvariable=Infoline4)
LabInfoline.place(x=10, y=100, height=25, width=300)
Infoline5 = tk.IntVar()
LabInfoline = tk.Label(InfoPage, textvariable=Infoline5)
LabInfoline.place(x=10, y=130, height=25, width=300)

FrameGpsInfo = tk.Frame(InfoPage)
FrameGpsInfo.place(x=10, y=180, height=150, width=520)
FrameGpsInfo.configure(borderwidth="3", relief=tk.GROOVE, background="#d9d9d9", highlightbackground="#d9d9d9",
                       highlightcolor="black")

GpsInfoline1 = tk.StringVar()
LabInfoline = tk.Label(FrameGpsInfo, textvariable=GpsInfoline1)
LabInfoline.place(x=10, y=10, height=25, width=150)

GpsInfoline2 = tk.StringVar()
LabInfoline = tk.Label(FrameGpsInfo, textvariable=GpsInfoline2)
LabInfoline.place(x=10, y=35, height=25, width=150)

GpsInfoline3 = tk.StringVar()
LabInfoline = tk.Label(FrameGpsInfo, textvariable=GpsInfoline3)
LabInfoline.place(x=10, y=60, height=25, width=150)

GpsInfoline4 = tk.StringVar()
LabInfoline = tk.Label(FrameGpsInfo, textvariable=GpsInfoline4)
LabInfoline.place(x=10, y=85, height=25, width=150)

GpsInfoline5 = tk.StringVar()
LabInfoline = tk.Label(FrameGpsInfo, textvariable=GpsInfoline5)
LabInfoline.place(x=10, y=110, height=25, width=150)

GpsInfoline6 = tk.StringVar()
LabInfoline = tk.Label(FrameGpsInfo, textvariable=GpsInfoline6)
LabInfoline.place(x=160, y=10, height=25, width=150)

GpsInfoline7 = tk.StringVar()
LabInfoline = tk.Label(FrameGpsInfo, textvariable=GpsInfoline7)
LabInfoline.place(x=160, y=35, height=25, width=150)

GpsInfoline8 = tk.StringVar()
LabInfoline = tk.Label(FrameGpsInfo, textvariable=GpsInfoline8)
LabInfoline.place(x=160, y=60, height=25, width=150)

GpsInfoline9 = tk.StringVar()
LabInfoline = tk.Label(FrameGpsInfo, textvariable=GpsInfoline9)
LabInfoline.place(x=160, y=85, height=25, width=150)

GpsInfoline10 = tk.StringVar()
LabInfoline = tk.Label(FrameGpsInfo, textvariable=GpsInfoline10)
LabInfoline.place(x=160, y=110, height=25, width=150)

GpsInfoline11 = tk.StringVar()
LabInfoline = tk.Label(FrameGpsInfo, textvariable=GpsInfoline11)
LabInfoline.place(x=310, y=10, height=25, width=150)

ButtonBackHome = tk.Button(InfoPage, image=imgBack, command=ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=120, width=120)

""" THE MAPS PAGE ***************************************************"""

def ButtonCreateNewMap_click():
    
    mymower.focusOnPage = 12
    NewMapsPage.tkraise()

    
    
def onMap0click(event):
    if event.button is MouseButton.LEFT:
        print("left")
    if event.button is MouseButton.RIGHT:
        print("right")
    
    if event.button is MouseButton.FORWARD:
        print("roll up")
    if event.button is MouseButton.BACK:
        print("roll rev")
        
    if (event.dblclick):
        print(f'data coords {event.xdata} {event.ydata}')
    else:
        print("simple")

ind = None
selectedPoint = None
def onMapPickPoint(event):
    global selectedPoint
    global ind
    if event.mouseevent.button is MouseButton.LEFT :
        print("UP left")
        #line = event.artist
        #xdata, ydata = line.get_data()
        ind = event.ind
        #print('Mpa pick: ',ind,x_perimeter[ind],y_perimeter[ind])
    if event.mouseevent.button is MouseButton.RIGHT:
         print("right")
         print(event.mouseevent.xdata)
    

def onMap1click(event):
    global ind
    if event.button is MouseButton.LEFT:
        #print("up gauche")
        print('Mpa pick: ',ind,x_perimeter[ind],y_perimeter[ind])

def onMap1release(event):
    return
    if event.button is MouseButton.LEFT:
        #global option
        if (option.get()=='Peri'):
            x_perimeter[ind]=event.xdata
            y_perimeter[ind]=event.ydata
            mapPagePlotRefresh(1,0,0,0)
        if (option.get()=='Mow'):
            mapPagePlot(0,1,0,0)
        if (option.get()=='Exclu'):
            mapPagePlot(0,0,1,0)
        if (option.get()=='Dock'):
            print("dock")
            x_dock[ind]=event.xdata
            y_dock[ind]=event.ydata
            mapPagePlotRefresh(0,0,0,1)

       
        

def onMap1move(event):
    if event.button is MouseButton.LEFT:
        #return
        if (option.get()=='Peri'):
            x_perimeter[ind]=event.xdata
            y_perimeter[ind]=event.ydata
            mapPagePlotRefresh(1,0,0,0)
        if (option.get()=='Mow'):
            mapPagePlot(0,1,0,0)
        if (option.get()=='Exclu'):
            mapPagePlot(0,0,1,0)
        if (option.get()=='Dock'):
            print("dock")
            x_dock[ind]=event.xdata
            y_dock[ind]=event.ydata
            mapPagePlotRefresh(0,0,0,1)

    
        

#def move_on_map(event):
 #   if event.inaxes:
  #      print(f'data coords {event.xdata} {event.ydata},',
   #           f'pixel coords {event.x} {event.y}')


showdot = False
def Button_showDot_Nr_click():
    global showdot
    showdot = True
    showFullMapTab()
    print("_showDot_Nr")



def showFullMapTab():  # tab 0 show the full map

    ax[0].clear()
    fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + "/crcMapList.npy"
    mymower.totalMowingArea = 0
    if (os.path.exists(fileName)):
        crcMapList = np.load(fileName)

        for idx in range(int(len(crcMapList))):
            mapNr = int(crcMapList[idx, 0])

            fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                       "/maps" + "{0:0>2}".format(mapNr) + "/PERIMETER.npy"
            if (os.path.exists(fileName)):
                perimeterArray = np.load(fileName)
                polygon1 = Polygon(np.squeeze(perimeterArray))
                mymower.polygon[mapNr] = polygon1  # keep the polygon for later search
                mymower.totalMowingArea = mymower.totalMowingArea + int(
                    polygon1.area)  # print(polygon1.centroid.coords[0][0]) center of polygon
                # draw perimeter
                x = np.zeros(int(len(perimeterArray) + 1))
                y = np.zeros(int(len(perimeterArray) + 1))
                for idx1 in range(int(len(perimeterArray))):

                    x[idx1] = perimeterArray[idx1][0]
                    y[idx1] = perimeterArray[idx1][1]
                    if showdot:
                        ax[0].text(x[idx1], y[idx1], idx1, fontsize=8)
                    # close the drawing
                x[idx1 + 1] = perimeterArray[0][0]
                y[idx1 + 1] = perimeterArray[0][1]
                ax[0].text(polygon1.centroid.coords[0][0], polygon1.centroid.coords[0][1], mapNr, fontsize=8,
                           bbox=dict(facecolor='yellow', alpha=0.4))
                ax[0].plot(x, y, color='r', linewidth=0.4,picker=True)  # ,marker='.',markersize=2)

            else:
                print("no perimeter data for this map " + str(mapNr))

            # draw exclusion

            fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                       "/maps" + "{0:0>2}".format(mapNr) + "/MAIN.npy"
            nbTotalExclusion1 = 0

            if (os.path.exists(fileName)):
                maindata1 = np.load(fileName)
                nbTotalExclusion1 = int(maindata1[5])

            else:
                print("no main.npy for this house")

            for mymower.exclusionNr in range(nbTotalExclusion1):
                fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                           "/maps" + "{0:0>2}".format(mapNr) + "/EXCLUSION" + "{0:0>2}".format(
                    mymower.exclusionNr) + ".npy"

                if (os.path.exists(fileName)):
                    excluPts = np.load(fileName)
                    polygon1 = Polygon(np.squeeze(excluPts))
                    # mymower.polygon[mapNr] = polygon1 #keep the polygon for later search
                    mymower.totalMowingArea = mymower.totalMowingArea - int(polygon1.area)
                    x_lon = np.zeros(int(len(excluPts) + 1))
                    y_lat = np.zeros(int(len(excluPts) + 1))
                    if (int(len(excluPts)) >= 3):
                        for ip in range(int(len(excluPts))):
                            x_lon[ip] = excluPts[ip][0]
                            y_lat[ip] = excluPts[ip][1]
                        x_lon[ip + 1] = excluPts[0][0]
                        y_lat[ip + 1] = excluPts[0][1]
                    ax[0].plot(x_lon, y_lat, color='r', linewidth=1)

                else:
                    messagebox.showwarning('warning', "No exclusion points for this map ?????")
            # draw dock
            dockCRC = 0
            fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                       "/maps" + "{0:0>2}".format(mapNr) + "/DOCK.npy"
            if (os.path.exists(fileName)):
                dockPts = np.load(fileName)
                x_lon = np.zeros(int(len(dockPts)))
                y_lat = np.zeros(int(len(dockPts)))
                for ip in range(int(len(dockPts))):
                    x_lon[ip] = dockPts[ip][0]
                    y_lat[ip] = dockPts[ip][1]

                ax[0].plot(x_lon, y_lat, color='b', linewidth=0.6)

            else:
                messagebox.showwarning('warning', "No dock points for this map ?????")


    else:
        print("error no crcMapList.npy for this house")



    ax[0].clear()
     # draw newPerimeter
    toolbar = VerticalNavigationToolbar2Tk(canvas[0], MapsPage)
    toolbar.place(x=5, y=40)
    fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + "/test.npy"
    if (os.path.exists(fileName)):
        perimeterArray = np.load(fileName)

        polygon1 = Polygon(np.squeeze(perimeterArray))
        from shapely.validation import explain_validity
       
        

        mymower.polygon[mapNr] = polygon1  # keep the polygon for later search
        #mymower.totalMowingArea = mymower.totalMowingArea + int(polygon1.area)  # print(polygon1.centroid.coords[0][0]) center of polygon
                # draw perimeter
        x, y = polygon1.exterior.xy
        ax[0].plot(x, y, color='r', linewidth=0.4,picker=True,marker='.')  # ,markersize=2)ttom=0)

        print(explain_validity(polygon1))
        print("valid  polygon init " ,polygon1.is_valid)
        print(type(polygon1))
        new_polygon = polygon1.buffer(0) #try to remove err on polygon
        print(type(new_polygon))
        print("valid  polygon2 " ,new_polygon.is_valid)
        polygon2=new_polygon.simplify(tolerance=0.03, preserve_topology=True) #tolerance is 2 cm here
        print("poly 2 type " ,type(polygon2))
        #print(polygon2)
       
        x, y = polygon2.exterior.xy
        ax[1].plot(x,y,color='g', linewidth=0.4,picker=True,marker='.')
        
        
        parameters=PathPlannerCfg()
        parameters.mowarea=True
        parameters.mowborder=False
        parameters.mowexclusion=False
        start_pos=[0.0, 0.0]

        polygon4_list=calc(polygon2, parameters, start_pos)
        polygon4 = Polygon(np.squeeze(polygon4_list))
        
        #x, y = polygon4.geoms[0].exterior.xy

        x = np.zeros(int(len(polygon4_list) + 1))
        y = np.zeros(int(len(polygon4_list) + 1))
        for idx2 in range(int(len(polygon4_list))):

            x[idx2] = polygon4_list[idx2][0]
            y[idx2] = polygon4_list[idx2][1]
        ax[2].plot(x,y,color='g', linewidth=0.4,picker=True,marker='.')

        x, y = polygon4.exterior.xy
        ax[3].plot(x,y,color='black', linewidth=0.4,picker=True,marker='.')










        
    canvas[0].draw()

    MapsInfoline1.configure(text=" Total Area " + str(mymower.totalMowingArea) + " m2")
    #plt.show()

##    InfoHouseNrtxt.configure(text=mymower.House)
##    InfoHouseNrtxt.update()
def ButtonEditMap_click():
   
    if (option.get()=='Peri'):
        mapPagePlot(1,0,0,0)
    if (option.get()=='Mow'):
        mapPagePlot(0,1,0,0)
    if (option.get()=='Exclu'):
        mapPagePlot(0,0,1,0)
    if (option.get()=='Dock'):
        mapPagePlot(0,0,0,1)
    

    return

def onTabChange(event):
    mymower.mapSelected = int(MapsPage.index("current"))
    if (mymower.mapSelected == 0):
        
        ButtonDeleteMap.place_forget()
        ButtonExportMap.place_forget()
        ButtonImportMap.place_forget()
        Button_showDot_Nr.place(x=680, y=180, height=60, width=110)
        FrameEditSelection.place(x=420, y=90, height=120, width=120)
        ButtonCreateNewMap.place(x=680, y=25, height=30, width=110)

        showFullMapTab()
        return
    else:
        ButtonCreateNewMap.place_forget()
        FrameEditSelection.place_forget()
        
        ButtonDeleteMap.place(x=680, y=60, height=30, width=110)
        ButtonExportMap.place(x=680, y=115, height=60, width=110)
        ButtonImportMap.place(x=680, y=180, height=60, width=110)
        Button_showDot_Nr.place_forget()

    fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
               "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/MAIN.npy"
    if (os.path.exists(fileName)):
        maindata = np.load(fileName)
        mymower.perimeterPointsCount = int(maindata[0])
        mymower.exclusionPointsCount = int(maindata[1])
        mymower.dockPointsCount = int(maindata[2])
        mymower.mowPointsCount = int(maindata[3])
        mymower.freePointsCount = int(maindata[4])
        mymower.nbTotalExclusion = int(maindata[5])
        mymower.fileMapCRC = int(maindata[6])
        Infolinetxt = ""
        Infolinetxt = Infolinetxt + "Perimeter points: " + str(mymower.perimeterPointsCount) + '\n'
        Infolinetxt = Infolinetxt + "Nb Exclusions: " + str(mymower.nbTotalExclusion) + '\n'
        Infolinetxt = Infolinetxt + "Exclusion points: " + str(mymower.exclusionPointsCount) + '\n'
        Infolinetxt = Infolinetxt + "Mow points: " + str(mymower.mowPointsCount) + '\n'
        Infolinetxt = Infolinetxt + "Dock points: " + str(mymower.dockPointsCount) + '\n'
        Infolinetxt = Infolinetxt + "Free points: " + str(mymower.freePointsCount) + '\n'
        Infolinetxt = Infolinetxt + "File CRC: " + str(mymower.fileMapCRC) + '\n'
        MapsInfoline1.configure(text=Infolinetxt)
        InfoHouseNrtxt.configure(text=mymower.House)
        InfoHouseNrtxt.update()
        #mapPagePlot(1,1,1,1)
        toolbar = VerticalNavigationToolbar2Tk(canvas[mymower.mapSelected], MapsPage)
        toolbar.place(x=5, y=40)
        
        

    else:
        Infolinetxt = ""
        MapsInfoline1.configure(text=Infolinetxt)
        InfoHouseNrtxt.configure(text=mymower.House)
        InfoHouseNrtxt.update()


def delete_map():
    returnval = messagebox.askyesno('Info', "Do you want to remove the MAP" + "{0:0>2}".format(mymower.mapSelected))
    if returnval:
        dir_path = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                   "/maps" + "{0:0>2}".format(mymower.mapSelected)
        try:
            for filepath in os.listdir(dir_path):
                os.remove(dir_path + "/{}".format(filepath))
            os.rmdir(dir_path)
        except:
            print("no file")

        # update the main CRC datalist according to map nr
        fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + "/crcMapList.npy"
        if (os.path.exists(fileName)):
            crcMapList = np.load(fileName)
            print(fileName)
            print(crcMapList)
            find_row = np.where(crcMapList[:, 0] == mymower.mapSelected)
            result = crcMapList[find_row]
            if (result.size != 0):
                if (result[0, 0] == mymower.mapSelected):
                    crcMapList = np.delete(crcMapList, (find_row), axis=0)
                    print("new")
                    print(crcMapList)
                    np.save(fileName, crcMapList, allow_pickle=True, fix_imports=True)
                else:
                    print("Error into crcMaplist")


            else:
                print("Map not present into crcMaplist")

        else:
            print("error no crcMaplist for this house")

        consoleInsertText("Map " + "{0:0>2}".format(mymower.mapSelected) + ' is remove from Pi \n')
        messagebox.showinfo('info', "Map " + "{0:0>2}".format(mymower.mapSelected) + ' is remove from Pi')
        print(mymower.mapSelected)
        mymower.mapCRC = 0
        ax[mymower.mapSelected].clear()
        fig[mymower.mapSelected].clear()
        canvas[mymower.mapSelected].draw()
        # onTabChange(mymower.mapSelected)
        # mymower.mapSelected=0


def wait_the_feedBack():
    mymower.isSendindMap = True
    while (mymower.isSendindMap):
        time.sleep(0.1)

        print(" wait feed back \n")
        if (Due_Serial.inWaiting() != 0):
            mymower.dueSerialReceived = Due_Serial.readline()
            if str(mymower.dueSerialReceived) != "b''":
                mymower.dueSerialReceived = mymower.dueSerialReceived.decode('utf-8', errors='ignore')
                if mymower.dueSerialReceived[:1] != '$':  # it is console message because the first digit is not $
                    if (len(mymower.dueSerialReceived)) > 2:
                        decode_AT_message(mymower.dueSerialReceived)
                else:
                    consoleInsertText("recu sentense starting by $ " + '\n')
                    consoleInsertText(mymower.dueSerialReceived)


def ButtonExportMap_click():
    returnval = messagebox.askyesno('Info', "You are going to replace the MAP locate into mower by this one")
    if (returnval):

        export_map_to_mower(mymower.House, mymower.mapSelected)



def export_map_to_mower(House, mapNr):  # export the into mower see decode AT message on RP
    #stop Gps reading
    consoleInsertText("Stop GPS reading" + '\n')
    message = "AT+Y4"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)
    if (mapNr == 0):
        print("map 0 can't be exported to mower")
        return
    mymower.House = House
    mymower.mapSelected = mapNr
    mymower.finishedUploadingMap = False
    # read the main data of the map

    fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
               "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/MAIN.npy"
    if (os.path.exists(fileName)):
        maindata = np.load(fileName)
        mymower.perimeterPointsCount = int(maindata[0])
        mymower.exclusionPointsCount = int(maindata[1])
        mymower.dockPointsCount = int(maindata[2])
        mymower.mowPointsCount = int(maindata[3])
        mymower.freePointsCount = int(maindata[4])
        mymower.nbTotalExclusion = int(maindata[5])
        mymower.fileMapCRC = int(maindata[6])
    else:
        print("Error can't find the main data for this map " + str(mapNr))
        return

    mymower.isSendindMap = True
    mymower.startMowPointIdx = 0

    # part 1 AT+W map data waypoint

    trame_size = 30  # default value=30
    pt = 0
    pt_total = 0
    crc_point = 0

    ################ perimeter
    print("perimeter")
    crc_perimeter = 0
    fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
               "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/PERIMETER.npy"
    if (os.path.exists(fileName)):
        crc_perimeter = 0
        perimeterPts = np.load(fileName)
        perimeterPts_count = int(len(perimeterPts))
        if ((mymower.perimeterPointsCount) != perimeterPts_count):
            print("Error in the perimeter point count")
        else:
            print("Nb_point : ", perimeterPts_count)

        toSend = "AT+W,0,"

        for ip in range(int(len(perimeterPts))):
            toSend = toSend + str(perimeterPts[ip][0]) + "," + str(perimeterPts[ip][1]) + ","
            crc_perimeter = crc_perimeter + int(100 * (perimeterPts[ip][0])) + int(100 * (perimeterPts[ip][1]))
            pt = pt + 1
            pt_total = pt_total + 1
            if (pt >= trame_size):
                toSend = toSend + '\r'
                print(toSend)
                consoleInsertText(" Send " + toSend)
                send_serial_message(toSend)
                wait_the_feedBack()
                # prepare the new sentence
                toSend = "AT+W," + str(pt_total) + ","
                pt = 0
                # crc_point=0
    else:
        messagebox.showwarning('warning', "No perimeter for this map ???")
    consoleInsertText("Perimeter points send finish CRC: " + str(crc_perimeter) + '\n')
    print("fin de perimeter")

    ################ exclusion

    crc_exclusion = 0
    for mymower.exclusionNr in range(int(mymower.nbTotalExclusion)):
        print("exclusion : " + str(mymower.exclusionNr))
        fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                   "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/EXCLUSION" + "{0:0>2}".format(
            mymower.exclusionNr) + ".npy"

        if (os.path.exists(fileName)):
            crc_exclusion = 0
            exclusionPts = np.load(fileName)

            for ip in range(int(len(exclusionPts))):
                toSend = toSend + str(exclusionPts[ip][0]) + "," + str(exclusionPts[ip][1]) + ","
                crc_exclusion = crc_exclusion + int(100 * (exclusionPts[ip][0])) + int(100 * (exclusionPts[ip][1]))
                pt = pt + 1
                pt_total = pt_total + 1
                if (pt >= trame_size):
                    toSend = toSend + '\r'
                    print(toSend)
                    send_serial_message(toSend)
                    time.sleep(1)
                    checkSerial()
                    # prepare the new sentence
                    toSend = "AT+W," + str(pt_total) + ","
                    pt = 0


        else:
            messagebox.showwarning('warning', "No exclusion for this map ???")

        consoleInsertText("Exclusion points send finish CRC: " + str(crc_exclusion) + '\n')
    print(int(mymower.nbTotalExclusion))
    print("fin de exclusion")

    ################ dock
    print("Dock")
    crc_dock = 0
    fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
               "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/DOCK.npy"
    if (os.path.exists(fileName)):

        dockPts = np.load(fileName)
        # print(dockPts)
        dockPts_count = int(len(dockPts))
        if ((mymower.dockPointsCount) != dockPts_count):
            print("Error in the dock point count")
        else:
            print("Nb_point : ", dockPts_count)

        for ip in range(int(len(dockPts))):
            toSend = toSend + str(dockPts[ip][0]) + "," + str(dockPts[ip][1]) + ","
            crc_dock = crc_dock + int(100 * (dockPts[ip][0])) + int(100 * (dockPts[ip][1]))

            pt = pt + 1
            pt_total = pt_total + 1
            if (pt >= trame_size):
                toSend = toSend + '\r'
                print(toSend)
                send_serial_message(toSend)
                # prepare the new sentence
                toSend = "AT+W," + str(pt_total) + ","
                pt = 0

    print("fin de dock")
    consoleInsertText("Dock points send finish CRC: " + str(crc_dock) + '\n')

    ################ mow
    print("Mow")
    fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
               "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/MOW.npy"
    if (os.path.exists(fileName)):
        crc_mow = 0
        mowPts = np.load(fileName)
        # mowPts = mowPts1.astype(float)
        mowPts_count = int(len(mowPts))
        mymower.stopMowPointIdx = mowPts_count
        tk_labelStopIdx.set(mymower.stopMowPointIdx)
        if ((mymower.mowPointsCount) != mowPts_count):
            print("Error in the mow point count")
        else:
            print("Nb_point : ", mowPts_count)

        for ip in range(int(len(mowPts))):
            toSend = toSend + str(mowPts[ip][0]) + "," + str(mowPts[ip][1]) + ","
            crc_mow = crc_mow + int(100 * (mowPts[ip][0])) + int(100 * (mowPts[ip][1]))
            pt = pt + 1
            pt_total = pt_total + 1
            if (pt >= trame_size):
                print("Mow pts")
                print(toSend)
                toSend = toSend + '\r\n'
                send_serial_message(toSend)

                wait_the_feedBack()

                # prepare the new sentence
                toSend = "AT+W," + str(pt_total) + ","
                pt = 0




    else:
        messagebox.showwarning('warning', "No mowing points for this map ???")
    print("fin de mow")
    consoleInsertText("Mow points send finish CRC: " + str(crc_mow) + '\n')

    ################ free

    fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
               "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/FREE.npy"
    if (os.path.exists(fileName)):
        crc_free = 0
        freePts = np.load(fileName)
        for ip in range(int(len(freePts))):
            toSend = toSend + str(freePts[ip][0]) + "," + str(freePts[ip][1]) + ","
            crc_free = crc_free + int(100 * (freePts[ip][0])) + int(100 * (freePts[ip][1]))
            pt = pt + 1
            pt_total = pt_total + 1
            if (pt >= trame_size):
                print(toSend)
                toSend = toSend + '\r'
                send_serial_message(toSend)
                wait_the_feedBack()
                # prepare the new sentence
                toSend = "AT+W," + str(pt_total) + ","
                pt = 0

    else:
        messagebox.showwarning('warning', "No free points for this map ???")
    print("fin de free")
    consoleInsertText("Free points send finish CRC: " + str(crc_free) + '\n')

    if (toSend != ("AT+W," + str(pt_total) + ",")):
        print("reste final")
        # print(toSend)
        toSend = toSend + '\r'
        send_serial_message(toSend)
        wait_the_feedBack()
        pt = 0

    # part 2  : AT+N  points description

    toSend = "AT+N," + str(mymower.perimeterPointsCount) + "," + str(mymower.exclusionPointsCount)
    toSend = toSend + "," + str(mymower.dockPointsCount) + "," + str(mymower.mowPointsCount)
    toSend = toSend + "," + str(mymower.freePointsCount) + "," + str(crc_point)
    # print(toSend)
    toSend = toSend + '\r'
    send_serial_message(toSend)
    consoleInsertText("Points description send finish " + '\n')

    # part 3  : AT+X  exclusion description
    mymower.exclusionNr = 0
    toSend = "AT+X,0,"
    for mymower.exclusionNr in range(int(mymower.nbTotalExclusion)):

        fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                   "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/EXCLUSION" + "{0:0>2}".format(
            mymower.exclusionNr) + ".npy"

        if (os.path.exists(fileName)):
            exclusion_NP = np.load(fileName)
            toSend = toSend + str(len(exclusion_NP)) + ","

    # print (toSend)
    toSend = toSend + '\r'
    send_serial_message(toSend)
    consoleInsertText("Exclusion description send finish " + '\n')

    crc_point = float(crc_perimeter) + float(crc_exclusion) + float(crc_dock) + float(crc_mow) + float(crc_free)
    consoleInsertText(str(float(crc_point)) + '\n')
    print("crc_perimeter ", crc_perimeter)
    print("crc_exclusion ", crc_exclusion)
    print("crc_dock ", crc_dock)
    print("crc_mow ", crc_mow)
    print("crc_free ", crc_free)
    print("CRC MAP ", crc_point)

    tk_labelStartIdx.set(0)

    mymower.isSendindMap = False
    #restart reading Gps
    consoleInsertText("Start GPS reading" + '\n')
    message = "AT+Y5"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


# load the active map locate into mower and save it on pi SD see decode AT message on RN response
def import_map_from_mower():
    returnval = messagebox.askyesno('Info', "You are going to replace the actual MAP" + str(
        mymower.mapSelected) + " by the one locate in the mower")
    if returnval:
        message = "AT+RN"
        message = str(message)
        message = message + '\r'
        send_serial_message(message)
    else:
        messagebox.showinfo('info', "No Change made in the actual map")

def mapPagePlotRefresh(draw_perimeter,draw_mow,draw_exclusion,draw_dock):
   
    ax[mymower.mapSelected].clear()
    if draw_perimeter :
        ax[mymower.mapSelected].plot(x_perimeter, y_perimeter, color='r', linewidth=0.4, marker='.', markersize=2,picker=True)

    if draw_exclusion :
        return
    
    if draw_dock : 
        ax[mymower.mapSelected].plot(x_dock, y_dock, color='b', linewidth=0.6,marker='.', markersize=2,picker=True)
           




    canvas[mymower.mapSelected].draw()       

    
    

def mapPagePlot(draw_perimeter,draw_mow,draw_exclusion,draw_dock):
    global x_perimeter
    global y_perimeter
    global x_dock
    global y_dock
    
    # print("PLOT CRC LIST")
    ax[mymower.mapSelected].clear()
    # fig[mymower.mapSelected].clear()
    
    # draw perimeter
    perimeterCRC = 0
    if draw_perimeter :
        fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
               "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/PERIMETER.npy"

        if (os.path.exists(fileName)):
            perimeter = np.load(fileName)
            x_perimeter = np.zeros(int(len(perimeter) + 1))
            y_perimeter = np.zeros(int(len(perimeter) + 1))
            #need to compute the CRC drawing
            for ip in range(int(len(perimeter))):
                x_perimeter[ip] = perimeter[ip][0]
                y_perimeter[ip] = perimeter[ip][1]
                if showdot :
                    ax[mymower.mapSelected].text(x_perimeter[ip], y_perimeter[ip], ip, fontsize=8)
                perimeterCRC = perimeterCRC + int(100 * x_perimeter[ip]) + int(100 * y_perimeter[ip])

            # close the drawing
            x_perimeter[ip + 1] = perimeter[0][0]
            y_perimeter[ip + 1] = perimeter[0][1]
            ax[mymower.mapSelected].plot(x_perimeter, y_perimeter, color='r', linewidth=0.4, marker='.', markersize=2,picker=True)
            #fig[mymower.mapSelected].subplots_adjust(left=0, right=1, top=1, bottom=0)
           
           
            #polygon6 = Polygon(np.squeeze(perimeter))
            #ax[mymower.mapSelected].plot(*polygon6.exterior.xy, color='r', linewidth=0.4, marker='.', markersize=2,picker=True)
            #bber100
            #show simplify polygon for testing
            # polygon1 = Polygon(np.squeeze(perimeter))
            # polygon2=polygon1.simplify(tolerance=0.02, preserve_topology=True) #tolerance is 2 cm here
            # a, b = polygon2.exterior.xy
            # print(len(a),len(perimeter))
            # ax[mymower.mapSelected].plot(a,b)
   
        else:
            messagebox.showwarning('warning', "No perimeter for this map ???")

   

    # draw mow
    mowCRC = 0
    if draw_mow :        
        fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
               "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/MOW.npy"

        if (os.path.exists(fileName)):
            mowPts = np.load(fileName)
            x_mow = np.zeros(int(len(mowPts)))
            y_mow = np.zeros(int(len(mowPts)))
            for ip in range(int(len(mowPts))):
                x_mow[ip] = mowPts[ip][0]
                y_mow[ip] = mowPts[ip][1]
                mowCRC = mowCRC + int(100 * x_mow[ip]) + int(100 * y_mow[ip])
            ax[mymower.mapSelected].plot(x_mow, y_mow, color='g', linewidth=0.2)
        # canvas[mymower.mapSelected].draw()
        else:
            messagebox.showwarning('warning', "No mowing points for this map ?????")

        # print ("mowCRC ",mowCRC)

    # draw exclusion
    exclusionCRC = 0
    if draw_exclusion :        
        for mymower.exclusionNr in range(int(mymower.nbTotalExclusion)):
            fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                   "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/EXCLUSION" + "{0:0>2}".format(mymower.exclusionNr) + ".npy"

            if (os.path.exists(fileName)):
                mowPts = np.load(fileName)
                x_lon = np.zeros(int(len(mowPts) + 1))
                y_lat = np.zeros(int(len(mowPts) + 1))
                if (int(len(mowPts)) >= 3):
                    for ip in range(int(len(mowPts))):
                        x_lon[ip] = mowPts[ip][0]
                        y_lat[ip] = mowPts[ip][1]
                        exclusionCRC = exclusionCRC + int(100 * x_lon[ip]) + int(100 * y_lat[ip])
                        # close the drawing
                    x_lon[ip + 1] = mowPts[0][0]
                    y_lat[ip + 1] = mowPts[0][1]

                    ax[mymower.mapSelected].plot(x_lon, y_lat, color='r', linewidth=1,marker='.', markersize=2,picker=True)
                # canvas[mymower.mapSelected].draw()
            else:
                messagebox.showwarning('warning', "No exclusion points for this map ?????")
        # print("exclusionCRC ",exclusionCRC)

    # draw dock
    dockCRC = 0
    if draw_dock :        
        fileName = cwd + "/House" + "{0:0>2}".format(mymower.House) + \
               "/maps" + "{0:0>2}".format(mymower.mapSelected) + "/DOCK.npy"

        if (os.path.exists(fileName)):
            mowPts = np.load(fileName)
            # print(mowPts)
            x_dock = np.zeros(int(len(mowPts)))
            y_dock = np.zeros(int(len(mowPts)))
            for ip in range(int(len(mowPts))):
                x_dock[ip] = mowPts[ip][0]
                y_dock[ip] = mowPts[ip][1]
                dockCRC = dockCRC + int(100 * x_dock[ip]) + int(100 * y_dock[ip])
            ax[mymower.mapSelected].plot(x_dock, y_dock, color='b', linewidth=0.6,marker='.', markersize=2,picker=True)
            # canvas[mymower.mapSelected].draw()
        else:
            messagebox.showwarning('warning', "No dock points for this map ?????")

    # print("dockCRC " , dockCRC)
    mymower.plotMapCRC = perimeterCRC + mowCRC + dockCRC + exclusionCRC
    ctrl = int(mymower.plotMapCRC) - int(mymower.fileMapCRC)
    if draw_dock & draw_exclusion & draw_mow & draw_perimeter & (abs(ctrl) > mymower.mapCrcRoundingRange):
        messagebox.showwarning('warning',
                               "Issue into map file Try to import again this map PlotCRC / SunrayCRC : " + str(
                                   mymower.plotMapCRC) + " / " + str(mymower.fileMapCRC))

    # end of plot

    
    fig[mymower.mapSelected].subplots_adjust(left=0, right=1, top=1, bottom=0)
    
    
    # ax[uu].axis('off')
    # ax[uu].subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)
    # fig[uu](figsize=(8, 6), dpi=80)
    # ax[mymower.mapSelected].set_xlabel('xlabel', fontsize=8)
    # ax[mymower.mapSelected].set_ylabel('ylabel', fontsize=8)
    # ax[uu].rcParams({'font.size': 22})
    canvas[mymower.mapSelected].draw()

    


fig = [FigureCanvasTkAgg] * max_map_inUse

ax = [plt] * max_map_inUse
#toolbar = [None] * max_map_inUse
for uu in range(max_map_inUse):
    fig[uu], ax[uu] = plt.subplots()

#zoom_factory(ax[1])
#ph1=panhandler(fig[1], button=2)   

MapsPage = ttk.Notebook(fen1)
FrameMapsPage = [None] * max_map_inUse
canvas = [None] * max_map_inUse

for i in range(max_map_inUse):
    FrameMapsPage[i] = tk.Frame(MapsPage, borderwidth="1", relief=tk.SOLID)
    canvas[i] = FigureCanvasTkAgg(fig[i], master=FrameMapsPage[i])
    canvas[i].get_tk_widget().place(x=60, y=5, width=350, height=350)
    if (i == 0):
        MapsPage.add(FrameMapsPage[i], text="House")
    else:
        MapsPage.add(FrameMapsPage[i], text=str(i))

#binding_id = canvas[0].mpl_connect('motion_notify_event', move_on_map)
#cid0 = canvas[0].mpl_connect('button_press_event', onMap0click)
cid1 = canvas[1].mpl_connect('button_press_event', onMap1click)
cid2 = canvas[1].mpl_connect('button_release_event', onMap1release)
cid3 = canvas[1].mpl_connect('motion_notify_event', onMap1move)


cid10 = canvas[1].mpl_connect('pick_event', onMapPickPoint)
MapsPage.place(x=0, y=0, height=400, width=800)
MapsPage.bind("<<NotebookTabChanged>>", onTabChange)

MapsInfoline1 = tk.Label(MapsPage, text="Info maps")
MapsInfoline1.place(x=420, y=170, height=300, width=150)

FrameEditSelection=tk.Frame(MapsPage, borderwidth="1", relief=tk.SOLID)
FrameEditSelection.place(x=410, y=90, height=100, width=100)

ButtonEditMap = tk.Button(FrameEditSelection, text="Edit", command=ButtonEditMap_click)
ButtonEditMap.place(x=15, y=60, height=30, width=30)




option = tk.StringVar()
#option="Peri"
R1 = tk.Radiobutton(FrameEditSelection, text="Perimeter", value="Peri", var=option)
R2 = tk.Radiobutton(FrameEditSelection, text="Exclusion", value="Exclu", var=option)
R3 = tk.Radiobutton(FrameEditSelection, text="Dock", value="Dock", var=option)
R1.place(x=0,y=0)
R2.place(x=0,y=20)
R3.place(x=0,y=40)



tk.Label(MapsPage, text="House:", font=("Arial", 15), fg='green').place(x=440, y=45)
InfoHouseNrtxt = tk.Label(MapsPage, text=mymower.House, font=("Arial", 30), fg='red')
InfoHouseNrtxt.place(x=515, y=30, height=60, width=60)


ButtonCreateNewMap = tk.Button(MapsPage, text="New Map", wraplength=80, command=ButtonCreateNewMap_click)
ButtonCreateNewMap.place(x=680, y=15, height=30, width=110)

ButtonDeleteMap = tk.Button(MapsPage, text="Delete Map", wraplength=80, command=delete_map)
ButtonDeleteMap.place(x=680, y=60, height=30, width=110)

ButtonExportMap = tk.Button(MapsPage, text="EXPORT TO ROBOT", wraplength=80, command=ButtonExportMap_click)
ButtonExportMap.place(x=680, y=115, height=60, width=110)

ButtonImportMap = tk.Button(MapsPage, text="IMPORT FROM ROBOT", wraplength=80, command=import_map_from_mower)
ButtonImportMap.place(x=680, y=180, height=60, width=110)

Button_showDot_Nr = tk.Button(MapsPage, text="Test", wraplength=80, command=Button_showDot_Nr_click)
Button_showDot_Nr.place(x=680, y=200, height=60, width=110)

ButtonBackHome = tk.Button(MapsPage, image=imgBack, command=ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=119, width=115)

""" THE CAMERA PAGE ***************************************************"""


def visionThread(num):
    import os
    import cv2
    import numpy as np
    # from picamera.array import PiRGBArray
    from libcamera import controls
    from picamera2 import Picamera2
    import tensorflow as tf

    import sys
    frame_rate = 2
    count = 0
    consoleInsertText("Initialise Vision " + '\n')
    print("Initialise Vision ")
    # Set up camera constants
    # IM_WIDTH = 1280
    # IM_HEIGHT = 720
    # IM_WIDTH = 640   # Use smaller resolution for
    # IM_HEIGHT = 480  # slightly faster framerate
    IM_WIDTH = 544  # Use smaller resolution for
    IM_HEIGHT = 400  # slightly faster framerate
    #IM_WIDTH = 320   # Use smaller resolution for
    #IM_HEIGHT = 240  # slightly faster framerate

    # This is needed for working directory
    sys.path.append('..')

    # Import utilites
    from utils import label_map_util
    from utils import visualization_utils as vis_util

    # Name of the directory containing the object detection module we're using
    MODEL_NAME = 'ssdlite_mobilenet_v2_coco_2018_05_09'

    # Grab path to current working directory
    CWD_PATH = os.getcwd()

    # Path to frozen detection graph .pb file, which contains the model that is used
    # for object detection.
    PATH_TO_CKPT = os.path.join(CWD_PATH, MODEL_NAME, 'frozen_inference_graph.pb')

    # Path to label map file
    PATH_TO_LABELS = os.path.join(CWD_PATH, 'data', 'mscoco_label_map.pbtxt')

    # Path to save image
    folder_name = time.strftime("%Y%m%d%H%M%S")
    PATH_TO_SAVE_IMG = cwd + "/vision/" + folder_name + "/"
    os.mkdir(PATH_TO_SAVE_IMG)

    # Number of classes the object detector can identify
    NUM_CLASSES = 99

    ## Load the label map.
    # Label maps map indices to category names, so that when the convolution
    # network predicts `5`, we know that this corresponds to `airplane`.
    # Here we use internal utility functions, but anything that returns a
    # dictionary mapping integers to appropriate string labels would be fine
    label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
    categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES,
                                                                use_display_name=True)
    category_index = label_map_util.create_category_index(categories)

    # Load the Tensorflow model into memory.
    print("Load the Tensorflow model into memory ")
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.compat.v1.GraphDef()
        with tf.io.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')

        sess = tf.compat.v1.Session(graph=detection_graph)

    # Define input and output tensors (i.e. data) for the object detection classifier

    # Input tensor is the image
    image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

    # Output tensors are the detection boxes, scores, and classes
    # Each box represents a part of the image where a particular object was detected
    detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

    # Each score represents level of confidence for each of the objects.
    # The score is shown on the result image, together with the class label.
    detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
    detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

    # Number of objects detected
    num_detections = detection_graph.get_tensor_by_name('num_detections:0')

    # Initialize frame rate calculation
    frame_rate_calc = 1
    freq = cv2.getTickFrequency()
    font = cv2.FONT_HERSHEY_SIMPLEX

    # Initialize camera and perform object detection.
    # print(category_index)
    ##    for index in category_index :
    ##
    ##        print(category_index[index]['id'],category_index[index]['name'])

    # Initialize Picamera and grab reference to the raw capture
    print("Init Camera")
    camera = Picamera2(0)

    # normalSize = (640, 480)
    lowresSize = (IM_WIDTH, IM_HEIGHT)
    config = camera.create_preview_configuration({'format': 'RGB888', "size": lowresSize})
    camera.configure(config)
    camera.set_controls({"FrameRate": frame_rate})
    camera.start()

    while (1):
        frame = camera.capture_array('main')

        if (time.time() > mymower.visionDetectAt + 2):  # wait before next detection
            t1 = cv2.getTickCount()
            # Acquire frame and expand frame dimensions to have shape: [1, None, None, 3]
            # i.e. a single-column array, where each item in the column has the pixel RGB value
            frame.setflags(write=1)
            frame_expanded = np.expand_dims(frame, axis=0)

            # Perform the actual detection by running the model with the image as input
            if True:
                # if (time.time() > mymower.visionDetectAt + 0.5):  #wait before next detection
                # if ((myRobot.stateNames[mymower.state]=="OFF") or (myRobot.stateNames[mymower.state]=="FRWODO") or (myRobot.stateNames[mymower.state]=="PFND")):

                (boxes, scores, classes, num) = sess.run(
                    [detection_boxes, detection_scores, detection_classes, num_detections],
                    feed_dict={image_tensor: frame_expanded})

                # Draw the box and results of the detection inside frame (aka 'visulaize the results')
                vis_util.visualize_boxes_and_labels_on_image_array(
                    frame,
                    np.squeeze(boxes),
                    np.squeeze(classes).astype(np.int32),
                    np.squeeze(scores),
                    category_index,
                    use_normalized_coordinates=True,
                    line_thickness=2,
                    min_score_thresh=0.60)  # draw box only if score >60%

                t2 = cv2.getTickCount()

                cv2.putText(frame,"FPS: {0:.2f}".format(frame_rate_calc),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)
               # cv2.putText(frame, time.strftime("%x  %X"), (30, 50), font, 0.8, (255, 255, 0), 2, cv2.LINE_AA)

                # All  results have been drawn on the frame, so it's time to display it.

                img_update = ImageTk.PhotoImage(Image.fromarray(frame))
                paneeli_image.configure(image=img_update)
                paneeli_image.image = img_update
                paneeli_image.update()

                # multiple detection are possible into one frame
                # test only index 0 because it's the higher detection score
                if (scores[0][0] > visionDetectMinScore / 100):  # see config.py for global min score value

                    x = int(((boxes[0][0][1] + boxes[0][0][3]) / 2) * IM_WIDTH)
                    y = int(((boxes[0][0][0] + boxes[0][0][2]) / 2) * IM_HEIGHT)

                    lx = int(boxes[0][0][3] * IM_WIDTH - boxes[0][0][1] * IM_WIDTH)
                    ly = int(boxes[0][0][2] * IM_HEIGHT + boxes[0][0][0] * IM_HEIGHT)

                    consoleInsertText("vision detect something here : " + str(x) + "  " + str(y) + '\n')
                    consoleInsertText("size is : " + str(lx) + "  " + str(ly) + '\n')
                    mymower.visionScore = 100 * scores[0][0]
                    mymower.objectDetectedID = int(classes[0][0])
                    mymower.surfaceDetected = 100 * (lx * ly) / (IM_WIDTH * IM_HEIGHT)
                    mymower.visionDetectAt = time.time()
                    mymower.visionDetect = True
                    if (x > IM_WIDTH / 2):
                        mymower.VisionRollRight = 1
                        # consoleInsertText("Mower need to rotate RIGHT" + '\n')

                    else:
                        mymower.VisionRollRight = 0
                        # consoleInsertText("Mower need to rotate LEFT" + '\n')

                    fileName1 = PATH_TO_SAVE_IMG + str(count) + ".jpg"
                    print(fileName1)

                    cv2.imwrite(fileName1, frame)
                    count = count + 1

                    img_update = ImageTk.PhotoImage(Image.fromarray(frame))
                    paneeli_image.configure(image=img_update)
                    paneeli_image.image = img_update
                    paneeli_image.update()

            else:
                # t1 = cv2.getTickCount()
                # photo1=ImageTk.PhotoImage(image=Image.fromarray(frame))
                # print(1000*(cv2.getTickCount()-t1)/freq)
                # Video_canvas.create_image(0,0,image = photo1,anchor=tk.NW)
                img_update = ImageTk.PhotoImage(Image.fromarray(frame))
                paneeli_image.configure(image=img_update)
                paneeli_image.image = img_update
                paneeli_image.update()

            t2 = cv2.getTickCount()
            time1 = (t2 - t1) / freq
            frame_rate_calc = 1 / time1
            # print(frame_rate_calc)
            # Press 'q' to quit
            if (cv2.waitKey(1) == ord('q') or not (mymower.visionRunning)):
                # send_pfo_message('rd0','1','2','3','4','5','6',)
                break

    camera.close()
    cv2.destroyAllWindows()
    consoleInsertText("Vision Stopped " + '\n')


def BtnStreamVideoStart_click():
    consoleInsertText("Start Video streaming" + '\n')
    if CamVar1.get() == 1:
        myStreamVideo.start(1)

        # webbrowser.open("http://localhost:8000/index.html")

    else:
        myStreamVideo.start(0)


def BtnStreamVideoStop_click():
    consoleInsertText("Stop Video streaming" + '\n')
    myStreamVideo.stop()


def BtnVisionStart_click():
    if not (mymower.visionRunning):
        BtnVisionStart.place_forget()
        BtnVisionStop.place(x=650, y=200, height=25, width=80)
        print("start new thread")
        mymower.visionRunning = True
        t1 = threading.Thread(target=visionThread, args=(10,))
        t1.start()


def BtnVisionStop_click():
    if (mymower.visionRunning):
        mymower.visionRunning = False
        print("stop vision")
        BtnVisionStop.place_forget()
        BtnVisionStart.place(x=650, y=200, height=25, width=80)


StreamVideoPage = tk.Frame(fen1)
StreamVideoPage.place(x=0, y=0, height=400, width=800)
FrameStreamVideo = tk.Frame(StreamVideoPage, borderwidth="1", relief=tk.SOLID)
FrameStreamVideo.place(x=0, y=0, width=544, height=400)
Video_canvas = tk.Canvas(FrameStreamVideo, bg='white')
Video_canvas.place(x=0, y=0, width=544, height=400)

paneeli_image = tk.Label(FrameStreamVideo)  # ,image=img)
paneeli_image.grid(row=0, column=0, columnspan=3, pady=1, padx=10)

OptBtnStreamVideo1 = tk.Radiobutton(StreamVideoPage, text="320*240", relief=tk.SOLID, variable=CamVar1, value=0,
                                    anchor='nw').place(x=620, y=10, width=100, height=20)
OptBtnStreamVideo2 = tk.Radiobutton(StreamVideoPage, text="640*480", relief=tk.SOLID, variable=CamVar1, value=1,
                                    anchor='nw').place(x=620, y=30, width=100, height=20)
# tk.Label(FrameStreamVideo, text='To view the vidéo stream use a browser http://(Your PI IP Adress and):8000/index.html').place(x=10,y=180)
# tk.Label(FrameStreamVideo, text='Do not forget to activate the Camera into the RaspiConfig').place(x=10,y=200)

BtnStreamVideoStart = tk.Button(StreamVideoPage, command=BtnStreamVideoStart_click, text="Start Streaming")
BtnStreamVideoStart.place(x=650, y=100, height=25, width=110)
BtnStreamVideoStop = tk.Button(StreamVideoPage, command=BtnStreamVideoStop_click, text="Stop Streaming")
BtnStreamVideoStop.place(x=650, y=130, height=25, width=110)

BtnVisionStart = tk.Button(StreamVideoPage, command=BtnVisionStart_click, text="Start Vision")
if (useVision):
    BtnVisionStart.place(x=650, y=200, height=25, width=80)
BtnVisionStop = tk.Button(StreamVideoPage, command=BtnVisionStop_click, text="Stop Vision")

ButtonBackHome = tk.Button(StreamVideoPage, image=imgBack, command=ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=120, width=120)

""" THE TEST PAGE ***************************************************"""
def ButtonListVar_click():
    message = "AT+Q1,0"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)
    ConsolePage.tkraise()
    fen1.update()


def ButtonRebootGps_click():
    message = "AT+Y2,0"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)
    ConsolePage.tkraise()
    fen1.update()

def ButtonOdo10TurnFw_click():
    message = "AT+E1"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)
    ConsolePage.tkraise()
    fen1.update()


def ButtonFanStart_click():
    message = "AT+E4"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


def ButtonFanStop_click():
    message = "AT+E5"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)


def ButtonOdo3MlFw_click():
    message = "AT+E3"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)
    ConsolePage.tkraise()
    fen1.update()


def ButtonTestSensor_click():
    message = "AT+F"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)
    ConsolePage.tkraise()
    fen1.update()


def ButtonOdoRot360_click():
    message = "AT+E2"
    message = str(message)
    message = message + '\r'
    send_serial_message(message)
    ConsolePage.tkraise()
    fen1.update()


TestPage = tk.Frame(fen1)
TestPage.place(x=0, y=0, height=400, width=800)

ButtonOdo10TurnFw = tk.Button(TestPage)
ButtonOdo10TurnFw.place(x=30, y=15, height=25, width=200)
ButtonOdo10TurnFw.configure(command=ButtonOdo10TurnFw_click)
ButtonOdo10TurnFw.configure(text="TEST 10 Turn")

ButtonFanStart = tk.Button(TestPage)
ButtonFanStart.place(x=300, y=115, height=25, width=200)
ButtonFanStart.configure(command=ButtonFanStart_click)
ButtonFanStart.configure(text="Start Fan")
ButtonFanStop = tk.Button(TestPage)
ButtonFanStop.place(x=300, y=150, height=25, width=200)
ButtonFanStop.configure(command=ButtonFanStop_click)
ButtonFanStop.configure(text="Stop Fan")

ButtonTestSensor = tk.Button(TestPage)
ButtonTestSensor.place(x=550, y=165, height=25, width=200)
ButtonTestSensor.configure(command=ButtonTestSensor_click)
ButtonTestSensor.configure(text="Test Sensor")
##
ButtonOdo3MlFw = tk.Button(TestPage)
ButtonOdo3MlFw.place(x=30, y=115, height=25, width=200)
ButtonOdo3MlFw.configure(command=ButtonOdo3MlFw_click)
ButtonOdo3MlFw.configure(text="3 Meters Forward")

##ButtonOdoRot180= tk.Button(TestPage)
##ButtonOdoRot180.place(x=300,y=65, height=25, width=200)
##ButtonOdoRot180.configure(command = ButtonOdoRot180_click)
##ButtonOdoRot180.configure(text="Rotate 180 Degree")





ButtonOdoRot360 = tk.Button(TestPage)
ButtonOdoRot360.place(x=30, y=165, height=25, width=200)
ButtonOdoRot360.configure(command=ButtonOdoRot360_click)
ButtonOdoRot360.configure(text="Rotate 360 Degree")


ButtonRebootGps = tk.Button(TestPage)
ButtonRebootGps.place(x=30, y=195, height=25, width=120)
ButtonRebootGps.configure(command=ButtonRebootGps_click, text="GPS reboot")

ButtonListVar = tk.Button(TestPage)
ButtonListVar.configure(command = ButtonListVar_click,text="List Var")
ButtonListVar.place(x=30,y=225, height=25, width=120)

##ButtonOdoRotNonStop= tk.Button(TestPage)
##ButtonOdoRotNonStop.place(x=300,y=165, height=25, width=200)
##ButtonOdoRotNonStop.configure(command = ButtonOdoRotNonStop_click)
##ButtonOdoRotNonStop.configure(text="Rotate Non Stop 100 Turns")


def ButtonWifiOn_click():
    # returnval=messagebox.askyesno('Info',"Turn On the Wifi")
    # if returnval :
    subprocess.Popen("sudo systemctl start hostapd", shell=True)
    subprocess.Popen("sudo systemctl start dnsmasq", shell=True)
    subprocess.Popen("sudo rfkill unblock wifi", shell=True)
    consoleInsertText('WIFI is ON' + '\n')


def ButtonWifiOff_click():
    # returnval=messagebox.askyesno('Info',"Turn Off the Wifi")
    # if returnval :
    subprocess.Popen("sudo systemctl stop hostapd", shell=True)
    subprocess.Popen("sudo systemctl stop dnsmasq", shell=True)
    subprocess.Popen("sudo rfkill block wifi", shell=True)
    consoleInsertText('WIFI is OFF' + '\n')


ButtonWifiOn = tk.Button(TestPage)
ButtonWifiOn.place(x=550, y=65, height=25, width=100)
ButtonWifiOn.configure(command=ButtonWifiOn_click)
ButtonWifiOn.configure(text="Wifi On")

ButtonWifiOff = tk.Button(TestPage)
ButtonWifiOff.place(x=660, y=65, height=25, width=100)
ButtonWifiOff.configure(command=ButtonWifiOff_click)
ButtonWifiOff.configure(text="Wifi Off")


def ButtonBTOn_click():
    returnval = messagebox.askyesno('Info', "Turn On the Bluetooth")
    if returnval:
        subprocess.Popen("sudo rfkill unblock bluetooth", shell=True)


def ButtonBTOff_click():
    returnval = messagebox.askyesno('Info', "Turn Off the Bluetooth")
    if returnval:
        subprocess.Popen("sudo rfkill block bluetooth", shell=True)


ButtonBTOn = tk.Button(TestPage)
ButtonBTOn.place(x=550, y=15, height=25, width=100)
ButtonBTOn.configure(command=ButtonBTOn_click)
ButtonBTOn.configure(text="BT On")

ButtonBTOff = tk.Button(TestPage)
ButtonBTOff.place(x=660, y=15, height=25, width=100)
ButtonBTOff.configure(command=ButtonBTOff_click)
ButtonBTOff.configure(text="BT Off")

ButtonBackHome = tk.Button(TestPage, image=imgBack, command=ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=120, width=120)

""" THE TIMER PAGE ***************************************************"""


def onSliderMapChange(event):
    print("map change")
    tabTimerSelected = int(TabTimer.index("current"))
    updateTabTimerImage(tabTimerSelected)


def onSliderHouseChange(event):
    print("House change")
    tabTimerSelected = int(TabTimer.index("current"))
    updateTabTimerImage(tabTimerSelected)


def onTabTimerChange(event):
    print("tab timer change")
    tabTimerSelected = int(TabTimer.index("current"))
    updateTabTimerImage(tabTimerSelected)


def updateTabTimerImage(tabTimerSelected):
    if (SliderStartMap[tabTimerSelected].get() == 0):
        print("map 0 is not viewable")
        return
    fileName = cwd + "/House" + "{0:0>2}".format(SliderStartHouse[tabTimerSelected].get()) + \
               "/maps" + "{0:0>2}".format(SliderStartMap[tabTimerSelected].get()) + "/MAIN.npy"
    if (os.path.exists(fileName)):
        image02 = imgHouseMap[SliderStartHouse[tabTimerSelected].get()]
        ImageMap.configure(image=image02)
        ImageMap.update()
    else:
        ImageMap.configure(image=imgHouseNoMap)
        ImageMap.update()


def SliderHourStartGroup_click(var1):
    pass
    # print("heure change "+str(var1))


def checkTimerStop():
    if (mymower.timerUse):
        print("checktimer stop : ", mymower.ActualRunningTimer, " / ", mymower.lastRunningTimer)
        print("++/" + mymower.opNames + "/++")
        # stop part
        # reset timer value if end mowing at end of map or on low bat
        # warning if go to dock duration in < 1 minute ????????
        if ((mymower.ActualRunningTimer < 98) and (mymower.opNames == "Go TO DOCK")):
            print("go to dock : timer is reset")
            mymower.lastRunningTimer = mymower.ActualRunningTimer
            mymower.ActualRunningTimer = 99
            tk_infoTimer.set("")

        if (mymower.ActualRunningTimer < 98):  # one timer is active
            stopmin = 60 * myRobot.TimerstopTime_hour[mymower.ActualRunningTimer] + myRobot.TimerstopTime_minute[
                mymower.ActualRunningTimer]
            currmin = 60 * dt.datetime.today().hour + dt.datetime.today().minute
            if ((currmin >= stopmin)):
                print("Timer stop mowing cycle")
                button_home_click()
                tk_infoTimer.set("")

            else:

                print("Continue mowing for ", str(stopmin - currmin), " minutes.")



        else:
            print("no timer active and not actual mowing ")


def checkTimerStart():
    if (mymower.opNames != "DOCK"):
        return
    if (mymower.timerUse):
        print("checktimer start : ", mymower.ActualRunningTimer, " / ", mymower.lastRunningTimer)
        # start part
        info01 = "checktimer : " + str(mymower.ActualRunningTimer) + " / " + str(mymower.lastRunningTimer)
        tk_infoTimer.set(info01)

        for i in range(15):

            if ((mymower.opNames == "DOCK") & (myRobot.Timeractive[i])):

                if (myRobot.TimerdaysOfWeek[i] & (1 << dt.datetime.today().weekday())):
                    startmin = 60 * myRobot.TimerstartTime_hour[i] + myRobot.TimerstartTime_minute[i]
                    stopmin = 60 * myRobot.TimerstopTime_hour[i] + myRobot.TimerstopTime_minute[i]
                    # print(startmin)
                    currmin = 60 * dt.datetime.today().hour + dt.datetime.today().minute
                    # reset the last running timer at 0H00 each day
                    if ((currmin == 0) & (mymower.lastRunningTimer != 99)):
                        mymower.lastRunningTimer = 99

                    # print(currmin)
                    # print(stopmin)
                    if ((currmin >= startmin) & (currmin < stopmin)):
                        if (mymower.lastRunningTimer == i):
                            print("Timer ", i, " already run for this day")
                            info01 = "Timer " + str(i) + " already run for this day"
                            tk_infoTimer.set(info01)

                        else:
                            if (mymower.batSense > -1 * myRobot.batFullCurrent):
                                print("Start timer nr ", i)
                                mymower.ActualRunningTimer = i
                                mymower.TimerHouseToStart = tk_timerHouse[i].get()
                                mymower.TimerMapToStart = tk_TimerstartMap[i].get()
                                print(mymower.TimerHouseToStart, mymower.TimerMapToStart)
                                mymower.mapSelected = mymower.TimerMapToStart
                                mymower.House = mymower.TimerHouseToStart
                                textHouseNr.configure(text=mymower.House)
                                # rebuildHouseMap()
                                export_map_to_mower(mymower.House, mymower.mapSelected)
                                mymower.startAfterUploadFinish = True
                                info01 = "Timer " + str(i) + " is running"
                                tk_infoTimer.set(info01)
                            else:
                                print("Battery not enought charged ", mymower.batSense)
                                info01 = "Bat charging "
                                tk_infoTimer.set(info01)


    else:
        info01 = "Timer not active "
        tk_infoTimer.set(info01)


TabTimer = ttk.Notebook(fen1)
SheetTimer = [None] * 15
for i in range(15):
    SheetTimer[i] = tk.Frame(TabTimer, width=800, height=380)
    TabTimer.add(SheetTimer[i], text="T " + str(i))

TabTimer.place(x=0, y=0, height=430, width=800)

TabTimer.bind("<<NotebookTabChanged>>", onTabTimerChange)

tk_timerActive = []
tk_timerdaysOfWeek = []
tk_timerStartTimehour = []
tk_timerStopTimehour = []
tk_timerStartTimeMinute = []
tk_timerStopTimeMinute = []
tk_timerStartArea = []
tk_timerStartNrLane = []
tk_timerStartRollDir = []
tk_TimerstartMap = []
tk_timerStartLaneMaxlengh = []

tk_timerStartMap = []
tk_timerHouse = []
tk_Random = []
tk_ByLane = []
tk_Perimeter = []

for i in range(15):
    tk_timerActive.append(tk.IntVar())
    tk_timerdaysOfWeek.append(tk.IntVar())
    tk_timerStartTimehour.append(tk.IntVar())
    tk_timerStopTimehour.append(tk.IntVar())
    tk_timerStartTimeMinute.append(tk.IntVar())
    tk_timerStopTimeMinute.append(tk.IntVar())
    tk_timerStartNrLane.append(tk.IntVar())
    tk_timerStartArea.append(tk.IntVar())
    tk_timerStartRollDir.append(tk.IntVar())
    tk_TimerstartMap.append(tk.IntVar())
    tk_timerStartMap.append(tk.IntVar())
    tk_timerHouse.append(tk.IntVar())
    tk_timerStartLaneMaxlengh.append(tk.IntVar())

tk_timerDayVar = [[None] * 7 for i in range(15)]

ChkBtnDayGroup = [[None] * 7 for i in range(15)]
ChkBtnEnableGroup = [None] * 15
FrameStartGroup = [None] * 15
FrameStopGroup = [None] * 15
SliderHourStartGroup = [None] * 15
SliderMinuteStartGroup = [None] * 15
SliderHourStopGroup = [None] * 15
SliderMinuteStopGroup = [None] * 15
SliderStartNrLaneGroup = [None] * 15
SliderStartArea = [None] * 15
SliderStartMowPatternGroup = [None] * 15
SliderStartLaneMaxlenghGroup = [None] * 15
SliderStartHouse = [None] * 15
SliderStartMap = [None] * 15

FrameRollDir = [None] * 15
FrameMowPattern = [None] * 15
imageMapCanvas = [tk.Canvas] * 15
itemImageMapCanvas = [None] * 15
FrameLaneParameter = [None] * 15

# RdBtn_Random=[None]*15
# RdBtn_ByLane=[None]*15
# RdBtn_Perimeter=[None]*15


# create the house map image for fast preview into timer page
imgHouseNoMap = tk.PhotoImage(file=cwd + "/icons/noMap.png")
imgHouseMap = [ImageTk.PhotoImage] * 10
for i in range(10):
    fileName = cwd + "/House" + "{0:0>2}".format(i) + "/fullHouseMap.png"
    if (os.path.exists(fileName)):
        img2 = Image.open(fileName)
        img3 = img2.resize((250, 200), Image.LANCZOS)
        imgHouseMap[i] = ImageTk.PhotoImage(img3)

for i in range(15):
    ChkBtnEnableGroup[i] = tk.Checkbutton(SheetTimer[i], text="Enable this Timer", font=("Arial", 14), fg='red',
                                          variable=tk_timerActive[i], anchor='w')
    ChkBtnEnableGroup[i].place(x=20, y=0, height=25, width=380)

    FrameStartGroup[i] = tk.Frame(SheetTimer[i], borderwidth="1", relief=tk.SUNKEN)
    FrameStartGroup[i].place(x=20, y=30, height=115, width=350)
    startText = "Mower START at " + str(tk_timerStartTimehour[i].get()) + ":" + str(tk_timerStartTimeMinute[i].get())
    tk.Label(FrameStartGroup[i], text=startText, font=("Arial", 12), fg='green').place(x=0, y=10, height=15, width=300)
    SliderHourStartGroup[i] = tk.Scale(FrameStartGroup[i], command=SliderHourStartGroup_click(i), from_=0, to=23,
                                       variable=tk_timerStartTimehour[i], relief=tk.SOLID, orient='horizontal')
    SliderHourStartGroup[i].place(x=80, y=30, height=40, width=265)
    SliderMinuteStartGroup[i] = tk.Scale(FrameStartGroup[i], from_=0, to=59, variable=tk_timerStartTimeMinute[i],
                                         relief=tk.SOLID, orient='horizontal')
    SliderMinuteStartGroup[i].place(x=80, y=70, height=40, width=265)
    tk.Label(FrameStartGroup[i], text='Hour :', font=("Arial", 14), fg='green').place(x=10, y=30, height=40, width=70)
    tk.Label(FrameStartGroup[i], text='Minute :', font=("Arial", 14), fg='green').place(x=10, y=70, height=40, width=70)

    FrameStopGroup[i] = tk.Frame(SheetTimer[i], borderwidth="1", relief=tk.SUNKEN)
    FrameStopGroup[i].place(x=380, y=30, height=115, width=350)
    stopText = "Mower STOP at " + str(tk_timerStopTimehour[i].get()) + ":" + str(tk_timerStopTimeMinute[i].get())
    tk.Label(FrameStopGroup[i], text=stopText, font=("Arial", 12), fg='green').place(x=0, y=10, height=15, width=300)
    SliderHourStopGroup[i] = tk.Scale(FrameStopGroup[i], from_=0, to=23, variable=tk_timerStopTimehour[i],
                                      relief=tk.SOLID, orient='horizontal')
    SliderHourStopGroup[i].place(x=80, y=30, height=40, width=265)
    SliderMinuteStopGroup[i] = tk.Scale(FrameStopGroup[i], from_=0, to=59, variable=tk_timerStopTimeMinute[i],
                                        relief=tk.SOLID, orient='horizontal')
    SliderMinuteStopGroup[i].place(x=80, y=70, height=40, width=265)
    tk.Label(FrameStopGroup[i], text='Hour :', font=("Arial", 14), fg='green').place(x=10, y=30, height=40, width=70)
    tk.Label(FrameStopGroup[i], text='Minute :', font=("Arial", 14), fg='green').place(x=10, y=70, height=40, width=70)

    tk.Label(SheetTimer[i], text="House :", fg='green').place(x=5, y=180, height=20, width=80)
    SliderStartHouse[i] = tk.Scale(SheetTimer[i], from_=0, to=10, font=("Arial", 8), variable=tk_timerHouse[i],
                                   relief=tk.SOLID, orient='horizontal')
    SliderStartHouse[i].place(x=10, y=200, height=35, width=180)
    SliderStartHouse[i].bind("<ButtonRelease-1>", onSliderHouseChange)
    tk.Label(SheetTimer[i], text="Map :", fg='green').place(x=195, y=180, height=20, width=100)
    SliderStartMap[i] = tk.Scale(SheetTimer[i], from_=0, to=10, font=("Arial", 8), variable=tk_TimerstartMap[i],
                                 relief=tk.SOLID, orient='horizontal')
    SliderStartMap[i].place(x=200, y=200, height=35, width=200)
    SliderStartMap[i].bind("<ButtonRelease-1>", onSliderMapChange)

    ##    FrameMowPattern[i] = tk.Frame(SheetTimer[i],borderwidth="1",relief=tk.SUNKEN)
    ##    FrameMowPattern[i].place(x=410, y=180, height=200, width=250)
    ##    imageMapCanvas[i] = tk.Canvas(FrameMowPattern[i])
    ##
    ##    imageMapCanvas[i].create_image(0,0, image = image01, anchor = "nw")

    ##    itemImageMapCanvas[i]=imageMapCanvas[i].create_image(0,0, image = image02, anchor = "nw")
    ##    imageMapCanvas[i].pack(expand = tk.YES, fill = tk.BOTH)
    # itemImageMapCanvas[i].pack(expand = tk.YES, fill = tk.BOTH)

    # tk.Label(FrameMowPattern[i],text="MOW PATTERN :",fg='green').pack(side='top',anchor='w')
    # RdBtn_Random[i]=tk.Radiobutton(FrameMowPattern[i], text="Random", variable=tk_TimerstartMap[i], value=0).pack(side='top',anchor='w')
    # RdBtn_ByLane[i]=tk.Radiobutton(FrameMowPattern[i], text="By Lane", variable=tk_TimerstartMap[i], value=1).pack(side='top',anchor='w')
    # RdBtn_Perimeter[i]=tk.Radiobutton(FrameMowPattern[i], text="Perimeter", variable=tk_TimerstartMap[i], value=2).pack(side='top',anchor='w')

    # FrameLaneParameter[i] = tk.Frame(SheetTimer[i],borderwidth="1",relief=tk.SUNKEN)
    # FrameLaneParameter[i].place(x=10, y=240, height=140, width=390)
    # tk.Label(FrameLaneParameter[i],text="Maximum Lane Lenght :",fg='green').place(x=10,y=0, height=20, width=360)
    # SliderStartLaneMaxlenghGroup[i]= tk.Scale(FrameLaneParameter[i],from_=1, to=50,font=("Arial", 8),variable=tk_timerStartLaneMaxlengh[i],relief=tk.SOLID,orient='horizontal')
    # SliderStartLaneMaxlenghGroup[i].place(x=10,y=20, height=35, width=360)

    # tk.Label(FrameLaneParameter[i], text='Roll Dir',font=("Arial", 12), fg='green').place(x=10,y=60, height=15, width=80)
    # RdBtn_Right=tk.Radiobutton(FrameLaneParameter[i], text="Right",variable=tk_timerStartRollDir[i], value=0).place(x=10,y=75, height=20, width=80)
    # RdBtn_Left=tk.Radiobutton(FrameLaneParameter[i], text="Left ",variable=tk_timerStartRollDir[i], value=1).place(x=10,y=95, height=20, width=80)

    ##    tk.Label(FrameLaneParameter[i],text="START Lane",font=("Arial", 12), fg='green').place(x=150,y=60, height=15, width=90)
    ##    SliderStartNrLaneGroup[i]= tk.Scale(FrameLaneParameter[i],from_=1, to=3,variable=tk_timerStartNrLane[i],relief=tk.SOLID,orient='horizontal').place(x=160,y=75, height=40, width=70)
    ##
    ##    tk.Label(FrameLaneParameter[i],text="START Area",font=("Arial", 12), fg='green').place(x=260,y=60, height=15, width=90)
    ##    SliderStartArea[i]= tk.Scale(FrameLaneParameter[i],from_=1, to=3,variable=tk_timerStartArea[i],relief=tk.SOLID,orient='horizontal').place(x=260,y=75, height=40, width=70)

    for j in range(7):
        tk_timerDayVar[i][j] = tk.BooleanVar()
        ChkBtnDayGroup[i][j] = tk.Checkbutton(SheetTimer[i], text=days_list[j], variable=tk_timerDayVar[i][j],
                                              relief=tk.GROOVE, borderwidth="1", anchor='w')
        ChkBtnDayGroup[i][j].place(x=110 * j + 10, y=150, height=25, width=120)

    ##


##
##    for i in range(15):
##        myRobot.Timeractive[i]=tk_timerActive[i].get()
##        myRobot.TimerstartTime_hour[i]=tk_timerStartTimehour[i].get()
##        myRobot.TimerstartTime_minute[i]=tk_timerStartTimeMinute[i].get()
##        myRobot.TimerstopTime_hour[i]=tk_timerStopTimehour[i].get()
##        myRobot.TimerstopTime_minute[i]=tk_timerStopTimeMinute[i].get()
##        myRobot.TimerstartHouse[i]=tk_timerHouse[i].get()
##        myRobot.TimerstartMap[i]=tk_TimerStartMap[i].get()
##        myRobot.TimerstartNrLane[i]=tk_timerStartNrLane[i].get()
##        myRobot.TimerstartRollDir[i]=tk_timerStartRollDir[i].get()
##        myRobot.TimerstartLaneMaxlengh[i]=tk_timerStartLaneMaxlengh[i].get()
##        #the 7 days of the week as byte
##        myRobot.TimerdaysOfWeek[i]=1*int(tk_timerDayVar[i][0].get())+2*int(tk_timerDayVar[i][1].get())+4*int(tk_timerDayVar[i][2].get())+\
##                  8*int(tk_timerDayVar[i][3].get())+16*int(tk_timerDayVar[i][4].get())+32*int(tk_timerDayVar[i][5].get())+\
##                  64*int(tk_timerDayVar[i][6].get())


##
##        Send_reqSetting_message('Timer','w',''+str(i)+'',''+str(myRobot.Timeractive[i])+\
##                                '',''+str(myRobot.TimerstartTime_hour[i])+\
##                                '',''+str(myRobot.TimerstartTime_minute[i])+\
##                                '',''+str(myRobot.TimerstopTime_hour[i])+\
##                                '',''+str(myRobot.TimerstopTime_minute[i])+\
##                                '',''+str(myRobot.TimerstartHouse[i])+\
##                                '',''+str(myRobot.TimerstartMap[i])+\
##                                '',''+str(myRobot.TimerstartNrLane[i])+\
##                                '',''+str(myRobot.TimerstartRollDir[i])+\
##                                '',''+str(myRobot.TimerstartLaneMaxlengh[i]),)
##
##

##        Send_reqSetting_message('Timer1','w',''+str(i)+'',''+str(myRobot.TimerstartArea[i])+\
##                                '',''+str(myRobot.TimerdaysOfWeek[i])+\
##                                '','0','0','0','0','0','0','0','0',)
##
##


def load_TimerList():
    # Read the file and create the list
    with open("timer_list.bin", "rb") as fp:
        timer_list = pickle.load(fp)
        # print("Timer file loaded")
        # print(timer_list)

        for i in range(15):
            myRobot.Timeractive[i] = timer_list[i][0]
            tk_timerActive[i].set(myRobot.Timeractive[i])
            myRobot.TimerstartTime_hour[i] = timer_list[i][1]
            tk_timerStartTimehour[i].set(myRobot.TimerstartTime_hour[i])
            myRobot.TimerstartTime_minute[i] = timer_list[i][2]
            tk_timerStartTimeMinute[i].set(myRobot.TimerstartTime_minute[i])
            myRobot.TimerstopTime_hour[i] = timer_list[i][3]
            tk_timerStopTimehour[i].set(myRobot.TimerstopTime_hour[i])
            myRobot.TimerstopTime_minute[i] = timer_list[i][4]
            tk_timerStopTimeMinute[i].set(myRobot.TimerstopTime_minute[i])
            myRobot.TimerstartHouse[i] = timer_list[i][5]
            tk_timerHouse[i].set(myRobot.TimerstartHouse[i])
            myRobot.TimerstartMap[i] = timer_list[i][6]
            tk_TimerstartMap[i].set(myRobot.TimerstartMap[i])
            myRobot.TimerstartNrLane[i] = timer_list[i][7]
            tk_timerStartNrLane[i].set(myRobot.TimerstartNrLane[i])
            myRobot.TimerstartRollDir[i] = timer_list[i][8]
            tk_timerStartRollDir[i].set(myRobot.TimerstartRollDir[i])
            myRobot.TimerstartLaneMaxlengh[i] = timer_list[i][9]
            tk_timerStartLaneMaxlengh[i].set(myRobot.TimerstartLaneMaxlengh[i])
            # the 7 days of the week as byte
            tk_timerDayVar[i][0].set(timer_list[i][10])
            tk_timerDayVar[i][1].set(timer_list[i][11])
            tk_timerDayVar[i][2].set(timer_list[i][12])
            tk_timerDayVar[i][3].set(timer_list[i][13])
            tk_timerDayVar[i][4].set(timer_list[i][14])
            tk_timerDayVar[i][5].set(timer_list[i][15])
            tk_timerDayVar[i][6].set(timer_list[i][16])

            myRobot.TimerdaysOfWeek[i] = 1 * int(tk_timerDayVar[i][0].get()) + 2 * int(
                tk_timerDayVar[i][1].get()) + 4 * int(tk_timerDayVar[i][2].get()) + \
                                         8 * int(tk_timerDayVar[i][3].get()) + 16 * int(
                tk_timerDayVar[i][4].get()) + 32 * int(tk_timerDayVar[i][5].get()) + \
                                         64 * int(tk_timerDayVar[i][6].get())


def save_TimerList():
    # timer_list=[[0,10,10,11,11,0,0,0,0,0,1,1,1,1,1,1,1],[0,10,10,11,11,0,0,0,0,0,0,1,0,1,0,1,0,1],[0,10,10,11,11,0,0,0,0,0,0,1,0,1,0,1,0,1],[0,10,10,11,11,0,0,0,0,0,0,1,0,1,0,1,0,1],[0,10,10,11,11,0,0,0,0,0,0,1,0,1,0,1,0,1]]
    timer_list = []
    # refresh variable to page change
    for i in range(15):
        myRobot.Timeractive[i] = tk_timerActive[i].get()
        myRobot.TimerstartTime_hour[i] = tk_timerStartTimehour[i].get()
        myRobot.TimerstartTime_minute[i] = tk_timerStartTimeMinute[i].get()
        myRobot.TimerstopTime_hour[i] = tk_timerStopTimehour[i].get()
        myRobot.TimerstopTime_minute[i] = tk_timerStopTimeMinute[i].get()
        myRobot.TimerstartHouse[i] = tk_timerHouse[i].get()
        myRobot.TimerstartMap[i] = tk_TimerstartMap[i].get()
        myRobot.TimerstartNrLane[i] = tk_timerStartNrLane[i].get()
        myRobot.TimerstartRollDir[i] = tk_timerStartRollDir[i].get()
        myRobot.TimerstartLaneMaxlengh[i] = tk_timerStartLaneMaxlengh[i].get()
        # the 7 days of the week as byte
        myRobot.TimerdaysOfWeek[i] = 1 * int(tk_timerDayVar[i][0].get()) + 2 * int(
            tk_timerDayVar[i][1].get()) + 4 * int(tk_timerDayVar[i][2].get()) + \
                                     8 * int(tk_timerDayVar[i][3].get()) + 16 * int(
            tk_timerDayVar[i][4].get()) + 32 * int(tk_timerDayVar[i][5].get()) + \
                                     64 * int(tk_timerDayVar[i][6].get())

    maTimerLigne = []
    for i in range(15):
        maTimerLigne.append(myRobot.Timeractive[i])
        maTimerLigne.append(myRobot.TimerstartTime_hour[i])
        maTimerLigne.append(myRobot.TimerstartTime_minute[i])
        maTimerLigne.append(myRobot.TimerstopTime_hour[i])
        maTimerLigne.append(myRobot.TimerstopTime_minute[i])
        maTimerLigne.append(myRobot.TimerstartHouse[i])
        maTimerLigne.append(myRobot.TimerstartMap[i])
        maTimerLigne.append(myRobot.TimerstartNrLane[i])
        maTimerLigne.append(myRobot.TimerstartRollDir[i])
        maTimerLigne.append(myRobot.TimerstartLaneMaxlengh[i])
        for v in range(7):
            maTimerLigne.append(tk_timerDayVar[i][v].get())
        # print(maTimerLigne)
        timer_list.append(maTimerLigne)
        maTimerLigne = []

    with open("timer_list.bin", "wb") as fp:
        pickle.dump(timer_list, fp)
        # print("Timer file saved")


ButtonLoadTimer = tk.Button(TabTimer)
ButtonLoadTimer.place(x=680, y=225, height=25, width=150)
ButtonLoadTimer.configure(command=load_TimerList)
ButtonLoadTimer.configure(text="Load Timer")

ButtonSaveTimer = tk.Button(TabTimer)
ButtonSaveTimer.place(x=680, y=265, height=25, width=150)
ButtonSaveTimer.configure(command=save_TimerList)
ButtonSaveTimer.configure(text="Save Timer")

ImageMap = tk.Label(TabTimer, font=("Arial", 20), fg='red')
ImageMap.place(x=410, y=210, height=200, width=250)

# FrameLaneParameter[i] = tk.Frame(SheetTimer[i],borderwidth="1",relief=tk.SUNKEN)
# FrameLaneParameter[i].place(x=10, y=240, height=140, width=390)
# ButtonCheckTimer = tk.Button(TabTimer)
# ButtonCheckTimer.place(x=300,y=400, height=25, width=150)
# ButtonCheckTimer.configure(command = checkTimerStart,text="checkTimerStart")

ButtonBackHome = tk.Button(TabTimer, image=imgBack, command=ButtonBackToMain_click)
# ButtonBackHome = tk.Button(TabTimer, image=imgHouseMap[0], command = ButtonBackToMain_click)


ButtonBackHome.place(x=680, y=310, height=120, width=120)

""" THE MAIN PAGE ***************************************************"""


def ButtonPowerOff_click():
    returnval = messagebox.askyesno('Info', "Are you sure you want to shutdown all the PCB ?")
    if returnval:
        message = "AT+Y3"
        message = str(message)
        message = message + '\r'
        send_serial_message(message)


MainPage = tk.Frame(fen1)
MainPage.place(x=0, y=0, height=480, width=800)

ButtonAuto = tk.Button(MainPage, image=imgAuto, command=ButtonAuto_click)
ButtonAuto.place(x=10, y=10, height=130, width=100)

ButtonManual = tk.Button(MainPage, image=imgManual)
ButtonManual.place(x=145, y=10, height=130, width=100)
ButtonManual.configure(command=ButtonManual_click)

ButtonSetting = tk.Button(MainPage, image=imgSetting)
ButtonSetting.place(x=280, y=10, height=130, width=100)
ButtonSetting.configure(command=ButtonSetting_click)

ButtonConsole = tk.Button(MainPage, image=imgConsole)
ButtonConsole.place(x=415, y=10, height=130, width=100)
ButtonConsole.configure(command=ButtonConsole_click)

ButtonTest = tk.Button(MainPage, image=imgTest)
ButtonTest.place(x=550, y=10, height=130, width=100)
ButtonTest.configure(command=ButtonTest_click)

# ButtonPlot = tk.Button(MainPage, image=imgPlot, command = ButtonPlot_click)
# ButtonPlot.place(x=10,y=145,width=100, height=130)

ButtonSchedule = tk.Button(MainPage, image=imgSchedule, command=ButtonSchedule_click)
ButtonSchedule.place(x=145, y=145, width=100, height=130)

ButtonCamera = tk.Button(MainPage, image=imgCamera, command=ButtonCamera_click)
ButtonCamera.place(x=280, y=145, width=100, height=130)

# ButtonGps = tk.Button(MainPage, image=imgGps, command = ButtonGps_click)
# ButtonGps.place(x=415,y=145,width=100, height=130)

ButtonMaps = tk.Button(MainPage, image=imgMaps, command=ButtonMaps_click)
ButtonMaps.place(x=415, y=145, width=100, height=130)

ButtonPowerOff = tk.Button(MainPage, image=imgPowerOff, command=ButtonPowerOff_click)
ButtonPowerOff.place(x=685, y=280, width=100, height=120)

Buttonimgardu = tk.Button(MainPage, image=imgArdumower, command=ButtonInfo_click)
Buttonimgardu.place(x=10, y=280, height=120, width=650)

Datetext = tk.Label(MainPage, text='', textvariable=tk_date_Now, font=("Arial", 20), fg='red')
Datetext.place(x=10, y=400, height=25, width=240)

Statustext = tk.Label(MainPage, text='', textvariable=tk_MainStatusLine, font=("Arial", 20), fg='red')
Statustext.place(x=240, y=400, height=25, width=400)

################## DESACTIVATE THE BT TO SAVE BATTERY
# subprocess.call(["rfkill","block","bluetooth"])
#subprocess.Popen("sudo rfkill block bluetooth", shell=True)
#subprocess.Popen("sudo iw wlan0 set power_save off", shell=True)


# subprocess.Popen("sudo rfkill block wifi", shell=True)
# sudo rfkill block wifi
# sudo rfkill unblock wifi
# sudo rfkill block bluetooth
# sudo rfkill unblock bluetooth

# use to drive the mower using the keyboard in manual
def check_keyboard(e):
    if (page_list[mymower.focusOnPage] == "MANUAL") & (ManualKeyboardUse.get() == 1):
        if (e.char == "m"):
            buttonBlade_start_click()
        if (e.char == 'q'):
            buttonBlade_stop_click()


def kbd_spaceKey(e):
    if (page_list[mymower.focusOnPage] == "MANUAL") & (ManualKeyboardUse.get() == 1):
        button_stop_all_click()


def kbd_leftKey(e):
    if (page_list[mymower.focusOnPage] == "MANUAL") & (ManualKeyboardUse.get() == 1):
        ButtonLeft_click()


def kbd_rightKey(e):
    if (page_list[mymower.focusOnPage] == "MANUAL") & (ManualKeyboardUse.get() == 1):
        ButtonRight_click()


def kbd_upKey(e):
    if (page_list[mymower.focusOnPage] == "MANUAL") & (ManualKeyboardUse.get() == 1):
        ButtonForward_click()


def kbd_downKey(e):
    if (page_list[mymower.focusOnPage] == "MANUAL") & (ManualKeyboardUse.get() == 1):
        ButtonStop_click()


fen1.bind("<KeyPress>", check_keyboard)
fen1.bind('<Left>', kbd_leftKey)
fen1.bind('<Right>', kbd_rightKey)
fen1.bind('<Up>', kbd_upKey)
fen1.bind('<Down>', kbd_downKey)
fen1.bind('<space>', kbd_spaceKey)

##mymower.focusOnPage=1
##AutoPage.tkraise()
# fen1.wm_attributes("-transparentcolor", 'grey')


# on startup PI update Date time from Internet
# subprocess.Popen("sudo systemctl stop ntp.service", shell=True)
# subprocess.Popen("sudo systemctl disable ntp.service", shell=True)
# time.sleep(10)


##
if (streamVideoOnPower):
    BtnStreamVideoStart_click()

ButtonReadSettingFromFile_click()
load_TimerList()

checkSerial()
fen1.mainloop()




            
            
        
