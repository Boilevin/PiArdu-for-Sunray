###!/usr/bin/env python3

#axes.add_patch(Polygon([(0, 0), (1, 0.2), (0.3, 0.4), (0.2, 1)],
#                       closed=True, facecolor='red'))

PiVersion="76"
from pathlib import Path
import traceback
import sys
import serial
import time
import datetime as dt

import csv
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
#from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from shapely import geometry
from shapely import MultiPolygon, Polygon

import numpy as np
import subprocess 
import pickle

import os
from tkinter import ttk

from tkinter import messagebox
from tkinter import filedialog
import tkinter as tk
import math

from gpiozero import CPUTemperature

from config import cwd
from config import myOS
from config import GpsConnectedOnPi
#from config import NanoConnectedOnPi
from config import DueConnectedOnPi
from config import GpsIsM6n
from config import AutoRecordBatCharging
#from config import useDebugConsole
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

from config import useVision
from config import visionDetectMinScore

from config import max_map_inUse

#Show video in tkinter canvas but very slow
#if(useVision):
from PIL import Image,ImageTk

import threading




#matplotlib navigation vertical bar 
class VerticalNavigationToolbar2Tk(NavigationToolbar2Tk):
   def __init__(self, canvas, window):
      super().__init__(canvas, window, pack_toolbar=False)

   # override _Button() to re-pack the toolbar button in vertical direction
   def _Button(self, text, image_file, toggle, command):
      b = super()._Button(text, image_file, toggle, command)
      b.pack(side=tk.TOP) # re-pack button in vertical direction
      return b

   # override _Spacer() to create vertical separator
   def _Spacer(self):
      s = tk.Frame(self, width=26, relief=tk.RIDGE, bg="DarkGray", padx=2)
      s.pack(side=tk.TOP, pady=5) # pack in vertical direction
      return s

   # disable showing mouse position in toolbar
   def set_message(self, s):
      pass
    
   


if myOS == "Linux":
    from Ps4remote import PS4Controller


"""      bb file     """
from robot import *

sys.path.insert(0, "/home/pi/Documents/PiArdumower") #add to avoid KST plot error on path





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
    message="AT+U1"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)   
    DueConnectedOnPi=False
    Due_Serial.close()
    subprocess.Popen("/home/pi/Documents/./teensy &", shell=True)
    time.sleep(30)
    os.remove("/home/pi/Documents/PiArdumower/Due_firmware/firmware.hex")
    DueConnectedOnPi=True
    Due_Serial.open()

   


#################################### CAMERA MANAGEMENT ###############################################
class streamVideo_class(object):
    """class use to start and stop the video stream"""
    def __init__(self):
        self.streamVideo = None

    def start(self,resolution):
        self.stop()
        
        if resolution==0:
            self.streamVideo=subprocess.Popen(["/home/pi/Documents/PiArdumower/streamVideo320.py","shell=True","stdout=subprocess.PIPE"])
        if resolution==1:
            self.streamVideo=subprocess.Popen(["/home/pi/Documents/PiArdumower/streamVideo640.py","shell=True","stdout=subprocess.PIPE"])
        
    def stop(self):
        if self.streamVideo:
            self.streamVideo.kill()
            self.streamVideo.wait()
            self.streamVideo = None
            
myStreamVideo=streamVideo_class()




#################################### KST PLOT MANAGEMENT ###############################################
    
class PlotterKst_class(object):
    """class use to start and stop the kst plotting prog"""
    def __init__(self):
        self.PlotterKst = None

    def start(self,fileNameKst):
        self.stop()
        self.PlotterKst=subprocess.Popen(["kst2",fileNameKst,"show=maximize","shell=True","stdout=subprocess.PIPE"])
        
    def stop(self):
        if self.PlotterKst:
            self.PlotterKst.kill()
            self.PlotterKst.wait()
            self.PlotterKst = None

         

#################################### VARIABLE INITIALISATION ###############################################
   




direction_list=['LEFT','RIGHT']
days_list=['MONDAY','TUESDAY','WEDNESDAY','THURSDAY','FRIDAY','SATURDAY','SUNDAY']
page_list=['MAIN','AUTO','MANUAL','SETTING','CONSOLE','TEST','PLOT','TIMER','VIDEO','GPS','MAPS']


motPlotterKst = PlotterKst_class()
mowPlotterKst = PlotterKst_class()
periPlotterKst = PlotterKst_class()
batPlotterKst = PlotterKst_class()
ImuPlotterKst = PlotterKst_class()


firstplotMotx=0
firstplotMowx=0
firstplotBatx=0
firstplotPerx=0
firstplotImux=0

actualRep=os.getcwd()
dateNow=time.strftime('%d/%m/%y %H:%M:%S',time.localtime())

fen1 =tk.Tk()
"""Variable for check button """
MotVar1 = tk.IntVar()
MotVar2 = tk.IntVar()
PeriVar1 = tk.IntVar()
PeriVar2 = tk.IntVar()
PeriVar3 = tk.IntVar()
PeriVar4 = tk.IntVar()
ImuVar1=tk.IntVar()
SonVar1=tk.IntVar()
SonVar2=tk.IntVar()
SonVar3=tk.IntVar()
SonVar4=tk.IntVar()

BatVar1=tk.IntVar()
MowVar1=tk.IntVar()
PlotVar1=tk.IntVar()
CamVar1=tk.IntVar()


tk_date_Now=tk.StringVar()
tk_time_Now=tk.StringVar()
tk_MainStatusLine=tk.StringVar()
tk_date_hour=tk.IntVar()
tk_date_minute=tk.IntVar()
tk_date_dayOfWeek=tk.IntVar()
tk_date_day=tk.IntVar()
tk_date_month=tk.IntVar()
tk_date_year=tk.IntVar()
tk_mowingPattern=tk.IntVar()

"""variable use into Auto Menu"""
tk_batteryVoltage=tk.DoubleVar()
tk_ImuYaw=tk.DoubleVar()
tk_batSense=tk.DoubleVar()
tk_ImuRoll=tk.DoubleVar()
tk_PiTemp=tk.DoubleVar()
tk_Dht22Humid=tk.DoubleVar()
tk_loopsPerSecond=tk.DoubleVar()
tk_areaToGo=tk.IntVar()
tk_areaToGo.set(1)
tk_ResumeMowing=tk.IntVar()
tk_HouseView=tk.IntVar()
tk_GpsSolution=tk.StringVar()
tk_gpsnumSV=tk.StringVar()
tk_mowPointIdx=tk.StringVar()
tk_labelStopIdx=tk.IntVar()
tk_labelStartIdx=tk.IntVar()
tk_AutoInfoMap=tk.StringVar()


"""variable use into refreh plot"""
tk_millis=tk.IntVar()
tk_motorLeftPower=tk.DoubleVar()
tk_motorRightPower=tk.DoubleVar()
tk_motorLeftPWMCurr=tk.IntVar()
tk_motorRightPWMCurr=tk.IntVar()
tk_motorMowPower=tk.DoubleVar()
tk_motorMowPWMCurr=tk.IntVar()
#tk_batteryVoltage=tk.IntVar()
tk_chgVoltage=tk.DoubleVar()
tk_chgSense=tk.DoubleVar()
tk_perimeterMag=tk.IntVar()
tk_perimeterMagRight=tk.IntVar()
tk_gyroYaw=tk.DoubleVar()
tk_compassYaw=tk.DoubleVar()

ManualKeyboardUse=tk.IntVar()
MainperimeterUse= tk.IntVar()
MainimuUse= tk.IntVar()
MaingpsUse= tk.IntVar()
MainbluetoothUse= tk.IntVar()
MainbumperUse= tk.IntVar()
MainsonarUse= tk.IntVar()
MainDHT22Use= tk.IntVar()
MainlawnSensorUse= tk.IntVar()
tk_MaintimerUse= tk.IntVar()
tk_infoTimer=tk.StringVar()
tk_MainrainUse= tk.IntVar()
tk_useDebugConsole = tk.IntVar()

MaintiltUse= tk.IntVar()

tk_rollDir=tk.StringVar()
tk_laneInUse= tk.IntVar()
tk_YawActual=tk.DoubleVar()
tk_YawCible=tk.DoubleVar()

firstFixFlag=False
firstFixDate=0


fen1.title('SUNRAY')
fen1.geometry("800x480+0+0")

class  datetime:
    def __init__(self):       
        datetime.hour = 12
        datetime.minute = 0
        datetime.dayOfWeek = 0
        datetime.day = 1
        datetime.month = 1
        datetime.year = 2017

class mower:
    #char* mowPatternNames[] = {"RAND", "LANE",  "WIRE"};
    def __init__(self):
        self.millis=0
        self.status=0
        mower.state=0
        mower.odox=0
        mower.odoy=0
        mower.prevYaw=0
        mower.batteryVoltage=0.00
        mower.chgVoltage=0.00
        mower.batSense=0.00

        


        
        mower.yaw=0
        mower.pitch=0
        mower.roll=0
        mower.version='Unknow'
        mower.statsOverride=0
        mower.statsMowTimeMinutesTrip=0
        mower.statsMowTimeHoursTotal=0
        mower.statsBatteryChargingCounterTotal=0
        mower.statsBatteryChargingCapacityTrip=0
        mower.statsBatteryChargingCapacityTotal=0
        mower.statsBatteryChargingCapacityAverage=0
        #mower.motorLeftSenseCurrent=0
        #mower.motorRightSenseCurrent=0
        mower.motorLeftPower=0
        mower.motorRightPower=0
        mower.motorLeftPWMCurr=0
        mower.motorRightPWMCurr=0
        mower.motorMowPower=0
        mower.motorMowPWMCurr=0
        mower.mowPatternCurr=0
        mower.Dht22Temp=0
        mower.Dht22Humid=0
        mower.rollDir=0
        mower.laneInUse= 0
        mower.YawActual=0
        mower.YawCible=0
        mower.loopsPerSecond=0
        mower.useJoystick=False
        mower.laserSensor1=9000
        mower.laserSensor2=9000
        mower.laserSensor3=9000
        mower.laserSensor4=9000
        mower.laserSensor5=9000
        mower.laserSensor6=9000
        mower.rainDetect=False
        mower.areaInMowing=1
        mower.areaToGo=1
        
        mower.sigAreaOff=True
        
        mower.timeToStartAreaSignal=0
        mower.focusOnPage=0
        mower.dueSerialReceived=''
        mower.autoRecordBatChargeOn=False

        mower.mqtt_message_id=0
        mower.callback_id=0
        mower.timeToSendMqttIdle=time.time()+20
        mower.timeToReconnectMqtt=time.time()+40
        mower.oneMinuteTrigger=time.time()+10
        mower.FiveSecondTrigger=time.time()+5
        mower.OneSecondTrigger=time.time()+1
        
        mower.cpuFan=False
        mower.lastMqttBatteryValue=0
        
        mower.visionDetect=False
        mower.visionDetectAt=time.time()-3
        mower.visionRollRight=1
        mower.visionRunning=False
        mower.surfaceDetected=0
        mower.objectDetectedID=0
        mower.userOut2=False
        mower.userOut3=False
        mower.visionScore=0

        self.House="00"
        
        self.mowPointsIdx=0
        self.startMowPointIdx=0
        self.stopMowPointIdx=99999

        self.statex=0.00
        self.statey=0.00
        self.laststatex=0.00
        self.laststatey=0.00
        self.stateDelta=0
        self.gpsSolution=0
        self.opNames="Idle"
        self.stateSensor=0
        self.targetPointx=0.00
        self.targetPointy=0.00
        self.lasttargetPointx=0.00
        self.lasttargetPointy=0.00
        
        self.gpsAccuracy=0
        self.gpsnumSV=0
        self.gpsnumSVdgps=0
        self.mapCRC=0 #sum of point coordonnees to be sure map is not false
        self.plotMapCRC=0 #when import a map check if it's egal to mapcrc,the one send each second
        self.lateralerror=0

        self.perimeterPointsCount=0
        self.exclusionPointsCount=0
        self.dockPointsCount=0
        self.mowPointsCount=0
        self.freePointsCount=0
        self.nbTotalExclusion=0
        self.fileMapCRC=0

        self.startPiArduTime=time.time()
        self.map=[0]*30  #need bigger ??
        self.mapSelected = 1
        self.finishedUploadingMap=False
        self.nbTotalExclusion=3
        self.ActiveMapX = []
        self.ActiveMapY = []
        self.polygon = [None]*10
        self.mapNrList = []
        self.perimeter=np.zeros([],[])
        self.dockPts=np.zeros([],[])
        self.mowPts=np.zeros([],[])
        self.isSendindMap=False
        self.mapCrcRoundingRange=50 #map crc is not exactly the same between pi and mower 
        self.full_house=0
        self.actualMowingArea=0
        self.totalMowingArea=0

        self.ActualRunningTimer = 99
        self.lastRunningTimer=99
        self.TimerHouseToStart = 0
        self.TimerMapToStart = 0
        self.startAfterUploadFinish = False

        self.useDebugConsole = False
        self.timerUse = False
        self.rainUse = False
        
        
        
        
    
mymower=mower()

myRobot=robot()
myDate=datetime()
cpu = CPUTemperature()

if myOS == "Linux":
    myps4 = PS4Controller()

def consoleInsertText(texte):
    
        txtConsoleRecu.insert('1.0',' ' + texte)
        txtConsoleRecu.insert('1.0', time.strftime('%H:%M:%S',time.localtime()))   
       
    
    

#################################### MAIN LOOP ###############################################

def checkSerial():  #the main loop is here
##    if (mymower.isSendindMap) :
##        return
    try:
        global DueConnectedOnPi
        if (DueConnectedOnPi and Due_Serial.inWaiting() != 0) :
            mymower.dueSerialReceived=Due_Serial.readline()
            if str(mymower.dueSerialReceived)!="b''":
                mymower.dueSerialReceived = mymower.dueSerialReceived.decode('utf-8', errors='ignore')
                if mymower.dueSerialReceived[:1] != '$' : #it is console message because the first digit is not $
                    if(len(mymower.dueSerialReceived))>2:
                        #consoleInsertText(mymower.dueSerialReceived)
                        decode_AT_message(mymower.dueSerialReceived)
                else :
                    consoleInsertText("recu sentense starting by $ "+ '\n')
                    consoleInsertText(mymower.dueSerialReceived)
                    

    except Exception:
        ConsolePage.tkraise()
        DueConnectedOnPi=False
        exc_type, exc_value, exc_traceback = sys.exc_info()
        #consoleInsertText("*** print_tb:")
        traceback.print_tb(exc_traceback, limit=1, file=sys.stdout)
        traceback.print_exception(exc_type, exc_value, exc_traceback,limit=2, file=sys.stdout)
        consoleInsertText(repr(traceback.print_exc()))
        formatted_lines = traceback.format_exc().splitlines()
        consoleInsertText(formatted_lines[0])
        consoleInsertText(formatted_lines[-1])
        consoleInsertText(repr(traceback.format_exception(exc_type, exc_value,exc_traceback)))
        print ("*** extract_tb:") 
        print (repr(traceback.extract_tb(exc_traceback)))
        print ("*** format_tb:")
        print (repr(traceback.format_tb(exc_traceback)))
        print ("*** tb_lineno:", exc_traceback.tb_lineno)
        print("ERROR PLEASE CHECK TRACEBACK INFO")
        consoleInsertText("ERROR PLEASE CHECK TRACEBACK INFO" + '\n')
        
        if (mymower.visionRunning):
            mymower.visionRunning=False
        if(useMqtt):
            consoleInsertText('Close Mqtt Connection'+ '\n')
            Mqqt_DisConnection() 
        consoleInsertText('Start to save all Console Data'+ '\n')
        ButtonSaveReceived_click()  #save the console txt
        
        
        


    if mymower.useJoystick :
        myps4.listen()
        if myps4.leftClick:
            ButtonLeft_click()
        if myps4.rightClick:
            ButtonRight_click()       
        if myps4.upClick:
            send_var_message('w','motorSpeedMaxPwm',''+str(manualSpeedSlider.get())+'','0','0','0','0','0','0','0')
            send_pfo_message('nf','1','2','3','4','5','6',)
        if myps4.downClick:
            ButtonStop_click()
  
        #self.crossClick=False
        #self.roundClick=False
        #self.squareClick=False
        #self.triangle_click=False

        if myps4.triangleClick:
            pass
            #print("triangleClick:")
        if myps4.squareClick:
            buttonBlade_start_click()
            #print("squareClick:")
        if myps4.crossClick:
            button_stop_all_click()
            #print("crossClick:")
        if myps4.roundClick:
            buttonBlade_stop_click()
            #print("roundClick:")
        
    
   
    
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

    #one minute ticker
    if (time.time() > mymower.oneMinuteTrigger):
        if(mymower.isSendindMap):
            return
        #timer part
        if (mymower.startAfterUploadFinish != True): #do not start mowing if upload in process 
            print("timer loop")
            if ((mymower.ActualRunningTimer >= 98) and (mymower.opNames == "DOCK")): #no timer are already mowing
                checkTimerStart()
            if ((mymower.opNames == "Go TO DOCK") or (mymower.opNames == "MOW")):
                checkTimerStop()
                
            
                
        #clean the console page
        txtRecu.delete('2000.0',tk.END) #keep only  lines
        txtSend.delete('2000.0',tk.END) #keep only  lines
        txtConsoleRecu.delete('2500.0',tk.END) #keep only  lines
        
        #consoleInsertText("PI Température : " + str(cpu.temperature) + '\n')
        #start or stop the Raspberry fan
        if (cpu.temperature>=65.0):
            mymower.cpuFan=True
            ButtonFanStart_click()
        else:
            if(mymower.cpuFan==True):
                mymower.cpuFan=False
                ButtonFanStop_click()

        if (mymower.visionRunning==True) and (myRobot.statusNames[mymower.status]=="IN_STATION"):
            consoleInsertText("Station detected vision is not needed" + '\n')
            BtnVisionStop_click()
        mymower.oneMinuteTrigger=time.time()+60

    #five seconde ticker   
    if (time.time() > mymower.FiveSecondTrigger):
        tk_PiTemp.set(int(cpu.temperature))
        #initialisation of auto page
##        if ((time.time()-mymower.startPiArduTime) < 9):
##            ButtonAuto_click()                       
        mymower.FiveSecondTrigger=time.time()+5

    #one seconde ticker
    if (time.time() > mymower.OneSecondTrigger):
        if (mymower.isSendindMap):
            return
        tk_PiTemp.set(int(cpu.temperature))
        #refresh pi data from mcu
        message="AT+S"
        message=str(message)
        message=message + '\r'
        send_serial_message(message)              
        mymower.OneSecondTrigger=time.time()+1
    
    fen1.after(10,checkSerial)  # here is the main loop each 10ms
    
   

def updateAutoPage():
    
    
    if (mymower.targetPointx != mymower.lasttargetPointx):
        targetpos1=[mymower.statex,mymower.targetPointx]
        targetpos2=[mymower.statey,mymower.targetPointy]
        mymower.lasttargetPointx=mymower.targetPointx
        mymower.lasttargetPointy=mymower.targetPointy
        
        axLiveMap.plot(targetpos1,targetpos2,color='y',linewidth=1)

    
    mowerpos1=[mymower.laststatex,mymower.statex]
    mowerpos2=[mymower.laststatey,mymower.statey]
   
    axLiveMap.plot(mowerpos1,mowerpos2,color='g',linewidth=2)
    #axLiveMap.scatter(mymower.statex,mymower.statey, color='g', s=10)
    axLiveMap.autoscale(False)
    canvasLiveMap.draw()
    


    
#################################### END OF MAINLOOP ###############################################
def decode_AT_message(message):  #decode sunray console message
            know_message=False
            if mymower.useDebugConsole:
                txtRecu.insert('1.0', str(message)+ '\n')
            if message[:2] =='W,':
                know_message=True
                print(str(message))
                #consoleInsertText(str(message) + '\n')
                mymower.isSendindMap=False
                

                
            if message[:2] =='S,': # message statistique 'AT+S'
                know_message=True
                list_recu=(str(message).split(','))
                
                mymower.batteryVoltage=list_recu[1]
                tk_batteryVoltage.set(mymower.batteryVoltage)
                mymower.laststatex=mymower.statex
                mymower.statex=float(list_recu[2])
                GpsInfoline1.set("StateX : " + str(mymower.statex))
                mymower.laststatey=mymower.statey
                mymower.statey=float(list_recu[3])
                GpsInfoline2.set("StateY : " + str(mymower.statey))
                mymower.stateDelta=list_recu[4]
                
                GpsInfoline3.set("stateDelta : " + mymower.stateDelta )
                mymower.gpsSolution=list_recu[5]
                GpsInfoline4.set("gpsSolution : " + mymower.gpsSolution )
               
                tk_GpsSolution.set (myRobot.gpsSolution[int(list_recu[5])])
                timeNow=time.strftime('%H:%M:%S',time.localtime())

                tk_MainStatusLine.set(timeNow + " " + myRobot.opNames[int(list_recu[6])])
                mymower.opNames=str(myRobot.opNames[int(list_recu[6])])
                mymower.mowPointsIdx=int(list_recu[7])
                tk_mowPointIdx.set(str(mymower.mowPointsIdx) + "/" + str(mymower.mowPointsCount))
                if(mymower.mowPointsIdx >= mymower.stopMowPointIdx):
                    button_home_click()
                    
                #mymower.inc=list_recu[8] need to check what is it

                mymower.stateSensor=list_recu[9]
                GpsInfoline5.set("stateSensor : " + mymower.stateSensor )
                mymower.targetPointx=float(list_recu[10])
                GpsInfoline6.set("targetPointx : " + str(mymower.targetPointx))
                mymower.targetPointy=float(list_recu[11])
                GpsInfoline7.set("targetPointy : " + str(mymower.targetPointy) )
                mymower.gpsAccuracy=list_recu[12]
                GpsInfoline8.set("gpsAccuracy : " + mymower.gpsAccuracy )
                mymower.gpsnumSV=list_recu[13]
                
                mymower.batSense=float(list_recu[14])
                tk_batSense.set(mymower.batSense)
                
                mymower.gpsnumSVdgps=list_recu[15]

                GpsInfoline9.set("gpsnumSV : " + mymower.gpsnumSVdgps+ "/" + mymower.gpsnumSV)
                tk_gpsnumSV.set (mymower.gpsnumSVdgps + "/" + mymower.gpsnumSV)
               
                mymower.mapCRC=int(list_recu[16])
                GpsInfoline11.set("MapCRC : " + str(mymower.mapCRC ))
                
                mymower.lateralerror=list_recu[17]
                
                GpsInfoline10.set("lateralerror : " + mymower.lateralerror )
                
                    
                    

                if ((mymower.focusOnPage==1) and ((mymower.opNames == "MOW") or (mymower.opNames == "Go TO DOCK"))):

                    
                    #mymower.ActiveMapX.append(mymower.statex)
                    #mymower.ActiveMapY.append(mymower.statey)
                    #if mymower.OneSecondTrigger % 2 == 0 : #only each 2 seconde
                    updateAutoPage()
                    axLiveMap.set_aspect('equal','box')
                                
                
##                if (myRobot.opNames[int(list_recu[6])] == "MOW"):
##                    f=open(cwd + "/plot/PlotPos.txt",'a+')
##                    f.write("{};{}\n".format(float(mymower.statex) , float(mymower.statey)))
##                    f.close()
               
                

            if message[:2] =='Y3': # message power off
                know_message=True
                mymower.focusOnPage=4
                ConsolePage.tkraise()
                if (mymower.visionRunning):
                        mymower.visionRunning=False
                if(useMqtt):
                        consoleInsertText('Close Mqtt Connection'+ '\n')
                        Mqqt_DisConnection() 
                consoleInsertText('Start to save all Console Data'+ '\n')
                ButtonSaveReceived_click()  #save the console txt
                consoleInsertText('All Console Data are saved'+ '\n')
                consoleInsertText('PI start Shutdown'+ '\n')
                time.sleep(3)
                subprocess.Popen('/home/pi/Documents/PiArdumower/PowerOff.py')
                fen1.destroy()
                time.sleep(1)
                sys.exit("PowerOFF ordered by Arduino Due")

            if message[:2] =='V,': # message version 'AT+V'
                know_message=True
                list_recu=(str(message).split(','))
                #mymower.developerActive=message.developerActive
                mymower.version=list_recu[1] + " " + list_recu[2]
                Infoline1.set("Firmware : " + mymower.version + "           Pi version : " + PiVersion)
                #mymower.statsOverride=message.statsOverride
                Infoline2.set("??? : " + list_recu[3])
                Infoline3.set("??? : " + list_recu[4])
                Infoline4.set("MCU : " + list_recu[5])
                Infoline5.set("??? : " + list_recu[6])

                
            if message[:3] =='RN,': # message maps main data
                know_message=True
                list_recu=(str(message).split(','))
                #consoleInsertText(str(list_recu) + '\n')
                mymower.perimeterPointsCount=int(list_recu[1])
                mymower.exclusionPointsCount=int(list_recu[2])
                mymower.dockPointsCount=int(list_recu[3])
                mymower.mowPointsCount=int(list_recu[4])
                mymower.freePointsCount=int(list_recu[5])
                message="AT+RNX"
                message=str(message)
                message=message + '\r'
                send_serial_message(message)

            if message[:3] =='RNX': # message maps main data
                know_message=True
                list_recu=(str(message).split(','))
                #consoleInsertText(str(list_recu) + '\n')
                mymower.nbTotalExclusion=int(list_recu[1])
                maindata=np.array([mymower.perimeterPointsCount,mymower.exclusionPointsCount,mymower.dockPointsCount,mymower.mowPointsCount,mymower.freePointsCount,mymower.nbTotalExclusion,mymower.mapCRC])

                fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/MAIN"
                output_file = Path(fileName)
                output_file.parent.mkdir(exist_ok=True, parents=True)
                np.save(fileName, maindata, allow_pickle=True, fix_imports=True)


                #update the main CRC datalist according to map nr
                fileName=cwd + "/House" + "{0:0>2}".format(mymower.House)+"/crcMapList.npy"
                if (os.path.exists(fileName)):
                    crcMapList=np.load(fileName)
                    print(fileName)
                    print(crcMapList)
                    find_row=np.where(crcMapList[:,1] == mymower.mapCRC)
                    result=crcMapList[find_row]
                    
                    if (result.size != 0):
                        if (result[0,1]==mymower.mapCRC):
                            messagebox.showwarning('warning',"This map already exist ,it's map : " + str(result[0,0]))
                            MapsPage.select(result[0,0])
                            mymower.mapSelected = int(MapsPage.index("current"))
                        else:
                            messagebox.showwarning('warning',"Error while loading crcMapList")
                           
                    else:
                        #add the new map
                        newrow=[mymower.mapSelected,mymower.mapCRC]
                        crcMapList = np.vstack([crcMapList, newrow])
                        np.save(fileName, crcMapList, allow_pickle=True, fix_imports=True)
                        print(crcMapList)
                        message="AT+RP"
                        message=str(message)
                        message=message + '\r'
                        send_serial_message(message)
                else:
                    #create the file for the first time
                    crcMapList=np.array([[mymower.mapSelected,mymower.mapCRC]])
                    np.save(fileName, crcMapList, allow_pickle=True, fix_imports=True)
                    message="AT+RP"
                    message=str(message)
                    message=message + '\r'
                    send_serial_message(message)
                
                       
            if message[:2] =='RP': # message perimeter list point
                print(message)
                mymower.actualMowingArea=0
                know_message=True
                pointCount=0
                list_recu=(str(message).split(','))
                list_recu.pop(0)  #remove first
                list_recu.pop(len(list_recu)-1) #remove last
                pointCount=int(len(list_recu)/2)
                dataArray = np.array(list_recu).reshape(pointCount,2)
                mymower.map[mymower.mapSelected] = dataArray.astype(float) # to float data
                fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/PERIMETER"
                output_file = Path(fileName)
                output_file.parent.mkdir(exist_ok=True, parents=True)
                np.save(fileName, mymower.map[mymower.mapSelected], allow_pickle=True, fix_imports=True)
                consoleInsertText('Perimeter map total point : '+ str(pointCount)+ '\n')

                perimeterArray=mymower.map[mymower.mapSelected]
                polygon1 = Polygon(np.squeeze(perimeterArray))
                mymower.actualMowingArea=int(polygon1.area)       


                message="AT+RM"
                message=str(message)
                message=message + '\r'
                send_serial_message(message)
                
            if message[:2] =='RM': # message mow list point
                know_message=True
                pointCount=0
                list_recu=(str(message).split(','))
                list_recu.pop(0)  #remove first
                list_recu.pop(len(list_recu)-1) #remove last
                pointCount=int(len(list_recu)/2)
                dataArray = np.array(list_recu).reshape(pointCount,2)
                mymower.map[mymower.mapSelected] = dataArray.astype(float) # to float data
               

                fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/MOW"
                output_file = Path(fileName)
                output_file.parent.mkdir(exist_ok=True, parents=True)
                np.save(fileName, mymower.map[mymower.mapSelected], allow_pickle=True, fix_imports=True)
                consoleInsertText('Mowing map point : ' + str(pointCount) + '\n')
                message="AT+RD"
                message=str(message)
                message=message + '\r'
                send_serial_message(message)
                
            if message[:2] =='RD': # message dock list point
                know_message=True
                pointCount=0
                list_recu=(str(message).split(','))
                list_recu.pop(0)  #remove first
                list_recu.pop(len(list_recu)-1) #remove last
                pointCount=int(len(list_recu)/2)
                dataArray = np.array(list_recu).reshape(pointCount,2)
                mymower.map[mymower.mapSelected] = dataArray.astype(float) # to float data
                fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/DOCK"
                output_file = Path(fileName)
                output_file.parent.mkdir(exist_ok=True, parents=True)
                np.save(fileName, mymower.map[mymower.mapSelected], allow_pickle=True, fix_imports=True)
                consoleInsertText('Docking point : '+ str(pointCount) + '\n')
                               
                message="AT+RF"
                message=str(message)
                message=message + '\r'
                send_serial_message(message)


            if message[:2] =='RF': # message free point
                know_message=True
                pointCount=0
                list_recu=(str(message).split(','))
                list_recu.pop(0)  #remove first
                list_recu.pop(len(list_recu)-1) #remove last
                pointCount=int(len(list_recu)/2)
                dataArray = np.array(list_recu).reshape(pointCount,2)
                mymower.map[mymower.mapSelected] = dataArray.astype(float) # to float data
                fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/FREE"
                output_file = Path(fileName)
                output_file.parent.mkdir(exist_ok=True, parents=True)
                np.save(fileName, mymower.map[mymower.mapSelected], allow_pickle=True, fix_imports=True)
                consoleInsertText('Free point : '+ str(pointCount) + '\n')
                mymower.exclusionNr=0

                message="AT+RX," +str(mymower.exclusionNr) + ","
                message=str(message)
                message=message + '\r'
                send_serial_message(message)

                print("   ")
                print(message)
                
                

            

            if message[:2] =='RX': # message exclusion list point
                know_message=True
                
                pointCount=0
                list_recu=(str(message).split(','))
                list_recu.pop(0)  #remove first
                list_recu.pop(len(list_recu)-1) #remove last
                pointCount=int(len(list_recu)/2)
                dataArray = np.array(list_recu).reshape(pointCount,2)
                mymower.map[mymower.mapSelected] = dataArray.astype(float) # to float data
               
                fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/EXCLUSION"+"{0:0>2}".format(mymower.exclusionNr)
                output_file = Path(fileName)
                output_file.parent.mkdir(exist_ok=True, parents=True)
 
                np.save(fileName, mymower.map[mymower.mapSelected], allow_pickle=True, fix_imports=True)
                consoleInsertText('Add Exclusion nr : ' + str(mymower.exclusionNr) + ' point : ' + str(pointCount)+ '\n')
                print(mymower.exclusionNr, "/", mymower.nbTotalExclusion)
                mymower.exclusionNr=mymower.exclusionNr+1
                
                if (mymower.exclusionNr<mymower.nbTotalExclusion):
                    consoleInsertText('new exclusion : '+ str(mymower.exclusionNr)+ '\n')
                    
                    message="AT+RX," +str(mymower.exclusionNr) + ","
                    message=str(message)
                    message=message + '\r'
                    send_serial_message(message)
                else:
                    consoleInsertText('Import Finish ' + '\n')
                    consoleInsertText('mapCRC = ' + str(mymower.mapCRC) + '\n')
                    #canvas[mymower.mapSelected].draw
                    onTabChange(mymower.mapSelected)
                    


            if message[:2] =='X,': # message exclusion list point
                know_message=True
                print("message exclusion list point  ")
                print(message)
                consoleInsertText('message exclusion list point ' + '\n')
                consoleInsertText(str(message) + '\n')


            if message[:3] =='FI,': # message exclusion list point
                know_message=True
                mymower.finishedUploadingMap=True
                tk_HouseView.set(0)
                initActiveMap(0)
                initialPlotAutoPage(0,99999)
                print("Map upload Finish and feedback OK  ")
                if (mymower.startAfterUploadFinish):
                    print ("time to start mowing")
                    mymower.startAfterUploadFinish=False
                    buttonStartMow_click()
                    
                                                    

                
                





                

            if message[:4] == 'CON:':
                know_message=True
##            if message[:2] == '0:':#a rectifier uniquement pour test
##                know_message=True

            if not (know_message):
                consoleInsertText(str(message))
                







                             




    


def BtnMowPlotStartRec_click():
    """create an empty txt file to have correct auto legend into the graph """
    f=open(cwd + "/plot/PlotMow.txt",'w')
    f.write("{};{};{}\n".format("Time","motorMowPower","motorMowPWMCurr"))
    f.write("{};{};{}\n".format("0","0","0"))
    f.close()
    mowPlotterKst.start('/home/pi/Documents/PiArdumower/plotMow.kst')
    send_req_message('MOW',''+str(SldMainMowRefresh.get())+'','1','10000','0','0','0',) #arduino start sending data
    
def BtnMowPlotStopRec_click():
    global firstplotMowx
    firstplotMowx=0  #initialise the first time plot for next plot
    mowPlotterKst.stop() #close the kst prog
    
    send_req_message('MOW','1','0','0','0','0','0',) #arduino stop sending data
    
    filename=cwd + "/plot/Plotmow" + time.strftime("%Y%m%d%H%M") + ".CSV"
    try:  #avoid error if file not exit 
        os.rename(cwd + "/plot/PlotMow.txt",filename) #keep a copy of the plot and clear the last kst file
        messagebox.showinfo('Info',"File " + filename + " created in plot directory")
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
    f=open(cwd + "/plot/PlotPeri.txt",'w')
    f.write("{};{};{}\n".format("Time","perimeterMag","perimeterMagRight"))
    f.write("{};{};{}\n".format("0","0","0"))
    f.close()
    periPlotterKst.start('/home/pi/Documents/PiArdumower/plotPeri.kst')
    send_req_message('PERI',''+str(SldMainPeriRefresh.get())+'','1','10000','0','0','0',)
    
def BtnPeriPlotStopRec_click():
    global firstplotPerx
    firstplotPerx=0
    periPlotterKst.stop() #close the kst prog
    
    send_req_message('PERI','1','0','0','0','0','0',)
    
    filename=cwd + "/plot/PlotPeri" + time.strftime("%Y%m%d%H%M") + ".CSV"
    try:
        os.rename(cwd + "/plot/PlotPeri.txt",filename) #keep a copy of the plot and clear the last kst file
        messagebox.showinfo('Info',"File " + filename + " created in plot directory")
    except OSError:
        print("Error when rename file /plot/PlotPeri.txt")
        consoleInsertText("Error when rename file /plot/PlotPeri.txt" + '\n')
        pass
    
    
def BtnBatPlotStartRec_click():
    #create an empty txt file to have correct auto legend into the graph """
    f=open(cwd + "/plot/PlotBat.txt",'w')
    f.write("{};{};{};{}\n".format("Time","chgVoltage","chgSense","batteryVoltage"))
    f.write("{};{};{};{}\n".format("0","0","0","0"))
    f.close()
    #if(mymower.autoRecordBatChargeOn!=True):#it's not the auto record so need to start KST
    batPlotterKst.start('/home/pi/Documents/PiArdumower/plotBat.kst')
    send_req_message('BAT',''+str(SldMainBatRefresh.get())+'','1','10000','0','0','0',)    

def BtnBatPlotStopRec_click():
    global firstplotBatx
    firstplotBatx=0
    batPlotterKst.stop() #close the kst prog
    
    send_req_message('BAT','1','0','0','0','0','0',)
    
    filename=cwd + "/plot/PlotBattery" + time.strftime("%Y%m%d%H%M") + ".CSV"
    try:
        os.rename(cwd + "/plot/PlotBat.txt",filename) #keep a copy of the plot and clear the last kst file
        #//bber17
        #if(mymower.autoRecordBatChargeOn==False): #it's not the auto record so send a message
        #messagebox.showinfo('Info',"File " + filename + " created in plot directory")
    except OSError:
        print("Error when rename file /plot/PlotBat.txt")
        consoleInsertText("Error when rename file /plot/PlotBat.txt" + '\n')
        
    
    
def BtnImuPlotStartRec_click():
    """create an empty txt file to have correct auto legend into the graph """
    f=open(cwd + "/plot/PlotImu.txt",'w')
    f.write("{};{};{}\n".format("Time","GyroYaw","CompassYaw"))
    f.write("{};{};{}\n".format("0","0","0"))
    f.close()
    ImuPlotterKst.start('/home/pi/Documents/PiArdumower/plotImu.kst')
    send_req_message('IMU',''+str(SldMainImuRefresh.get())+'','1','10000','0','0','0',)    

def BtnImuPlotStopRec_click():
    global firstplotImux
    firstplotImux=0
    ImuPlotterKst.stop() #close the kst prog
    
    send_req_message('IMU','1','0','0','0','0','0',)
    
    filename=cwd + "/plot/PlotImu" + time.strftime("%Y%m%d%H%M") + ".CSV"
    try:
        os.rename(cwd + "/plot/PlotImu.txt",filename) #keep a copy of the plot and clear the last kst file
        messagebox.showinfo('Info',"File " + filename + " created in plot directory")
    except OSError:
        print("Error when rename file /plot/PlotImu.txt")
        consoleInsertText("Error when rename file /plot/PlotImu.txt" + '\n')
        pass
    
 

   
def BtnBylaneStartRec_click():
    send_req_message('BYL','3','1','6000','0','0','0',)
    
def BtnBylaneStopRec_click():
    send_req_message('BYL','1','0','0','0','0','0',)
    
def BtnMotPlotStartRec_click():
    """create an empty txt file to have correct auto legend into the graph """
    f=open(cwd + "/plot/PlotMot.txt",'w')
    f.write("{};{};{};{};{}\n".format("Time","motorLeftPower","motorRightPower","motorLeftPWMCurr","motorRightPWMCurr"))
    f.write("{};{};{};{};{}\n".format("0","0","0","0","0"))
    f.close()
    motPlotterKst.start('/home/pi/Documents/PiArdumower/plotMot.kst')
    send_req_message('MOT',''+str(SldMainWheelRefresh.get())+'','1','10000','0','0','0',)
    
def BtnMotPlotStopRec_click():
    global firstplotMotx
    firstplotMotx=0
    motPlotterKst.stop() #close the kst prog
    send_req_message('MOT','1','0','0','0','0','0',)
    filename=cwd + "/plot/PlotMotor" + time.strftime("%Y%m%d%H%M") + ".CSV"
    try:
        os.rename(cwd + "/plot/PlotMot.txt",filename) #keep a copy of the plot and clear the last kst file
        messagebox.showinfo('Info',"File " + filename + " created in plot directory")
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
   
    #slidermowPatternDurationMax.set(myRobot.mowPatternDurationMax)
    slidermotorMowSpeedMaxPwm.set(myRobot.motorMowSpeedMaxPwm)
    slidermotorMowPowerMax.set(myRobot.motorMowPowerMax)
    slidermotorMowSpeedMinPwm.set(myRobot.motorMowSpeedMinPwm)    
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
    if myRobot.stopMotorDuringCalib=='1':
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
    #manualSpeedSlider.set(myRobot.motorSpeedMaxPwm)
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
    #ButtonSendSettingToDue_click

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
    fileName=cwd + "/log/" + time.strftime("%Y%m%d%H%M") + "_Received.txt" 
    with open(fileName,"w") as f:
        f.write(txtRecu.get('1.0','end'))
    fileName=cwd + "/log/" + time.strftime("%Y%m%d%H%M") + "_Send.txt"
    with open(fileName,"w") as f:
        f.write(txtSend.get('1.0','end'))
    fileName=cwd + "/log/" + time.strftime("%Y%m%d%H%M") + "_Console.txt"
    with open(fileName,"w") as f:
        f.write(txtConsoleRecu.get('1.0','end'))

    consoleInsertText('All Console file are saved' + '\n')
    

    



def button_stop_all_click():
    mymower.startAfterUploadFinish=False
    mymower.lastRunningTimer=mymower.ActualRunningTimer
    mymower.ActualRunningTimer = 99
    message="AT+C,-1,0,-1,-1,-1,-1,-1,-1"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)  
    

def ButtonInfo_click():
    message="AT+V"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)
    mymower.focusOnPage=11
    InfoPage.tkraise()
    
def ButtonCamera_click():
    mymower.focusOnPage=8
    StreamVideoPage.tkraise()
    
##def ButtonGps_click():
##    mymower.focusOnPage=9
##    GpsPage.tkraise()
    
def ButtonMaps_click():
    mymower.focusOnPage=9
    MapsPage.tkraise()
    MapsPage.select(0)
    #MapsPage.select(mymower.mapSelected)

def ButtonSchedule_click():
    mymower.focusOnPage=7
    TabTimer.tkraise()
    





def ButtonManual_click():
    manualSpeedSlider.set(myRobot.motorSpeedMaxPwm)
    mymower.focusOnPage=2
    ManualPage.tkraise()
        
def ButtonPlot_click():
    mymower.focusOnPage=6
    TabPlot.tkraise()

def ButtonSetting_click():
    mymower.focusOnPage=3
    TabSetting.tkraise()
   
def ButtonBackToMain_click():
    mymower.focusOnPage=0
    MainPage.tkraise()
    
                    
def ButtonConsole_click():
    mymower.focusOnPage=4
    ConsolePage.tkraise()

def ButtonTest_click():
    mymower.focusOnPage=5
    TestPage.tkraise()
    
def ButtonAuto_click():
    mymower.focusOnPage=1
    AutoPage.tkraise() 
    
    
    #try to find the map crc in the raspberry pi and init eveything
    #check if the map locate into mower have changed
    fileName=cwd + "/House" + "{0:0>2}".format(mymower.House)+"/crcMapList.npy"
    if ((os.path.exists(fileName)) & (mymower.mapCRC !=0)):
        crcMapList=np.load(fileName)
        map_find=False
        for ip in range(int(len(crcMapList))):
            #print("ctrl")
            #print(crcMapList[ip,1])
            #print(mymower.mapCRC)
            ctrl=crcMapList[ip,1]-mymower.mapCRC
            #print(ctrl)
            if (abs(ctrl)<mymower.mapCrcRoundingRange):
                mymower.mapSelected = int(crcMapList[ip,0])
##                print("Auto page map selected")
##                print(mymower.mapSelected)
##                print("auto page map CRC")
##                print(mymower.mapCRC)
                map_find=True
                if(mymower.full_house==0):
                    initActiveMap(0)
                    initialPlotAutoPage(mymower.startMowPointIdx,mymower.stopMowPointIdx)
                else:
                    rebuildHouseMap()
                    #initActiveMap(1)
                    #initialPlotAutoPageFullHouse()
           
                    
                return
        if (map_find==False):
            messagebox.showwarning('warning',"Map unknow (try to upload, see map page)" + str(mymower.mapCRC))
            
                
    
    else:
        rebuildHouseMap()
##        initActiveMap(1)
##        initialPlotAutoPageFullHouse()

    
   
    



def ButtonSendSettingByLaneToDue_click():
    myRobot.yawSet1=slideryawSet1.get()
    myRobot.yawSet2=slideryawSet2.get()
    myRobot.yawSet3=slideryawSet3.get()
       
    
    
def ButtonSendSettingDateTimeToDue_click():
    Send_reqSetting_message('Time','w','1',''+str(tk_date_hour.get())+\
                            '',''+str(tk_date_minute.get())+\
                            '',''+str(tk_date_dayOfWeek.get())+\
                            '',''+str(tk_date_day.get())+\
                            '',''+str(tk_date_month.get())+\
                            '',''+str(tk_date_year.get())+\
                            '',''+str(0)+\
                            '',''+str(0)+\
                            '',''+str(0)+\
                            '',''+str(0)+'',)
    
def ButtonSendSettingToDue_click():
    


    consoleInsertText("All Setting are change into the Due but not save for the moment" + '\n') 
    

     
def read_all_setting():
    Send_reqSetting_message('All','r','0','0','0','0','0','0','0','0','0','0','0')
    
def read_time_setting():
    Send_reqSetting_message('Time','r','0','0','0','0','0','0','0','0','0','0','0')
    
       
def send_req_message(val1,val2,val3,val4,val5,val6,val7):
    send_serial_message(message)
   

def send_var_message(val1,val2,val3,val4,val5,val6,val7,val8,val9,val10):
    send_serial_message(message)
    
def send_cmd_message(val1,val2,val3,val4,val5):
    send_serial_message(message)

    
    
def send_pfo_message(val1,val2,val3,val4,val5,val6,val7):
    message = pynmea2.PFO('RM', 'PFO', (val1,val2,val3,val4,val5,val6,val7))
    message=str(message)
    message=message + '\r' +'\n'
    send_serial_message(message)
    
def Send_reqSetting_message(val1,val2,val3,val4,val5,val6,val7,val8,val9,val10,val11,val12,val13):
    send_serial_message(message)   
               
def send_serial_message(message1):
    
    try:
        if DueConnectedOnPi:
            #checkSerial()
            Due_Serial.flush()
            Due_Serial.write(bytes(message1,'utf-8'))
            
            if mymower.useDebugConsole:
                txtSend.insert('1.0',  message1) 
            
    except :
            print("ERREUR while transfert")
            time.sleep(2)




""" ------------------- connecting the the PCB  ------------ """
try:
    if DueConnectedOnPi :
        if myOS == "Linux":
            if os.path.exists('/dev/ttyACM0') == True:
                Due_Serial = serial.Serial('/dev/ttyACM0',115200,timeout=2,write_timeout = 2)
                Due_Serial.flushInput()
                Due_Serial.flushOutput()  # clear the output buffer
                print("Find Serial on ttyACM0")

            if os.path.exists('/dev/ttyACM1') == True:
                Due_Serial = serial.Serial('/dev/ttyACM1',115200,timeout=10,write_timeout = 10)
                Due_Serial.flushInput()
                Due_Serial.flushOutput()  # clear the output buffer
                print("Find Serial on ttyACM1")
        else:
            Due_Serial = serial.Serial('COM9',115200,timeout=0)
            
        #byteResponse=Due_Serial.readline()
        #print(str(byteResponse))

        
        
except:
        print(" ")
        print("************************************")
        print("ERREUR DE CONNECTION WITH PCB1.3 DUE")
        print("************************************")
        print(" ")
        time.sleep(1)
        #sys.exit("Impossible de continuer")




"""-------------------ICONS LOADING---------------------"""
imgHome=tk.PhotoImage(file=cwd + "/icons/home.png")
imgTrack=tk.PhotoImage(file=cwd + "/icons/track.png")
imgStopAll=tk.PhotoImage(file=cwd + "/icons/stop all.png")
imgstartMow=tk.PhotoImage(file=cwd + "/icons/startmow.png")
imgBack=tk.PhotoImage(file=cwd + "/icons/back.png")
imgBladeStop = tk.PhotoImage(file=cwd + "/icons/bladeoff.png")
imgBladeStart = tk.PhotoImage(file=cwd + "/icons/bladeon.png")
imgForward=tk.PhotoImage(file=cwd + "/icons/forward.png")
imgForwardLeft=tk.PhotoImage(file=cwd + "/icons/forLeft.png")
imgForwardRight=tk.PhotoImage(file=cwd + "/icons/forRight.png")



imgReverse=tk.PhotoImage(file=cwd + "/icons/reverse.png")
imgLeft=tk.PhotoImage(file=cwd + "/icons/left.png")
imgRight=tk.PhotoImage(file=cwd + "/icons/right.png")
imgStop=tk.PhotoImage(file=cwd + "/icons/stop.png")
imgArdumower = tk.PhotoImage(file=cwd + "/icons/ardumower.png")
imgManual=tk.PhotoImage(file=cwd + "/icons/manual.png")
imgAuto=tk.PhotoImage(file=cwd + "/icons/auto.png")
imgTest=tk.PhotoImage(file=cwd + "/icons/test.png")
imgConsole=tk.PhotoImage(file=cwd + "/icons/console.png")
imgSetting=tk.PhotoImage(file=cwd + "/icons/setting.png")
imgPowerOff=tk.PhotoImage(file=cwd + "/icons/off.png")
imgPlot=tk.PhotoImage(file=cwd + "/icons/plot.png")
imgSchedule=tk.PhotoImage(file=cwd + "/icons/schedule.png")
imgCamera=tk.PhotoImage(file=cwd + "/icons/camera.png")
imgGps=tk.PhotoImage(file=cwd + "/icons/gps.png")
imgJoystickON=tk.PhotoImage(file=cwd + "/icons/joystick_on.png")
imgJoystickOFF=tk.PhotoImage(file=cwd + "/icons/joystick_off.png")
imgJoystick=tk.PhotoImage(file=cwd + "/icons/joystick.png")
imgMaps=tk.PhotoImage(file=cwd + "/icons/map.png")



""" THE SETTING PAGE ****************************************************"""

def ButtonSaveSettingToFile_click():

    setting_list=[]

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
    setting_list.append(myRobot.motorMowPowerMax)

    ButtonSetBatteryApply_click()
    setting_list.append(myRobot.batGoHomeIfBelow)
    setting_list.append(myRobot.batSwitchOffIfBelow)
    setting_list.append(myRobot.batSwitchOffIfIdle)
    setting_list.append(myRobot.startChargingIfBelow)
    setting_list.append(myRobot.batFullCurrent)
    setting_list.append(mymower.useDebugConsole)
   
 
    print (setting_list)
    
    with open("setting_list.bin","wb") as fp :
        pickle.dump(setting_list,fp)      
        print("setting file saved")
    
        

def ButtonReadSettingFromFile_click():
    setting_list=[]
    print ("read setting file")   
    with open("setting_list.bin","rb") as fp :
        setting_list=pickle.load(fp)
    myRobot.odometryTicksPerRevolution=setting_list[0]
    myRobot.odometryWheelDiameter=setting_list[1]
    myRobot.odometryWheelBaseCm=setting_list[2]
    refreshOdometrySettingPage()
   
    
    LinearSpeedSlider.set(setting_list[3])
    DockingSpeedSlider.set(setting_list[4])
    mymower.timerUse=setting_list[5]
    mymower.rainUse=setting_list[6]
   

    
    myRobot.motorPowerMax=setting_list[7]
    refreshMotorSettingPage()
    
    
    myRobot.motorMowSpeedMaxPwm=setting_list[8]
    myRobot.motorMowSpeedMinPwm=setting_list[9]
    myRobot.motorMowPowerMax=setting_list[10]
    refreshMowMotorSettingPage()

    
    myRobot.batGoHomeIfBelow=setting_list[11]
    myRobot.batSwitchOffIfBelow=setting_list[12]
    myRobot.batSwitchOffIfIdle=setting_list[13]
    myRobot.startChargingIfBelow=setting_list[14]
    myRobot.batFullCurrent=setting_list[15]
    refreshBatterySettingPage()

    mymower.useDebugConsole=setting_list[16]
    #print("rr",mymower.useDebugConsole)
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
    #refreshTimerSettingPage()
    


def ButtonSetMowMotorApply_click():
    myRobot.motorMowSpeedMaxPwm=slidermotorMowSpeedMaxPwm.get()
    myRobot.motorMowSpeedMinPwm=slidermotorMowSpeedMinPwm.get()
    myRobot.motorMowPowerMax=slidermotorMowPowerMax.get()


def ButtonSetMotApply_click():
    myRobot.motorPowerMax=sliderPowerMax.get()

def ButtonSetBatteryApply_click():
    myRobot.batGoHomeIfBelow=sliderbatGoHomeIfBelow.get()
    myRobot.batSwitchOffIfBelow=sliderbatSwitchOffIfBelow.get()
    myRobot.batSwitchOffIfIdle=sliderbatSwitchOffIfIdle.get()
    myRobot.startChargingIfBelow=sliderstartChargingIfBelow.get()
    myRobot.batFullCurrent=sliderbatFullCurrent.get()
  
def ButtonSetOdometryApply_click():
    myRobot.odometryTicksPerRevolution=sliderodometryTicksPerRevolution.get()
    myRobot.odometryWheelDiameter=sliderodometryWheelDiameter.get()
    myRobot.odometryWheelBaseCm=sliderodometryWheelBaseCm.get()
   
def ButtonSetImuApply_click():
    myRobot.imuDirPID_Kp=sliderimuDirPID_Kp.get()
    myRobot.imuDirPID_Ki=sliderimuDirPID_Ki.get()
    myRobot.imuDirPID_Kd=sliderimuDirPID_Kd.get()   
    myRobot.delayBetweenTwoDmpAutocalib=sliderdelayBetweenTwoDmpAutocalib.get()
    myRobot.maxDurationDmpAutocalib=slidermaxDurationDmpAutocalib.get()
    myRobot.maxDriftPerSecond=slidermaxDriftPerSecond.get()
    
    myRobot.stopMotorDuringCalib='0'
    if ImuVar1.get()==1:
        myRobot.stopMotorDuringCalib='1'
    ButtonSendSettingToDue_click()
    

def ButtonSetMainApply_click():

    mymower.timerUse=False
    if tk_MaintimerUse.get()==1:
        mymower.timerUse=True       
    mymower.rainUse=False
    if tk_MainrainUse.get()==1:
        mymower.rainUse=True
    mymower.useDebugConsole=False
    if tk_useDebugConsole.get()==1:
        mymower.useDebugConsole=True
    

TabSetting=ttk.Notebook(fen1)
TabSetting.place(x=0,y=0)

#TabConsole=ttk.Notebook(fen1)
#img1=tk.PhotoImage(file="/pi/Ardumawer/img/setting1.png")
tabMain=tk.Frame(TabSetting,width=800,height=380)
tabWheelMotor=tk.Frame(TabSetting,width=800,height=380)
tabMowMotor=tk.Frame(TabSetting,width=800,height=380)
#tabPerimeter=tk.Frame(TabSetting,width=800,height=380)
#tabImu=tk.Frame(TabSetting,width=800,height=380)
#tabSonar=tk.Frame(TabSetting,width=800,height=380)
tabBattery=tk.Frame(TabSetting,width=800,height=380)
tabOdometry=tk.Frame(TabSetting,width=800,height=380)
#tabDateTime=tk.Frame(TabSetting,width=800,height=380)
#tabByLane=tk.Frame(TabSetting,width=800,height=380)
tabVision=tk.Frame(TabSetting,width=800,height=380)




TabSetting.add(tabMain,text="Main")
TabSetting.add(tabWheelMotor,text="Wheels Motor")
TabSetting.add(tabMowMotor,text="Mow Motor")
#TabSetting.add(tabPerimeter,text="Perimeter")
#TabSetting.add(tabImu,text="Imu")
#TabSetting.add(tabSonar,text="Sonar")
TabSetting.add(tabBattery,text="Battery")
TabSetting.add(tabOdometry,text="Odometry")
#TabSetting.add(tabDateTime,text="Date Time")
#TabSetting.add(tabByLane,text="ByLane")
TabSetting.add(tabVision,text="Vision")


"""************* Main setting *****************************"""

def updateChkBtnrainUse(event):
   ButtonSetMainApply_click()
   
def updateBtnuseDebugConsole(event):
   ButtonSetMainApply_click()

def updateChkBtntimerUse(event):
   ButtonSetMainApply_click()
    
    
def updateDockingSpeedSlider(event):
    print("updateDockingSpeedSlider",str(DockingSpeedSlider.get()))
    message="AT+CT,9," + str(DockingSpeedSlider.get()) +",0"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)

ButtonSaveSettingToFile= tk.Button(tabMain)
ButtonSaveSettingToFile.place(x=30,y=15, height=25, width=120)
ButtonSaveSettingToFile.configure(command = ButtonSaveSettingToFile_click)
ButtonSaveSettingToFile.configure(text="Save")

ButtonReadSettingFromFile= tk.Button(tabMain)
ButtonReadSettingFromFile.place(x=30,y=65, height=25, width=120)
ButtonReadSettingFromFile.configure(command = ButtonReadSettingFromFile_click)
ButtonReadSettingFromFile.configure(text="Read")

ButtonFlashDue= tk.Button(tabMain)
ButtonFlashDue.place(x=0,y=175, height=30, width=200)
ButtonFlashDue.configure(command = ButtonFlashDue_click)
ButtonFlashDue.configure(text="Update ROBOT Firmware")

##def ButtonReboot_click():
##    #01/09/2023 actualy nothing dev here because reboot can power off PCB and pi
##    print("reboot")
##ButtonReboot = tk.Button(tabMain,text="Reboot All",  command = ButtonReboot_click)
##ButtonReboot.place(x=30,y=115, height=40, width=100)

LinearSpeedSlider = tk.Scale(tabMain,orient='horizontal',label='Linear SPEED',relief=tk.SOLID,from_=0, to=0.8)
LinearSpeedSlider.config(resolution=0.05)
LinearSpeedSlider.place(x=270,y=10,width=200, height=55)
LinearSpeedSlider.set(0.4)

DockingSpeedSlider = tk.Scale(tabMain,orient='horizontal',label='Docking SPEED',relief=tk.SOLID, resolution=0.1 ,from_=0, to=0.8)
DockingSpeedSlider.place(x=270,y=65,width=200, height=55)
DockingSpeedSlider.set(0.2)
DockingSpeedSlider.bind("<ButtonRelease-1>", updateDockingSpeedSlider)

ChkBtntimerUse=tk.Checkbutton(tabMain, text="Use Timer",relief=tk.SOLID,variable=tk_MaintimerUse,onvalue=1,offvalue=0,anchor='nw')
ChkBtntimerUse.place(x=530,y=40,width=250, height=20)
ChkBtntimerUse.bind("<ButtonRelease-1>", updateChkBtntimerUse)

ChkBtnrainUse=tk.Checkbutton(tabMain, text="Use Rain Sensor",relief=tk.SOLID,variable=tk_MainrainUse,onvalue=1,offvalue=0,anchor='nw')
ChkBtnrainUse.place(x=530,y=70,width=250, height=20)
ChkBtnrainUse.bind("<ButtonRelease-1>", updateChkBtnrainUse)

ChkBtnuseDebugConsole=tk.Checkbutton(tabMain, text="Debug console",relief=tk.SOLID,variable=tk_useDebugConsole,onvalue=1,offvalue=0,anchor='nw')
ChkBtnuseDebugConsole.place(x=530,y=100,width=250, height=20)
ChkBtnuseDebugConsole.bind("<ButtonRelease-1>", updateBtnuseDebugConsole)

#ButtonSetMainApply = tk.Button(tabMain)
#ButtonSetMainApply.place(x=300,y=350, height=25, width=100)
#ButtonSetMainApply.configure(command = ButtonSetMainApply_click,text="Send To Mower")



ButtonBackHome = tk.Button(TabSetting, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=120, width=120)



"""************* Mow motor setting *****************************"""

slidermotorMowSpeedMaxPwm = tk.Scale(tabMowMotor, from_=100, to=255, label='Max PWM Speed',relief=tk.SOLID,orient='horizontal')
slidermotorMowSpeedMaxPwm.place(x=10,y=10,width=250, height=50)

slidermotorMowSpeedMinPwm = tk.Scale(tabMowMotor, from_=100, to=255, label='Min PWM Speed',relief=tk.SOLID,orient='horizontal')
slidermotorMowSpeedMinPwm.place(x=10,y=60,width=250, height=50)

slidermotorMowPowerMax = tk.Scale(tabMowMotor, from_=0, to=100, label='Power Max in Watt',relief=tk.SOLID,orient='horizontal')
slidermotorMowPowerMax.place(x=270,y=60,width=250, height=50)



"""************* Odometry setting *****************************"""
def updatesliderodometryTicksPerRevolution(event):
    message="AT+CT,4," + str(sliderodometryTicksPerRevolution.get()) +",0"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)

def updatesliderodometryWheelBaseCm(event):
    message="AT+CT,5," + str(sliderodometryWheelBaseCm.get()) +",0"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)  

def updatesliderodometryWheelDiameter(event):
    message="AT+CT,6," + str(sliderodometryWheelDiameter.get()) +",0"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)  


sliderodometryTicksPerRevolution = tk.Scale(tabOdometry, from_=500, to=1500, label='Ticks for one full revolution',relief=tk.SOLID,orient='horizontal')
sliderodometryTicksPerRevolution.place(x=10,y=10,width=300, height=50)
sliderodometryTicksPerRevolution.bind("<ButtonRelease-1>", updatesliderodometryTicksPerRevolution)

sliderodometryWheelDiameter = tk.Scale(tabOdometry, from_=100, to=350, label='Wheels diameters in mm  ',relief=tk.SOLID,orient='horizontal')
sliderodometryWheelDiameter.place(x=10,y=60,width=300, height=50)
sliderodometryWheelDiameter.bind("<ButtonRelease-1>", updatesliderodometryWheelDiameter)

sliderodometryWheelBaseCm = tk.Scale(tabOdometry, from_=0, to=50, label='Distance between the 2 Wheels Motor (CM)',relief=tk.SOLID,orient='horizontal')
sliderodometryWheelBaseCm.place(x=10,y=110,width=300, height=50)
sliderodometryWheelBaseCm.bind("<ButtonRelease-1>", updatesliderodometryWheelBaseCm)


"""************* Battery setting *****************************"""

sliderbatGoHomeIfBelow = tk.Scale(tabBattery, from_=20, to=30,resolution=0.1, label='Go Home if Voltage Below ',relief=tk.SOLID,orient='horizontal')
sliderbatGoHomeIfBelow.place(x=10,y=10,width=250, height=50)
sliderbatSwitchOffIfBelow = tk.Scale(tabBattery, from_=20, to=30,resolution=0.1, label='All OFF if Voltage Below ',relief=tk.SOLID,orient='horizontal')
sliderbatSwitchOffIfBelow.place(x=10,y=60,width=250, height=50)
sliderbatSwitchOffIfIdle = tk.Scale(tabBattery, from_=1, to=300, label='All OFF After this Delay in Minute',relief=tk.SOLID,orient='horizontal')
sliderbatSwitchOffIfIdle.place(x=10,y=110,width=250, height=50)
sliderstartChargingIfBelow = tk.Scale(tabBattery, from_=20, to=30,resolution=0.1, label='Start Charging if Voltage Below ',relief=tk.SOLID,orient='horizontal')
sliderstartChargingIfBelow.place(x=10,y=160,width=250, height=50)
sliderbatFullCurrent = tk.Scale(tabBattery, from_=0, to=2,resolution=0.1, label='Start mowing if Charge Current Below ',relief=tk.SOLID,orient='horizontal')
sliderbatFullCurrent.place(x=10,y=210,width=250, height=50)



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

sliderPowerMax = tk.Scale(tabWheelMotor,orient='horizontal',relief=tk.SOLID, from_=0, to=100, label='Power Max in Watt (0 to 100)')
sliderPowerMax.place(x=5,y=10,width=250, height=50)



"""************* Vision setting *****************************"""


#exemple of vision_list=[['person',1,80,15],['chair',52,85,18,]]
#vision_list=[['person',1,80,15],['chair',52,85,18]]
#Read the file and create the list
with open("vision_list.bin","rb") as fp :
        vision_list=pickle.load(fp)
        print("Vision file OK")
        
      
def OnClick_vision(app):
    try:
        itemVision = treeVision.selection()[0]
        txtVisionObject.delete('1.0', 'end')
        txtVisionObject.insert('1.0', vision_list[treeVision.item(itemVision,"text")][0])
        txtVisionID.delete('1.0', 'end')
        txtVisionID.insert('1.0', vision_list[treeVision.item(itemVision,"text")][1])
        txtVisionScore.delete('1.0', 'end')
        txtVisionScore.insert('1.0', vision_list[treeVision.item(itemVision,"text")][2])
        txtVisionArea.delete('1.0', 'end')
        txtVisionArea.insert('1.0', vision_list[treeVision.item(itemVision,"text")][3])
              
    except:
        print("Please click on correct line")


    
#Building the treeVisionView    
treeVision = ttk.Treeview(tabVision)
scrollbarVision1 = ttk.Scrollbar(treeVision,orient="vertical",command=treeVision.yview)
scrollbarVision1.pack(side=tk.RIGHT, fill=tk.Y)
treeVision.configure(yscrollcommand=scrollbarVision1.set)

minwidth = treeVision.column('#0', option='minwidth')
treeVision.column('#0', width=minwidth)

treeVision["columns"]=("0","1","2","3")
treeVision.column("0", width=150)
treeVision.column("1", width=60,anchor='center')
treeVision.column("2", width=60,anchor='center')
treeVision.column("3", width=60,anchor='center')

treeVision.heading("0", text="Name")
treeVision.heading("1", text="Object ID ")
treeVision.heading("2", text=" % Score")
treeVision.heading("3", text=" % Image Area")

treeVision.bind("<ButtonRelease-1>", OnClick_vision)
treeVision.place(x=0, y=0, height=300, width=520)



def rebuild_treeVisionview():
    for i in treeVision.get_children():
        treeVision.delete(i)   
    for i in range(0,len(vision_list)):
        treeVision.insert("" ,'end' ,text=i, values=(vision_list[i][0],vision_list[i][1],vision_list[i][2],vision_list[i][3]))
    

def save_VisionList():
    with open("vision_list.bin","wb") as fp :
        pickle.dump(vision_list,fp)
            
def del_vision_line():
    curr = treeVision.focus()
    if '' == curr: return
    search_object=txtVisionObject.get("1.0",'end-1c')
     
    for i in range(0,len(vision_list)):
        if (str(vision_list[i][0])== str(search_object)):
            for value in vision_list[:]:
                if (value[0] == search_object):
                    vision_list.remove(value)
                    print("remov ok")
                
            break
    
    rebuild_treeVisionview()
        
def add_vision_line(): 
    maVisionList=[txtVisionObject.get("1.0",'end-1c'),txtVisionID.get("1.0",'end-1c'),txtVisionScore.get("1.0",'end-1c'),txtVisionArea.get("1.0",'end-1c')]
    vision_list.append(maVisionList)
    rebuild_treeVisionview()

def test1():
    search_code=(b'dog')
    for i in range(0,len(vision_list)):
        if (str("b'"+vision_list[i][0]+"'")== str(search_code)):
            print("trouv")
            print(vision_list[i])   

tk.Label(tabVision,text="Name:",font=("Arial", 10), fg='green').place(x=530,y=5, height=20, width=80)
tk.Label(tabVision,text="ID:",font=("Arial", 10), fg='green').place(x=530,y=25, height=20, width=80)
tk.Label(tabVision,text="Score:",font=("Arial", 10), fg='green').place(x=530,y=45, height=20, width=80)
tk.Label(tabVision,text="Area:",font=("Arial", 10), fg='green').place(x=530,y=65, height=20, width=80)

txtVisionObject = tk.Text(tabVision)
txtVisionObject.place(x=630,y=5,width=120, height=20)
txtVisionObject.delete('1.0', 'end')
txtVisionObject.insert('1.0', vision_list[0][0])
txtVisionID = tk.Text(tabVision)
txtVisionID.place(x=630,y=25,width=120, height=20)
txtVisionID.delete('1.0', 'end')
txtVisionID.insert('1.0', vision_list[0][1])
txtVisionScore = tk.Text(tabVision)
txtVisionScore.place(x=630,y=45,width=120, height=20)
txtVisionScore.delete('1.0', 'end')
txtVisionScore.insert('1.0', vision_list[0][2])
txtVisionArea = tk.Text(tabVision)
txtVisionArea.place(x=630,y=65,width=120, height=20)
txtVisionArea.delete('1.0', 'end')
txtVisionArea.insert('1.0', vision_list[0][3])


ButtonVisionAdd = tk.Button(tabVision, text="Add ",command = add_vision_line)
ButtonVisionAdd.place(x=530, y=170, height=30, width=110)
ButtonVisionDel = tk.Button(tabVision, text="Delete ",command = del_vision_line)
ButtonVisionDel.place(x=660, y=170, height=30, width=100)
ButtonVisionSave = tk.Button(tabVision, text="Save to File",command = save_VisionList)
ButtonVisionSave.place(x=530, y=210, height=60, width=140)

#ButtonVisionTest = tk.Button(tabVision, text="seek test",command = test1)
#ButtonVisionTest.place(x=530, y=260, height=60, width=140)

rebuild_treeVisionview()



""" THE AUTO PAGE ***************************************************"""


def BtnTextHouseNrLeft_click():
    mymower.House=int(mymower.House)-1
    if (mymower.House <= 0):
        mymower.House=0
        
    textHouseNr.configure(text=mymower.House)
    rebuildHouseMap()
    
def BtnTextHouseNrRight_click():
    mymower.House=int(mymower.House)+1
    if (mymower.House >= 10):
        mymower.House=10
    textHouseNr.configure(text=mymower.House)
    rebuildHouseMap()
    
def rebuildHouseMap():
    mymower.full_house=1
    tk_HouseView.set(1)
    initialPlotAutoPageFullHouse()
    
    

def button_home_click():
    message="AT+C,-1,4,-1,-1,-1,-1,-1,1"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)
    mymower.lastRunningTimer=mymower.ActualRunningTimer
    mymower.ActualRunningTimer = 99
    


def buttonStartMow_click():
    # mow en ,op,speed,timeout,finishandrestart,mowpercent,skipNextMowingPoint,sonar.enabled

    message="AT+C,-1,1,"+str(LinearSpeedSlider.get())+",100,0,"+str(AutoSliderStart.get()/100) +",0,0,208"
    message=str(message)
    message=message + '\r'
    print(message)
    send_serial_message(message)
    

def updatesAutoSliderStart(event):
    mymower.startMowPointIdx=int(mymower.mowPointsCount*AutoSliderStart.get()/100)
    if (mymower.startMowPointIdx<0):
        mymower.startMowPointIdx=0
    if (int(AutoSliderStart.get())>=int(AutoSliderStop.get())):
        AutoSliderStop.set(AutoSliderStart.get())
        mymower.stopMowPointIdx=mymower.startMowPointIdx+1
        tk_labelStopIdx.set(mymower.stopMowPointIdx)
        
    tk_labelStartIdx.set(mymower.startMowPointIdx)
    initialPlotAutoPage(mymower.startMowPointIdx,mymower.stopMowPointIdx)
    
    

def updatesAutoSliderStop(event):
    mymower.stopMowPointIdx=int(mymower.mowPointsCount*int(AutoSliderStop.get())/100)
    tk_labelStopIdx.set(mymower.stopMowPointIdx)
    initialPlotAutoPage(mymower.startMowPointIdx,mymower.stopMowPointIdx)
    
   
def updatesRdBtn_HouseView(event):
    if tk_HouseView.get()==1:
        mymower.full_house=1
        #print("house view")
        initialPlotAutoPageFullHouse()
         
    else:
        mymower.full_house=0
        #print("Map view")
        initialPlotAutoPage(mymower.startMowPointIdx,mymower.stopMowPointIdx)
        
        

    
def updatesRdBtn_Resume(event):
    
    if tk_ResumeMowing.get()==1:
        AutoSliderStart.place_forget()
        AutoPercentLabel.place_forget()
        AutoPercentLabel.text=''
        AutoSliderStart.set(-1)
        AutoSliderStop.set(100)
        
        
        
    else:        
        AutoSliderStart.place(x=430,y=175,width=40, height=170)
        AutoPercentLabel.place(x=445,y=155)        
    mymower.startMowPointIdx=mymower.mowPointsIdx
    initialPlotAutoPage(mymower.startMowPointIdx,mymower.stopMowPointIdx)
    tk_labelStartIdx.set(mymower.mowPointsIdx)
    tk_labelStopIdx.set(mymower.mowPointsCount)
    
   
AutoPage = tk.Frame(fen1)
AutoPage.place(x=0, y=0, height=400, width=800)

batteryFrame= tk.Frame(AutoPage)
batteryFrame.place(x=430, y=10, height=80, width=100)
batteryFrame.configure(borderwidth="3",relief=tk.GROOVE,background="#d9d9d9",highlightbackground="#d9d9d9",highlightcolor="black")

tk.Label(batteryFrame,text="BATTERY",fg='green',font=("Arial", 12)).pack(side='top',anchor='n')
tk.Label(batteryFrame,textvariable=tk_batSense, fg='red',font=("Arial", 12)).pack(side='bottom',anchor='n')
tk.Label(batteryFrame,textvariable=tk_batteryVoltage, fg='red',font=("Arial", 20)).pack(side='bottom',anchor='n')

temperatureFrame= tk.Frame(AutoPage)
temperatureFrame.place(x=430, y=95, height=60, width=100)
temperatureFrame.configure(borderwidth="3",relief=tk.GROOVE,background="#d9d9d9",highlightbackground="#d9d9d9",highlightcolor="black")
tk.Label(temperatureFrame,text="TEMPERATURE",fg='green').pack(side='top',anchor='n')
tk.Label(temperatureFrame,textvariable=tk_PiTemp, fg='red',font=("Arial", 20)).pack(side='bottom',anchor='n')

GpsFrame= tk.Frame(AutoPage)
GpsFrame.place(x=570, y=145, width=100, height=80)
GpsFrame.configure(borderwidth="3",relief=tk.GROOVE,background="#d9d9d9",highlightbackground="#d9d9d9",highlightcolor="black")
tk.Label(GpsFrame,text="GPS",fg='green',font=("Arial", 12)).place(x=25,y=1)
tk.Label(GpsFrame,textvariable=tk_GpsSolution, fg='red',font=("Arial", 20)).pack(side='bottom',anchor='center')
tk.Label(GpsFrame,textvariable=tk_gpsnumSV, fg='black',font=("Arial", 10)).pack(side='bottom',anchor='center')

PosFrame= tk.Frame(AutoPage)
PosFrame.place(x=570, y=228, width=100, height=50)
PosFrame.configure(borderwidth="3",relief=tk.GROOVE,background="#d9d9d9",highlightbackground="#d9d9d9",highlightcolor="black")
tk.Label(PosFrame,text="POS",fg='green',font=("Arial", 12)).place(x=25,y=1)
tk.Label(PosFrame,textvariable=tk_mowPointIdx, fg='black',font=("Arial", 10)).pack(side='bottom',anchor='center')

#creation of the canvas for map view
FrameLiveMap = tk.Frame(AutoPage,borderwidth="1",relief=tk.SOLID)
FrameLiveMap.place(x=5,y=5, height=330, width=360)

figLiveMap,axLiveMap=plt.subplots()
#axLiveMap.set_xlim(left=-30, right=30)
axLiveMap.autoscale(False)

canvasLiveMap = FigureCanvasTkAgg(figLiveMap, master=FrameLiveMap)
canvasLiveMap.get_tk_widget().place(x=0,y=0,width=360, height=330)

axLiveMap.plot(mymower.ActiveMapX,mymower.ActiveMapY, color='r', linewidth=0.4,marker='.',markersize=2)
canvasLiveMap.draw()

fig, ax = plt.subplots()
ax.plot(np.random.rand(10))

def onMapclick(event):
    
    print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
          ('double' if event.dblclick else 'single', event.button,
           event.x, event.y, event.xdata, event.ydata))

    Point_X = event.xdata
    Point_Y = event.ydata
    if (event.dblclick):
        point = geometry.Point(Point_X, Point_Y)
        
        for idx in range(int(len(mymower.mapNrList))):
                mapNr=int(mymower.mapNrList[idx])
                if(mymower.polygon[mapNr].contains(point)):
                    print (mapNr)
                    returnval=messagebox.askyesno('Info',"Upload map "+ str(mapNr) + " to robot")
                    if (returnval):
                        mymower.mapSelected=mapNr
                        export_map_to_mower(mymower.House,mymower.mapSelected)
                        
                        tk_HouseView.set(0)
                        initActiveMap(0)
                        initialPlotAutoPage(0,99999)
                        return
                        


    
    #figLiveMap.canvas.mpl_disconnect(cid)

cid = figLiveMap.canvas.mpl_connect('button_press_event', onMapclick)
toolbarLiveMap = VerticalNavigationToolbar2Tk(canvasLiveMap, AutoPage)
toolbarLiveMap.children['!button2'].pack_forget()
toolbarLiveMap.children['!button3'].pack_forget()
toolbarLiveMap.children['!button4'].pack_forget()

toolbarLiveMap.update()
toolbarLiveMap.place(x=370,y=5,height=150, width=30)



def initActiveMap(full_house):
    #main data
    
    if(full_house==1):
        print("init full map " + str(mymower.full_house))
        
    fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/MAIN.npy"
    
    if (os.path.exists(fileName)):
        maindata=np.load(fileName)
        mymower.perimeterPointsCount=int(maindata[0])
        mymower.exclusionPointsCount=int(maindata[1])
        mymower.dockPointsCount=int(maindata[2])
        mymower.mowPointsCount=int(maindata[3])
        mymower.freePointsCount=int(maindata[4])
        mymower.nbTotalExclusion=int(maindata[5])
        mymower.fileMapCRC=int(maindata[6])
    else:
        messagebox.showwarning('warning',"No main data for this map ???")
        
    fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/PERIMETER.npy"
    if (os.path.exists(fileName)):
        mymower.perimeter=np.load(fileName)      
    else:
        messagebox.showwarning('warning',"No perimeter for this map ???")
        
    fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/MOW.npy"
    if (os.path.exists(fileName)):
        mymower.mowPts=np.load(fileName)
    else:
        messagebox.showwarning('warning',"No mowing points for this map ???")
        
    fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/DOCK.npy"
    if (os.path.exists(fileName)):
        mymower.dockPts=np.load(fileName)


def initialPlotAutoPageFullHouse():
    if (mymower.mapSelected==0):
        return
    mymower.mapNrList.clear
    axLiveMap.clear()
    
    fileName=cwd + "/House" + "{0:0>2}".format(mymower.House)+"/crcMapList.npy"
    mymower.totalMowingArea=0
    if (os.path.exists(fileName)):
        crcMapList=np.load(fileName)
        #print(crcMapList)
        for idx in range(int(len(crcMapList))):
            mapNr=int(crcMapList[idx,0])
            mymower.mapNrList.append(mapNr)
            fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mapNr)+"/PERIMETER.npy"
            if (os.path.exists(fileName)):
                perimeterArray=np.load(fileName)
                polygon1 = Polygon(np.squeeze(perimeterArray))



                
                mymower.polygon[mapNr] = polygon1 #keep the polygon for later search


                #pp3 = plt.Polygon(np.squeeze(perimeterArray))
                #axLiveMap.add_patch(pp3)

                
                #print(str(mapNr)+ " " +str(int(polygon1.area)))
                mymower.totalMowingArea=mymower.totalMowingArea+int(polygon1.area)                #print(polygon1.centroid.coords[0][0]) center of polygon
                #draw perimeter
                x = np.zeros(int(len(perimeterArray)+1))
                y = np.zeros(int(len(perimeterArray)+1))
                for idx1 in range(int(len(perimeterArray))):
                    x[idx1] = perimeterArray[idx1][0]
                    y[idx1] = perimeterArray[idx1][1]       
                      #close the drawing
                x[idx1+1] = perimeterArray[0][0]
                y[idx1+1] = perimeterArray[0][1]
                axLiveMap.text(polygon1.centroid.coords[0][0], polygon1.centroid.coords[0][1], mapNr, fontsize=8, bbox=dict(facecolor='yellow', alpha=0.4))
                axLiveMap.plot(x,y, color='r', linewidth=0.4)#,marker='.',markersize=2)
 
            else:
                print("no perimeter data for this map "+str(mapNr))

            if(mapNr==mymower.mapSelected):
                fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/MOW.npy"

                if (os.path.exists(fileName)):
                    mowPts=np.load(fileName)
                    x = np.zeros(int(len(mowPts)))
                    y = np.zeros(int(len(mowPts)))
                    for ip in range(int(len(mowPts))):
                        x[ip] = mowPts[ip][0]
                        y[ip] = mowPts[ip][1]
                    
                    axLiveMap.plot(x,y, color='black', linewidth=0.2)
        
                else:
                    messagebox.showwarning('warning',"No mowing points for this map ?????")

                
        tk_AutoInfoMap.set("House " + "{0:0>2}".format(mymower.House) + " Total Area " +str(mymower.totalMowingArea)+ " m2")      

    else:
        tk_AutoInfoMap.set("House " + "{0:0>2}".format(mymower.House) + " NO MAP ") 
        
        print("error no crcMapList.npy for this house")

   
    canvasLiveMap.draw()
    #print(mymower.totalMowingArea)
    



    

def initialPlotAutoPage(start,stop):
    if (mymower.mapSelected==0):
        print("no map selected into initialPlotAutoPage")
        return
     
    axLiveMap.clear()
    if (mymower.opNames == "MOW"):
        mymower.startMowPointIdx= mymower.mowPointsIdx
    #reduce len of the array according start and stop
        #bug here
    array_size=int(len(mymower.mowPts)-mymower.startMowPointIdx)#-(len(mymower.mowPts)-mymower.stopMowPointIdx)
    #print(array_size)
    x_lon = np.zeros(array_size)
    y_lat = np.zeros(array_size)
    idx_tab=0
    
    
    for ip in range(int(len(mymower.mowPts))):
        if((ip>=mymower.startMowPointIdx) and (ip<=mymower.stopMowPointIdx)):
            x_lon[idx_tab] = float(mymower.mowPts[ip][0])
            y_lat[idx_tab] = float(mymower.mowPts[ip][1])
            idx_tab=idx_tab+1
    x_lon[x_lon != 0.00] #try to remove 0 value ???????????
    y_lat[y_lat != 0.00]
    #print(x_lon)
    #print(y_lat)
    axLiveMap.plot(x_lon,y_lat, color='black', linewidth=0.2)                 
 

    #draw perimeter
    x_lon = np.zeros(int(len(mymower.perimeter)+1))
    y_lat = np.zeros(int(len(mymower.perimeter)+1))
    for ip in range(int(len(mymower.perimeter))):
        x_lon[ip] = mymower.perimeter[ip][0]
        y_lat[ip] = mymower.perimeter[ip][1]       
      #close the drawing
    x_lon[ip+1] = mymower.perimeter[0][0]
    y_lat[ip+1] = mymower.perimeter[0][1]
    
    #perimeterArray=mymower.map[mymower.mapSelected]
    polygon1 = Polygon(np.squeeze(mymower.perimeter))
    mymower.actualMowingArea=int(polygon1.area)       

    axLiveMap.plot(x_lon,y_lat, color='r', linewidth=0.6)#,marker='.',markersize=2)
    figLiveMap.subplots_adjust(left=0, right=1, top=1, bottom=0)
 
    #draw dock     
    x_lon = np.zeros(int(len(mymower.dockPts)))
    y_lat = np.zeros(int(len(mymower.dockPts)))
    for ip in range(int(len(mymower.dockPts))):
        x_lon[ip] = mymower.dockPts[ip][0]
        y_lat[ip] = mymower.dockPts[ip][1]
    axLiveMap.plot(x_lon,y_lat, color='b', linewidth=0.4)
   # print(str(x_lon[ip])+' station '+str(y_lat[ip]) )
   # print(str(mymower.statex)+' pos     '+str(mymower.statey) )
    
    axLiveMap.scatter(x_lon[ip],y_lat[ip], color='r', s=20)
    axLiveMap.scatter(float(mymower.statex),float(mymower.statey), color='g', s=10)

    tk_AutoInfoMap.set("House " + "{0:0>2}".format(mymower.House) + " Map "+"{0:0>2}".format(mymower.mapSelected)+" Area " +str(mymower.actualMowingArea)+" m2")
   
    canvasLiveMap.draw()
    

tk_AutoInfoMap.set("No Map")
AutoInfoMapLabel=tk.Label(AutoPage, textvariable=tk_AutoInfoMap,font=("Arial", 10))
AutoInfoMapLabel.place(x=150,y=335)

tk_infoTimer.set("")
AutoInfoMapTimer=tk.Label(AutoPage, textvariable=tk_infoTimer,font=("Arial", 10))
AutoInfoMapTimer.place(x=150,y=360)

AutoSliderStart = tk.Scale(AutoPage,orient='vertical', resolution=1 ,font=("Arial", 8) ,from_=-1, to=100)
AutoSliderStart.place(x=430,y=175,width=40, height=170)
AutoSliderStart.set(0)
AutoLabelStartIdx=tk.Label(AutoPage, textvariable=tk_labelStartIdx,font=("Arial", 8)).place(x=453,y=342)

AutoSliderStop = tk.Scale(AutoPage,orient='vertical', resolution=1 ,font=("Arial", 8) ,from_=0, to=100)
AutoSliderStop.place(x=470,y=175,width=40, height=170)
AutoSliderStop.set(100)
AutoLabelStopIdx=tk.Label(AutoPage, textvariable=tk_labelStopIdx,font=("Arial", 8)).place(x=493,y=342)

AutoSliderStart.bind("<ButtonRelease-1>", updatesAutoSliderStart)
AutoSliderStop.bind("<ButtonRelease-1>", updatesAutoSliderStop)
AutoPercentLabel = tk.Label(AutoPage, text='Start  Stop',fg='green')
AutoPercentLabel.place(x=445,y=155)

RdBtn_Resume=tk.Checkbutton(AutoPage, text="Resume last session",font=("Arial", 10), variable=tk_ResumeMowing)
RdBtn_Resume.place(x=515,y=290,width=150, height=20)
RdBtn_Resume.bind("<ButtonRelease-1>", updatesRdBtn_Resume)

RdBtn_HouseView=tk.Checkbutton(AutoPage, text="House view",font=("Arial", 10), variable=tk_HouseView)
RdBtn_HouseView.place(x=5,y=335,width=90, height=15)
RdBtn_HouseView.bind("<ButtonRelease-1>", updatesRdBtn_HouseView)


textHouseNr=tk.Label(AutoPage,borderwidth=1,relief="solid",text=int(mymower.House),font=("Arial", 12), fg='green')
textHouseNr.place(x=34,y=355, height=30, width=40)
BtnTextHouseNrLeft= tk.Button(AutoPage,font=("Arial", 18,'bold'), text="<",command = BtnTextHouseNrLeft_click)
BtnTextHouseNrLeft.place(x=5, y=355, height=30, width=30)
BtnTextHouseNrRight= tk.Button(AutoPage,font=("Arial", 18,'bold'), text=">",command = BtnTextHouseNrRight_click)
BtnTextHouseNrRight.place(x=73, y=355, height=30, width=30)



ButtonStartMow = tk.Button(AutoPage, image=imgstartMow, command = buttonStartMow_click)
ButtonStartMow.place(x=570,y=10,width=100, height=130)
Buttonhome = tk.Button(AutoPage, image=imgHome, command = button_home_click)
Buttonhome.place(x=680,y=145,width=100, height=130)
ButtonStopAllAuto = tk.Button(AutoPage, image=imgStopAll, command = button_stop_all_click)
ButtonStopAllAuto.place(x=680,y=10,width=100, height=130)


ButtonBackHome = tk.Button(AutoPage, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=120, width=120)






"""**************************THE MANUAL PAGE  **********************************************"""


    

    

def ButtonJoystickON_click() :
    subprocess.call(["rfkill","unblock","bluetooth"])
    print ("BlueTooth Start")
    returnval=messagebox.askyesno('Info',"Power ON the joystick now and wait until the led is fix")
    if returnval :
        try:
            #if not pygame.joystick.get_init():
            #print(myps4.get_init())
            myps4.init()
            myps4.listen()
            mymower.useJoystick=True
            #print(myps4.get_init())
        except:
            subprocess.call(["rfkill","block","bluetooth"])
            returnval=messagebox.showerror('Error',"The joystick is not found: Please try to connect under PI GUI First")

def ButtonJoystickOFF_click() :
    subprocess.call(["rfkill","block","bluetooth"])
    print ("BlueTooth Stop")
    mymower.useJoystick=False
    messagebox.showinfo('Info',"Bluetooth and joystick are OFF")
        
def buttonBlade_stop_click():
    message="AT+C,0,0,-1,-1,-1,-1,-1,-1"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)  
    
def buttonBlade_start_click():
    message="AT+C,1,0,-1,-1,-1,-1,-1,-1"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)  
    

def ButtonForward_click():
    ButtonReverse.configure(state='disabled')
    message="AT+M," + str(manualSpeedSlider.get()) + ",0,0"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)

def ButtonForwardLeft_click():
    ButtonReverse.configure(state='disabled')
    message="AT+M," + str(manualSpeedSlider.get()) + "," + str(manualSpeedSlider.get()) + ",0"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)

def ButtonForwardRight_click():
    ButtonReverse.configure(state='disabled')
    message="AT+M," + str(manualSpeedSlider.get()) + "," + str(-1*manualSpeedSlider.get()) + ",0"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)
    
    
def ButtonRight_click():
    ButtonReverse.configure(state='disabled')
    message="AT+M," + str(0.3*manualSpeedSlider.get()) + "," + str(-1*manualSpeedSlider.get()) + ",0"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)
    
def ButtonLeft_click():
    ButtonReverse.configure(state='disabled')
    message="AT+M," + str(0.3*manualSpeedSlider.get()) + "," + str(manualSpeedSlider.get()) + ",0"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)
    
def ButtonReverse_click():
    message="AT+M,-" + str(manualSpeedSlider.get()) + ",0"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)
    
def ButtonStop_click():
    ButtonReverse.configure(state='active')
    message="AT+M,0,0"
    message=str(message)
    message=message + '\r'
    send_serial_message(message) 
     
    
ManualPage = tk.Frame(fen1)
ManualPage.place(x=0, y=0, height=400, width=800)
Frame1 = tk.Frame(ManualPage)
Frame1.place(x=0, y=0, height=300, width=300)


ButtonForward = tk.Button(Frame1,image=imgForward, command = ButtonForward_click,repeatdelay=500, repeatinterval=500)
ButtonForward.place(x=100, y=0, height=100, width=100)

ButtonForwardLeft = tk.Button(Frame1,image=imgForwardLeft, command = ButtonForwardLeft_click,repeatdelay=500, repeatinterval=500)
ButtonForwardLeft.place(x=0, y=0, height=100, width=100)

ButtonForwardRight = tk.Button(Frame1,image=imgForwardRight, command = ButtonForwardRight_click,repeatdelay=500, repeatinterval=500)
ButtonForwardRight.place(x=200, y=0, height=100, width=100)

ButtonStop = tk.Button(Frame1,image=imgStop, command = ButtonStop_click)
ButtonStop.place(x=100, y=100, height=100, width=100)

ButtonRight = tk.Button(Frame1,image=imgRight, command = ButtonRight_click,repeatdelay=500, repeatinterval=500)
ButtonRight.place(x=200, y=100, height=100, width=100)

ButtonLeft = tk.Button(Frame1,image=imgLeft, command = ButtonLeft_click,repeatdelay=500, repeatinterval=500)
ButtonLeft.place(x=0, y=100, height=100, width=100)

ButtonReverse = tk.Button(Frame1,image=imgReverse,command = ButtonReverse_click)
ButtonReverse.place(x=100,y=200, height=100, width=100)

ButtonBladeStart = tk.Button(ManualPage, image=imgBladeStart, command = buttonBlade_start_click)
ButtonBladeStart.place(x=680,y=5,width=100, height=50)
ButtonBladeStop = tk.Button(ManualPage, image=imgBladeStop, command = buttonBlade_stop_click)
ButtonBladeStop.place(x=680,y=55,width=100, height=80)



ButtonStopAllManual = tk.Button(ManualPage, image=imgStopAll, command = button_stop_all_click)
ButtonStopAllManual.place(x=400,y=200,width=100, height=130)

tk.Label(ManualPage, image=imgJoystick).place(x=400,y=5)
ButtonJoystickON = tk.Button(ManualPage, image=imgJoystickON, command = ButtonJoystickON_click)
ButtonJoystickON.place(x=450,y=80,width=50, height=50)
ButtonJoystickOFF = tk.Button(ManualPage, image=imgJoystickOFF, command = ButtonJoystickOFF_click)
ButtonJoystickOFF.place(x=400,y=80,width=50, height=50)

RdBtn_keyboard=tk.Checkbutton(ManualPage, text="Use Keyboard to drive the mower", variable=ManualKeyboardUse).place(x=0,y=365)



manualSpeedSlider = tk.Scale(ManualPage,orient='horizontal', resolution=0.1 ,from_=0, to=0.8)
manualSpeedSlider.place(x=0,y=300,width=300, height=60)
manualSpeedSlider.set(0.2)
tk.Label(ManualPage, text='SPEED',fg='green').place(x=0,y=300)

"""
slider1 = tk.Scale(orient='horizontal', from_=0, to=350)
slider1.place(x=50,y=50,anchor='nw',width=300, height=50)
slider2 = tk.Scale(orient='horizontal', from_=0, to=100)
slider2.place(x=50,y=100,anchor='nw',width=300, height=50)
"""
ButtonBackHome = tk.Button(ManualPage, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=120, width=120)



""" The Console page  ************************************"""

def ButtonListVar_click():
 
    fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps02"+"/DOCK.npy"
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
            crc_mow=0.0
            mowPts=np.load(fileName)
            mowPts_count=int(len(mowPts))
            print(mowPts)
            
    fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps04"+"/DOCK.npy"
    if (os.path.exists(fileName)):
            crc_mow=0.0
            mowPts1=np.load(fileName)
            mowPts1_count=int(len(mowPts1))
            print(mowPts1)

    for ip in range(int(len(mowPts))):
       # print(mowPts[ip][0], " / " , mowPts1[ip][0])

        
        if ((100*float(mowPts[ip][0])) != (100*float(mowPts1[ip][0]))) :
            print("erreur x :", ip ,(mowPts[ip][0]) ,(mowPts1[ip][0]) )

        if ((100*float(mowPts[ip][1])) != (100*float(mowPts1[ip][1]))) :
            print("erreur y :", ip ,100*float(mowPts[ip][1]) ,100*float(mowPts1[ip][1]) )
    print("fini")


def ButtonConsoleMode_click():
    message="AT+Y2,0"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)  
    

def ButtonClearConsole_click():
    
    txtRecu.delete('1.0',tk.END) 
    txtSend.delete('1.0',tk.END)
    txtConsoleRecu.delete('1.0',tk.END) 
        

    

ConsolePage = tk.Frame(fen1)
ConsolePage.place(x=0, y=0, height=400, width=800)


txtRecu = tk.Text(ConsolePage)
ScrollTxtRecu = tk.Scrollbar(txtRecu)
ScrollTxtRecu.pack(side=tk.RIGHT, fill=tk.Y)
txtRecu.pack(side=tk.LEFT, fill=tk.Y)
ScrollTxtRecu.config(command=txtRecu.yview)
txtRecu.config(yscrollcommand=ScrollTxtRecu.set)
txtRecu.place(x=0,y=300,anchor='nw',width=480, height=100)

txtSend = tk.Text(ConsolePage)
ScrollTxtSend = tk.Scrollbar(txtSend)
ScrollTxtSend.pack(side=tk.RIGHT, fill=tk.Y)
txtSend.pack(side=tk.LEFT, fill=tk.Y)
ScrollTxtSend.config(command=txtSend.yview)
txtSend.config(yscrollcommand=ScrollTxtSend.set)
txtSend.place(x=490,y=300,anchor='nw',width=300, height=100)

txtConsoleRecu = tk.Text(ConsolePage)
ScrolltxtConsoleRecu = tk.Scrollbar(txtConsoleRecu)
ScrolltxtConsoleRecu.pack(side=tk.RIGHT, fill=tk.Y)
txtConsoleRecu.pack(side=tk.LEFT, fill=tk.Y)
ScrolltxtConsoleRecu.config(command=txtConsoleRecu.yview)
txtConsoleRecu.config(yscrollcommand=ScrolltxtConsoleRecu.set)
txtConsoleRecu.place(x=0,y=5,anchor='nw',width=800, height=290)


ButtonConsoleMode = tk.Button(ConsolePage)
ButtonConsoleMode.place(x=660,y=15, height=25, width=120)
ButtonConsoleMode.configure(command = ButtonConsoleMode_click,text="GPS reboot")

##ButtonListVar = tk.Button(ConsolePage)
##ButtonListVar.configure(command = ButtonListVar_click,text="List Var")
##ButtonListVar.place(x=660,y=45, height=25, width=120)

ButtonClearConsole = tk.Button(ConsolePage)
ButtonClearConsole.place(x=660,y=75, height=35, width=120)
ButtonClearConsole.configure(command = ButtonClearConsole_click,text="Clear Console")

ButtonSaveReceived = tk.Button(ConsolePage)
ButtonSaveReceived.place(x=660,y=120, height=35, width=120)
ButtonSaveReceived.configure(command = ButtonSaveReceived_click,text="Save To File")





ButtonBackHome = tk.Button(ConsolePage, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=660, y=160, height=120, width=120)

""" THE PLOT PAGE ***************************************************"""

TabPlot=ttk.Notebook(fen1)
tabPlotMain=tk.Frame(TabPlot,width=800,height=400)
tabPlotWheelMotor=tk.Frame(TabPlot,width=800,height=200)
tabPlotMowMotor=tk.Frame(TabPlot,width=800,height=200)
tabPlotPerimeter=tk.Frame(TabPlot,width=800,height=200)
tabPlotBattery=tk.Frame(TabPlot,width=800,height=200)
tabPlotImu=tk.Frame(TabPlot,width=800,height=200)

TabPlot.add(tabPlotMain,text="Main")
TabPlot.add(tabPlotWheelMotor,text="Wheels Motor")
TabPlot.add(tabPlotMowMotor,text="Mow Motor")
TabPlot.add(tabPlotPerimeter,text="Perimeter")
TabPlot.add(tabPlotBattery,text="Battery")
TabPlot.add(tabPlotImu,text="Imu")

TabPlot.place(x=0, y=0, height=400, width=800)

#'Main
Frame11= tk.Frame(tabPlotMain,relief=tk.GROOVE,borderwidth="3")
Frame11.place(x=10, y=20, height=80, width=680)

BtnMotPlotStopAll= tk.Button(Frame11,command = BtnMotPlotStopAll_click,text="Stop ALL the Data send by DUE")
BtnMotPlotStopAll.place(x=10,y=10, height=25, width=250)

#'Wheel Motor
tk.Label(tabPlotWheelMotor, text="Mower Millis : ").place(x=300,y=0)
tk.Label(tabPlotWheelMotor, textvariable=tk_millis).place(x=400,y=0)
Frame12= tk.Frame(tabPlotWheelMotor,relief=tk.GROOVE,borderwidth="3")
Frame12.place(x=10, y=20, height=80, width=680)
BtnMotPlotStartRec= tk.Button(Frame12,command = BtnMotPlotStartRec_click,text="Start")
BtnMotPlotStartRec.place(x=0,y=0, height=25, width=60)
BtnMotPlotStopRec= tk.Button(Frame12,command = BtnMotPlotStopRec_click,text="Stop")
BtnMotPlotStopRec.place(x=0,y=25, height=25, width=60)
SldMainWheelRefresh = tk.Scale(Frame12, from_=1, to=10, label='Refresh Rate per second',relief=tk.SOLID,orient='horizontal')
SldMainWheelRefresh.place(x=70,y=0,width=250, height=50)


tk.Label(Frame12, text='Power',fg='green').place(x=400,y=0)
tk.Label(Frame12, text='Left',fg='green').place(x=350,y=15)
tk.Label(Frame12, textvariable=tk_motorLeftPower).place(x=400,y=15)
tk.Label(Frame12, text='Right',fg='green').place(x=350,y=35)
tk.Label(Frame12, textvariable=tk_motorRightPower).place(x=400,y=35)
tk.Label(Frame12, text='PWM',fg='green').place(x=550,y=0)
tk.Label(Frame12, textvariable=tk_motorLeftPWMCurr).place(x=550,y=15)
tk.Label(Frame12, textvariable=tk_motorRightPWMCurr).place(x=550,y=35)

#'Mow Motor
tk.Label(tabPlotMowMotor, text="Mower Millis : ").place(x=300,y=0)
tk.Label(tabPlotMowMotor, textvariable=tk_millis).place(x=400,y=0)
Frame13= tk.Frame(tabPlotMowMotor,relief=tk.GROOVE,borderwidth="3")
Frame13.place(x=10, y=20, height=80, width=680)
BtnMowPlotStartRec= tk.Button(Frame13,command = BtnMowPlotStartRec_click,text="Start")
BtnMowPlotStartRec.place(x=0,y=0, height=25, width=60)
BtnMowPlotStopRec= tk.Button(Frame13,command = BtnMowPlotStopRec_click,text="Stop")
BtnMowPlotStopRec.place(x=0,y=25, height=25, width=60)
SldMainMowRefresh = tk.Scale(Frame13, from_=1, to=10, label='Refresh Rate per second',relief=tk.SOLID,orient='horizontal')
SldMainMowRefresh.place(x=70,y=0,width=250, height=50)

tk.Label(Frame13, text='Power',fg='green').place(x=400,y=0)
tk.Label(Frame13, textvariable=tk_motorMowPower).place(x=400,y=15)
tk.Label(Frame13, text='PWM',fg='green').place(x=550,y=0)
tk.Label(Frame13, textvariable=tk_motorMowPWMCurr).place(x=550,y=15)

#'Battery'
tk.Label(tabPlotBattery, text="Mower Millis : ").place(x=300,y=0)
tk.Label(tabPlotBattery, textvariable=tk_millis).place(x=400,y=0)
Frame14= tk.Frame(tabPlotBattery,relief=tk.GROOVE,borderwidth="3")
Frame14.place(x=10, y=20, height=80, width=680)
BtnBatPlotStartRec= tk.Button(Frame14,command = BtnBatPlotStartRec_click,text="Start")
BtnBatPlotStartRec.place(x=0,y=0, height=25, width=60)
BtnBatPlotStopRec= tk.Button(Frame14,command = BtnBatPlotStopRec_click,text="Stop")
BtnBatPlotStopRec.place(x=0,y=25, height=25, width=60)
SldMainBatRefresh = tk.Scale(Frame14, from_=1, to=100, label='Refresh Rate per minute',relief=tk.SOLID,orient='horizontal')
SldMainBatRefresh.place(x=70,y=0,width=250, height=50)

tk.Label(Frame14, text='Charge',fg='green').place(x=350,y=15)
tk.Label(Frame14, text='Battery',fg='green').place(x=350,y=35)
tk.Label(Frame14, text='Sense',fg='green').place(x=400,y=0)
tk.Label(Frame14, textvariable=tk_chgSense).place(x=400,y=15)
tk.Label(Frame14, text='Voltage',fg='green').place(x=550,y=0)
tk.Label(Frame14, textvariable=tk_chgVoltage).place(x=550,y=15)
tk.Label(Frame14, textvariable=tk_batteryVoltage).place(x=550,y=35)


#'Perimeter'
tk.Label(tabPlotPerimeter, text="Mower Millis : ").place(x=300,y=0)
tk.Label(tabPlotPerimeter, textvariable=tk_millis).place(x=400,y=0)
Frame15= tk.Frame(tabPlotPerimeter,relief=tk.GROOVE,borderwidth="3")
Frame15.place(x=10, y=20, height=80, width=680)
BtnPeriPlotStartRec= tk.Button(Frame15,command = BtnPeriPlotStartRec_click,text="Start")
BtnPeriPlotStartRec.place(x=0,y=0, height=25, width=60)
BtnPeriPlotStopRec= tk.Button(Frame15,command = BtnPeriPlotStopRec_click,text="Stop")
BtnPeriPlotStopRec.place(x=0,y=25, height=25, width=60)
SldMainPeriRefresh = tk.Scale(Frame15, from_=1, to=10, label='Refresh Rate per second',relief=tk.SOLID,orient='horizontal')
SldMainPeriRefresh.place(x=70,y=0,width=250, height=50)

tk.Label(Frame15, text='Mag',fg='green').place(x=400,y=0)
tk.Label(Frame15, text='Left',fg='green').place(x=350,y=15)
tk.Label(Frame15, textvariable=tk_perimeterMag).place(x=400,y=15)
tk.Label(Frame15, text='Right',fg='green').place(x=350,y=35)
tk.Label(Frame15, textvariable=tk_perimeterMagRight).place(x=400,y=35)

#'Imu'
tk.Label(tabPlotImu, text="Mower Millis : ").place(x=0,y=200)
tk.Label(tabPlotImu, textvariable=tk_millis).place(x=100,y=200)
Frame16= tk.Frame(tabPlotImu,relief=tk.GROOVE,borderwidth="3")
Frame16.place(x=5, y=5, height=155, width=390)
BtnImuPlotStartRec= tk.Button(Frame16,command = BtnImuPlotStartRec_click,text="Start")
BtnImuPlotStartRec.place(x=0,y=0, height=25, width=60)
BtnImuPlotStopRec= tk.Button(Frame16,command = BtnImuPlotStopRec_click,text="Stop")
BtnImuPlotStopRec.place(x=0,y=25, height=25, width=60)
SldMainImuRefresh = tk.Scale(Frame16, from_=1, to=10, label='Refresh Rate per seconde',relief=tk.SOLID,orient='horizontal')
SldMainImuRefresh.place(x=70,y=0,width=250, height=50)

tk.Label(Frame16, text='Gyro',fg='green').place(x=50,y=75)
tk.Label(Frame16, text='Compass',fg='green').place(x=50,y=95)
tk.Label(Frame16, text='Value',fg='green').place(x=160,y=60)
tk.Label(Frame16, textvariable=tk_gyroYaw).place(x=160,y=75)
tk.Label(Frame16, textvariable=tk_compassYaw).place(x=160,y=95)

ButtonBackHome = tk.Button(TabPlot, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=120, width=120)


"""
 THE INFO PAGE ***************************************************
"""
InfoPage = tk.Frame(fen1)
InfoPage.place(x=0, y=0, height=400, width=800)
Infoline1=tk.StringVar()
LabInfoline = tk.Label(InfoPage, textvariable=Infoline1)
LabInfoline.place(x=10,y=10, height=25, width=450)
Infoline2=tk.IntVar()
LabInfoline = tk.Label(InfoPage, textvariable=Infoline2)
LabInfoline.place(x=10,y=40, height=25, width=300)
Infoline3=tk.IntVar()
LabInfoline = tk.Label(InfoPage, textvariable=Infoline3)
LabInfoline.place(x=10,y=70, height=25, width=300)
Infoline4=tk.IntVar()
LabInfoline = tk.Label(InfoPage, textvariable=Infoline4)
LabInfoline.place(x=10,y=100, height=25, width=300)
Infoline5=tk.IntVar()
LabInfoline = tk.Label(InfoPage, textvariable=Infoline5)
LabInfoline.place(x=10,y=130, height=25, width=300)

FrameGpsInfo = tk.Frame(InfoPage)
FrameGpsInfo.place(x=10, y=180, height=150, width=520)
FrameGpsInfo.configure(borderwidth="3",relief=tk.GROOVE,background="#d9d9d9",highlightbackground="#d9d9d9",highlightcolor="black")

GpsInfoline1=tk.StringVar()
LabInfoline = tk.Label(FrameGpsInfo, textvariable=GpsInfoline1)
LabInfoline.place(x=10,y=10, height=25, width=150)

GpsInfoline2=tk.StringVar()
LabInfoline = tk.Label(FrameGpsInfo, textvariable=GpsInfoline2)
LabInfoline.place(x=10,y=35, height=25, width=150)

GpsInfoline3=tk.StringVar()
LabInfoline = tk.Label(FrameGpsInfo, textvariable=GpsInfoline3)
LabInfoline.place(x=10,y=60, height=25, width=150)

GpsInfoline4=tk.StringVar()
LabInfoline = tk.Label(FrameGpsInfo, textvariable=GpsInfoline4)
LabInfoline.place(x=10,y=85, height=25, width=150)

GpsInfoline5=tk.StringVar()
LabInfoline = tk.Label(FrameGpsInfo, textvariable=GpsInfoline5)
LabInfoline.place(x=10,y=110, height=25, width=150)

GpsInfoline6=tk.StringVar()
LabInfoline = tk.Label(FrameGpsInfo, textvariable=GpsInfoline6)
LabInfoline.place(x=160,y=10, height=25, width=150)

GpsInfoline7=tk.StringVar()
LabInfoline = tk.Label(FrameGpsInfo, textvariable=GpsInfoline7)
LabInfoline.place(x=160,y=35, height=25, width=150)

GpsInfoline8=tk.StringVar()
LabInfoline = tk.Label(FrameGpsInfo, textvariable=GpsInfoline8)
LabInfoline.place(x=160,y=60, height=25, width=150)

GpsInfoline9=tk.StringVar()
LabInfoline = tk.Label(FrameGpsInfo, textvariable=GpsInfoline9)
LabInfoline.place(x=160,y=85, height=25, width=150)


GpsInfoline10=tk.StringVar()
LabInfoline = tk.Label(FrameGpsInfo, textvariable=GpsInfoline10)
LabInfoline.place(x=160,y=110, height=25, width=150)

GpsInfoline11=tk.StringVar()
LabInfoline = tk.Label(FrameGpsInfo, textvariable=GpsInfoline11)
LabInfoline.place(x=310,y=10, height=25, width=150)





ButtonBackHome = tk.Button(InfoPage, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=120, width=120)



""" THE MAPS PAGE ***************************************************"""





def showFullMapTab(): #tab 0 show the full map
    ax[0].clear()
    fileName=cwd + "/House" + "{0:0>2}".format(mymower.House)+"/crcMapList.npy"
    mymower.totalMowingArea=0
    if (os.path.exists(fileName)):
        crcMapList=np.load(fileName)
        
        for idx in range(int(len(crcMapList))):
            mapNr=int(crcMapList[idx,0])
           
            
            fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mapNr)+"/PERIMETER.npy"
            if (os.path.exists(fileName)):
                perimeterArray=np.load(fileName)
                polygon1 = Polygon(np.squeeze(perimeterArray))
                mymower.polygon[mapNr] = polygon1 #keep the polygon for later search
                mymower.totalMowingArea=mymower.totalMowingArea+int(polygon1.area)                #print(polygon1.centroid.coords[0][0]) center of polygon
                #draw perimeter
                x = np.zeros(int(len(perimeterArray)+1))
                y = np.zeros(int(len(perimeterArray)+1))
                for idx1 in range(int(len(perimeterArray))):
                    x[idx1] = perimeterArray[idx1][0]
                    y[idx1] = perimeterArray[idx1][1]       
                      #close the drawing
                x[idx1+1] = perimeterArray[0][0]
                y[idx1+1] = perimeterArray[0][1]
                ax[0].text(polygon1.centroid.coords[0][0], polygon1.centroid.coords[0][1], mapNr, fontsize=8, bbox=dict(facecolor='yellow', alpha=0.4))
                ax[0].plot(x,y, color='r', linewidth=0.4)#,marker='.',markersize=2)
 
            else:
                print("no perimeter data for this map "+str(mapNr))

       
        
            #draw exclusion

            fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mapNr)+"/MAIN.npy"
            nbTotalExclusion1=0
    
            if (os.path.exists(fileName)):
                maindata1=np.load(fileName)
                nbTotalExclusion1=int(maindata1[5])
                
            else:
                print("no main.npy for this house")

                
        
            for mymower.exclusionNr in range(nbTotalExclusion1):
                fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mapNr)+"/EXCLUSION"+"{0:0>2}".format(mymower.exclusionNr)+".npy"
              
                if (os.path.exists(fileName)):
                    excluPts=np.load(fileName)
                    polygon1 = Polygon(np.squeeze(excluPts))
                    #mymower.polygon[mapNr] = polygon1 #keep the polygon for later search
                    mymower.totalMowingArea=mymower.totalMowingArea-int(polygon1.area) 
                    x_lon = np.zeros(int(len(excluPts)+1))
                    y_lat = np.zeros(int(len(excluPts)+1))
                    if (int(len(excluPts)) >=3):
                        for ip in range(int(len(excluPts))):
                            x_lon[ip] = excluPts[ip][0]
                            y_lat[ip] = excluPts[ip][1]
                        x_lon[ip+1] = excluPts[0][0]
                        y_lat[ip+1] = excluPts[0][1]
                    ax[0].plot(x_lon,y_lat, color='r', linewidth=1)
            
                else:
                    messagebox.showwarning('warning',"No exclusion points for this map ?????")
             #draw dock
            dockCRC=0
            fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mapNr)+"/DOCK.npy"
            if (os.path.exists(fileName)):
                dockPts=np.load(fileName)
                x_lon = np.zeros(int(len(dockPts)))
                y_lat = np.zeros(int(len(dockPts)))
                for ip in range(int(len(dockPts))):
                    x_lon[ip] = dockPts[ip][0]
                    y_lat[ip] = dockPts[ip][1]
                    
                ax[0].plot(x_lon,y_lat, color='b', linewidth=0.6)
        
            else:
                messagebox.showwarning('warning',"No dock points for this map ?????")
                

    else:
        print("error no crcMapList.npy for this house")


    canvas[0].draw()
   
    MapsInfoline1.configure(text=" Total Area " +str(mymower.totalMowingArea)+ " m2")
##    InfoHouseNrtxt.configure(text=mymower.House)
##    InfoHouseNrtxt.update()



    

def onTabChange(event):
    mymower.mapSelected = int(MapsPage.index("current"))
    if (mymower.mapSelected == 0):
        ButtonDeleteMap.place_forget()
        ButtonExportMap.place_forget()
        ButtonImportMap.place_forget()
        showFullMapTab()
        return
    else:
         ButtonDeleteMap.place(x=680,y=50,height=60, width=110)
         ButtonExportMap.place(x=680,y=115,height=60, width=110)
         ButtonImportMap.place(x=680,y=180,height=60, width=110)

       

    fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/MAIN.npy"      
    if (os.path.exists(fileName)):
        maindata=np.load(fileName)
        mymower.perimeterPointsCount=int(maindata[0])
        mymower.exclusionPointsCount=int(maindata[1])
        mymower.dockPointsCount=int(maindata[2])
        mymower.mowPointsCount=int(maindata[3])
        mymower.freePointsCount=int(maindata[4])
        mymower.nbTotalExclusion=int(maindata[5])
        mymower.fileMapCRC=int(maindata[6])
        Infolinetxt=""
        Infolinetxt=Infolinetxt+ "Perimeter points: " + str(mymower.perimeterPointsCount) + '\n'
        Infolinetxt=Infolinetxt+ "Nb Exclusions: " + str(mymower.nbTotalExclusion) + '\n'
        Infolinetxt=Infolinetxt+ "Exclusion points: " + str(mymower.exclusionPointsCount) + '\n'
        Infolinetxt=Infolinetxt+ "Mow points: " + str(mymower.mowPointsCount) + '\n'
        Infolinetxt=Infolinetxt+ "Dock points: " + str(mymower.dockPointsCount) + '\n'
        Infolinetxt=Infolinetxt+ "Free points: " + str(mymower.freePointsCount) + '\n'
        Infolinetxt=Infolinetxt+ "File CRC: " + str(mymower.fileMapCRC) + '\n'
        MapsInfoline1.configure(text=Infolinetxt)
        InfoHouseNrtxt.configure(text=mymower.House)
        InfoHouseNrtxt.update()
        plot()
        toolbar = VerticalNavigationToolbar2Tk(canvas[mymower.mapSelected], MapsPage)
        #toolbar.update()
        toolbar.place(x=5,y=40)
        
    else:
        Infolinetxt=""
        MapsInfoline1.configure(text=Infolinetxt)
        InfoHouseNrtxt.configure(text=mymower.House)
        InfoHouseNrtxt.update()

def delete_map():
     
    returnval=messagebox.askyesno('Info',"Do you want to remove the MAP"+"{0:0>2}".format(mymower.mapSelected))
    if returnval :
        dir_path=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mymower.mapSelected)
        try:
            for filepath in os.listdir(dir_path):
                os.remove(dir_path +  "/{}".format(filepath))
            os.rmdir(dir_path)
        except:
            print("no file")

        #update the main CRC datalist according to map nr
        fileName=cwd + "/House" + "{0:0>2}".format(mymower.House)+"/crcMapList.npy"
        if (os.path.exists(fileName)):
            crcMapList=np.load(fileName)
            print(fileName)
            print(crcMapList)
            find_row=np.where(crcMapList[:,0] == mymower.mapSelected)
            result=crcMapList[find_row]             
            if (result.size != 0):                                                  
                    if (result[0,0]==mymower.mapSelected):                    
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
            
        consoleInsertText("Map " +"{0:0>2}".format(mymower.mapSelected) + ' is remove from Pi \n')   
        messagebox.showinfo('info',"Map " +"{0:0>2}".format(mymower.mapSelected) + ' is remove from Pi')
        print(mymower.mapSelected)
        mymower.mapCRC=0
        ax[mymower.mapSelected].clear()
        fig[mymower.mapSelected].clear()
        canvas[mymower.mapSelected].draw()
        #onTabChange(mymower.mapSelected)
        #mymower.mapSelected=0
        
def wait_the_feedBack():
    mymower.isSendindMap=True
    while (mymower.isSendindMap):
        time.sleep(0.1)
        
        print(" wait feed back \n" )
        if (Due_Serial.inWaiting() != 0) :
            mymower.dueSerialReceived=Due_Serial.readline()
            if str(mymower.dueSerialReceived)!="b''":
                mymower.dueSerialReceived = mymower.dueSerialReceived.decode('utf-8', errors='ignore')
                if mymower.dueSerialReceived[:1] != '$' : #it is console message because the first digit is not $
                    if(len(mymower.dueSerialReceived))>2:
                        decode_AT_message(mymower.dueSerialReceived)
                else :
                    consoleInsertText("recu sentense starting by $ "+ '\n')
                    consoleInsertText(mymower.dueSerialReceived)
                    

def ButtonExportMap_click():
    returnval=messagebox.askyesno('Info',"You are going to replace the MAP locate into mower by this one")
    if (returnval):
        export_map_to_mower(mymower.House,mymower.mapSelected)
    
    
def export_map_to_mower(House,mapNr):#export the into mower see decode AT message on RP
    if(mapNr == 0):
        print("map 0 can't be exported to mower")
        return
    mymower.House=House
    mymower.mapSelected=mapNr
    mymower.finishedUploadingMap=False
    #read the main data of the map

    fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/MAIN.npy"      
    if (os.path.exists(fileName)):
        maindata=np.load(fileName)
        mymower.perimeterPointsCount=int(maindata[0])
        mymower.exclusionPointsCount=int(maindata[1])
        mymower.dockPointsCount=int(maindata[2])
        mymower.mowPointsCount=int(maindata[3])
        mymower.freePointsCount=int(maindata[4])
        mymower.nbTotalExclusion=int(maindata[5])
        mymower.fileMapCRC=int(maindata[6])
    else:
        print("Error can't find the main data for this map " + str(mapNr))
        return

    mymower.isSendindMap=True
    mymower.startMowPointIdx=0

    #part 1 AT+W map data waypoint
   
    trame_size=30  #default value=30
    pt=0
    pt_total=0
    crc_point=0
    
    ################ perimeter
    print("perimeter")
    crc_perimeter=0
    fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                      "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/PERIMETER.npy"
    if (os.path.exists(fileName)):
        crc_perimeter=0
        perimeterPts=np.load(fileName)
        perimeterPts_count=int(len(perimeterPts))
        if((mymower.perimeterPointsCount)!=perimeterPts_count):
            print("Error in the perimeter point count")
        else:
            print("Nb_point : " , perimeterPts_count)

        
        toSend="AT+W,0,"
                        
        for ip in range(int(len(perimeterPts))):
            toSend=toSend+str(perimeterPts[ip][0])+","+str(perimeterPts[ip][1])+","
            crc_perimeter=crc_perimeter + int(100*(perimeterPts[ip][0])) + int(100*(perimeterPts[ip][1]))
            pt=pt+1
            pt_total=pt_total+1
            if (pt>=trame_size) :
                toSend=toSend + '\r'
                print(toSend)
                consoleInsertText(" Send " + toSend )
                send_serial_message(toSend)
                wait_the_feedBack()
                #prepare the new sentence
                toSend="AT+W,"+str(pt_total) +","                        
                pt=0
                #crc_point=0
    else:
        messagebox.showwarning('warning',"No perimeter for this map ???")
    consoleInsertText("Perimeter points send finish CRC: " + str(crc_perimeter) + '\n')
    print("fin de perimeter")

    ################ exclusion
    
    crc_exclusion=0
    for mymower.exclusionNr in range(int(mymower.nbTotalExclusion)):
        print("exclusion : " +str(mymower.exclusionNr))
        fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                      "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/EXCLUSION"+"{0:0>2}".format(mymower.exclusionNr)+".npy"
          
        if (os.path.exists(fileName)):
            crc_exclusion=0
            exclusionPts=np.load(fileName)
        
            for ip in range(int(len(exclusionPts))):
                toSend=toSend+str(exclusionPts[ip][0])+","+str(exclusionPts[ip][1])+","
                crc_exclusion=crc_exclusion + int(100*(exclusionPts[ip][0])) + int(100*(exclusionPts[ip][1]))
                pt=pt+1
                pt_total=pt_total+1
                if (pt>=trame_size) :
                    toSend=toSend + '\r'
                    print(toSend)
                    send_serial_message(toSend)
                    time.sleep(1)
                    checkSerial()
                    #prepare the new sentence
                    toSend="AT+W,"+str(pt_total) +","                        
                    pt=0
                    
                
        else:
            messagebox.showwarning('warning',"No exclusion for this map ???")
            
        consoleInsertText("Exclusion points send finish CRC: " + str(crc_exclusion) + '\n')
    print (int(mymower.nbTotalExclusion))
    print("fin de exclusion")

    ################ dock
    print("Dock")
    crc_dock=0
    fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                      "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/DOCK.npy"
    if (os.path.exists(fileName)):
        
        dockPts=np.load(fileName)
        #print(dockPts)
        dockPts_count=int(len(dockPts))
        if((mymower.dockPointsCount)!=dockPts_count):
            print("Error in the dock point count")
        else:
            print("Nb_point : " , dockPts_count)
        
        for ip in range(int(len(dockPts))):
            toSend=toSend+str(dockPts[ip][0])+","+str(dockPts[ip][1])+","
            crc_dock=crc_dock + int(100*(dockPts[ip][0])) + int(100*(dockPts[ip][1]))
            
            pt=pt+1
            pt_total=pt_total+1
            if (pt>=trame_size) :
                toSend=toSend + '\r'
                print(toSend)
                send_serial_message(toSend)
                #prepare the new sentence
                toSend="AT+W,"+str(pt_total) +","
                pt=0
                
        
   

    print("fin de dock")
    consoleInsertText("Dock points send finish CRC: " + str(crc_dock) + '\n')  

    

    ################ mow
    print("Mow")
    fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                      "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/MOW.npy"
    if (os.path.exists(fileName)):
        crc_mow=0
        mowPts=np.load(fileName)
        #mowPts = mowPts1.astype(float)
        mowPts_count=int(len(mowPts))
        mymower.stopMowPointIdx=mowPts_count
        tk_labelStopIdx.set(mymower.stopMowPointIdx)
        if((mymower.mowPointsCount)!=mowPts_count):
            print("Error in the mow point count")
        else:
            print("Nb_point : " , mowPts_count)

        for ip in range(int(len(mowPts))):
            toSend=toSend+str(mowPts[ip][0])+","+str(mowPts[ip][1])+","
            crc_mow=crc_mow + int(100*(mowPts[ip][0])) + int(100*(mowPts[ip][1]))
            pt=pt+1
            pt_total=pt_total+1
            if (pt>=trame_size) :
                print("Mow pts")
                print(toSend)
                toSend=toSend + '\r\n'
                send_serial_message(toSend)
                
                wait_the_feedBack()
                
                #prepare the new sentence
                toSend="AT+W,"+str(pt_total) +","
                pt=0
                
                
        
            
    else:
        messagebox.showwarning('warning',"No mowing points for this map ???")
    print("fin de mow")
    consoleInsertText("Mow points send finish CRC: " + str(crc_mow) + '\n')   



        
    ################ free
    
    fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                      "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/FREE.npy"
    if (os.path.exists(fileName)):
        crc_free=0
        freePts=np.load(fileName)
        for ip in range(int(len(freePts))):
            toSend=toSend+str(freePts[ip][0])+","+str(freePts[ip][1])+","
            crc_free=crc_free + int(100*(freePts[ip][0])) + int(100*(freePts[ip][1]))
            pt=pt+1
            pt_total=pt_total+1
            if (pt>=trame_size) :
                print(toSend)
                toSend=toSend + '\r'
                send_serial_message(toSend)
                wait_the_feedBack()
                #prepare the new sentence
                toSend="AT+W,"+str(pt_total) +","
                pt=0
                
    else:
        messagebox.showwarning('warning',"No free points for this map ???")
    print("fin de free")
    consoleInsertText("Free points send finish CRC: " + str(crc_free) + '\n')   

    if (toSend != ("AT+W,"+str(pt_total) +",")):
            print ("reste final")
            #print(toSend)
            toSend=toSend + '\r'
            send_serial_message(toSend)
            wait_the_feedBack()
            pt=0
            


    #part 2  : AT+N  points description

    
    
    
    toSend="AT+N,"+str(mymower.perimeterPointsCount)+"," + str(mymower.exclusionPointsCount)
    toSend=toSend + "," + str(mymower.dockPointsCount) + "," + str(mymower.mowPointsCount)
    toSend=toSend + "," + str(mymower.freePointsCount) + "," + str(crc_point)
    #print(toSend)
    toSend=toSend + '\r'
    send_serial_message(toSend)
    consoleInsertText("Points description send finish " + '\n')   

    #part 3  : AT+X  exclusion description
    mymower.exclusionNr=0
    toSend="AT+X,0,"
    for mymower.exclusionNr in range(int(mymower.nbTotalExclusion)):
        
        fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                      "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/EXCLUSION"+"{0:0>2}".format(mymower.exclusionNr)+".npy"
          
        if (os.path.exists(fileName)):
            exclusion_NP=np.load(fileName)
            toSend=toSend+str(len(exclusion_NP))+","

    #print (toSend)
    toSend=toSend + '\r'
    send_serial_message(toSend)
    consoleInsertText("Exclusion description send finish " + '\n')

    crc_point=float(crc_perimeter) + float(crc_exclusion) + float(crc_dock) + float(crc_mow) + float(crc_free)
    consoleInsertText(str(float(crc_point)) + '\n')
    print("crc_perimeter ",crc_perimeter)
    print("crc_exclusion ",crc_exclusion)
    print("crc_dock ",crc_dock)
    print("crc_mow ",crc_mow)
    print("crc_free ",crc_free)
    print("CRC MAP ",crc_point)

    tk_labelStartIdx.set(0)
    

    mymower.isSendindMap=False
    
    
#load the active map locate into mower and save it on pi SD see decode AT message on RN response
def import_map_from_mower():
    returnval=messagebox.askyesno('Info',"You are going to replace the actual MAP"+str(mymower.mapSelected)+" by the one locate in the mower")
    if returnval :
        message="AT+RN"
        message=str(message)
        message=message + '\r'
        send_serial_message(message)
    else :
        messagebox.showinfo('info',"No Change made in the actual map")
            
        
    
    
def plot():
    #print("PLOT CRC LIST")
    ax[mymower.mapSelected].clear()
    #fig[mymower.mapSelected].clear()
    perimeterCRC=0

    #draw perimeter
    fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/PERIMETER.npy"

    
    if (os.path.exists(fileName)):
        perimeter=np.load(fileName)
        x_lon = np.zeros(int(len(perimeter)+1))
        y_lat = np.zeros(int(len(perimeter)+1))
        for ip in range(int(len(perimeter))):
                    x_lon[ip] = perimeter[ip][0]
                    y_lat[ip] = perimeter[ip][1]
                    perimeterCRC=perimeterCRC + int(100*x_lon[ip]) + int(100*y_lat[ip])
                  
        #close the drawing
        x_lon[ip+1] = perimeter[0][0]
        y_lat[ip+1] = perimeter[0][1]
        ax[mymower.mapSelected].plot(x_lon,y_lat, color='r', linewidth=0.4,marker='.',markersize=2)
        fig[mymower.mapSelected].subplots_adjust(left=0, right=1, top=1, bottom=0)
   #fig[uu],ax[uu]=plt.subplots()
        #canvas[mymower.mapSelected].draw()
    else:
        messagebox.showwarning('warning',"No perimeter for this map ???")

    
    #print("perimeterCRC ",perimeterCRC)

    #draw mow    
    mowCRC=0
    fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/MOW.npy"

    if (os.path.exists(fileName)):
        mowPts=np.load(fileName)
        x_lon = np.zeros(int(len(mowPts)))
        y_lat = np.zeros(int(len(mowPts)))
        for ip in range(int(len(mowPts))):
                    x_lon[ip] = mowPts[ip][0]
                    y_lat[ip] = mowPts[ip][1]
                    mowCRC=mowCRC+int(100*x_lon[ip])+int(100*y_lat[ip])
        ax[mymower.mapSelected].plot(x_lon,y_lat, color='g', linewidth=0.2)
        #canvas[mymower.mapSelected].draw()
    else:
        messagebox.showwarning('warning',"No mowing points for this map ?????")

    #print ("mowCRC ",mowCRC)
    
    #draw exclusion
    exclusionCRC=0
    for mymower.exclusionNr in range(int(mymower.nbTotalExclusion)):
        fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/EXCLUSION"+"{0:0>2}".format(mymower.exclusionNr)+".npy"
              
        if (os.path.exists(fileName)):
            mowPts=np.load(fileName)
            x_lon = np.zeros(int(len(mowPts)+1))
            y_lat = np.zeros(int(len(mowPts)+1))
            if (int(len(mowPts)) >=3):
                for ip in range(int(len(mowPts))):
                    x_lon[ip] = mowPts[ip][0]
                    y_lat[ip] = mowPts[ip][1]
                    exclusionCRC=exclusionCRC+int(100*x_lon[ip])+int(100*y_lat[ip]) 
                #close the drawing
                x_lon[ip+1] = mowPts[0][0]
                y_lat[ip+1] = mowPts[0][1]
             
                ax[mymower.mapSelected].plot(x_lon,y_lat, color='r', linewidth=1)
            #canvas[mymower.mapSelected].draw()
        else:
            messagebox.showwarning('warning',"No exclusion points for this map ?????")
    #print("exclusionCRC ",exclusionCRC)
    
    #draw dock
    dockCRC=0
    fileName=cwd + "/House" + "{0:0>2}".format(mymower.House) + \
                          "/maps"+"{0:0>2}".format(mymower.mapSelected)+"/DOCK.npy"

    if (os.path.exists(fileName)):
        mowPts=np.load(fileName)
        #print(mowPts)
        x_lon = np.zeros(int(len(mowPts)))
        y_lat = np.zeros(int(len(mowPts)))
        for ip in range(int(len(mowPts))):
                    x_lon[ip] = mowPts[ip][0]
                    y_lat[ip] = mowPts[ip][1]
                    dockCRC=dockCRC+int(100*x_lon[ip])+int(100*y_lat[ip])
        ax[mymower.mapSelected].plot(x_lon,y_lat, color='b', linewidth=0.6)
        #canvas[mymower.mapSelected].draw()
    else:
        messagebox.showwarning('warning',"No dock points for this map ?????")


    #print("dockCRC " , dockCRC)
    mymower.plotMapCRC=perimeterCRC+mowCRC+dockCRC+exclusionCRC
    ctrl=int(mymower.plotMapCRC)-int(mymower.fileMapCRC)
    if (abs(ctrl)>mymower.mapCrcRoundingRange):
        messagebox.showwarning('warning',"Issue into map file Try to import again this map PlotCRC / SunrayCRC : " + str(mymower.plotMapCRC) + " / " + str(mymower.fileMapCRC))

#end of plot

 
    # ax[mymower.mapSelected].set_xlabel('xlabel', fontsize=8)
       # ax[mymower.mapSelected].set_ylabel('ylabel', fontsize=8)
    #ax[uu].rcParams({'font.size': 22})
       # ax[mymower.mapSelected].subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)
    #ax[uu].axis('off')
    #ax[uu].subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)
    #fig[uu](figsize=(8, 6), dpi=80)
    canvas[mymower.mapSelected].draw()

        
fig=[FigureCanvasTkAgg]*max_map_inUse
ax =[plt]*max_map_inUse
toolbar=[None]*max_map_inUse
for uu in range (max_map_inUse):
    fig[uu],ax[uu]=plt.subplots()
    
MapsPage =ttk.Notebook(fen1)
FrameMapsPage= [None]*max_map_inUse
canvas = [None]*max_map_inUse

for i in range(max_map_inUse):
    FrameMapsPage[i] = tk.Frame(MapsPage,borderwidth="1",relief=tk.SOLID)
    canvas[i] = FigureCanvasTkAgg(fig[i],master=FrameMapsPage[i])
    canvas[i].get_tk_widget().place(x=60,y=5,width=350, height=350)
    if (i==0) :
        MapsPage.add(FrameMapsPage[i],text="House")
    else:
        MapsPage.add(FrameMapsPage[i],text=str(i))


MapsPage.place(x=0, y=0, height=400, width=800)
MapsPage.bind("<<NotebookTabChanged>>", onTabChange)
#MapsPage.bind("<ButtonRelease-1>", onTabChange)

MapsInfoline1 = tk.Label(MapsPage, text="Info maps")
MapsInfoline1.place(x=420,y=70, height=300, width=150)

tk.Label(MapsPage, text="House:",font=("Arial", 15), fg='green').place(x=440,y=45)
InfoHouseNrtxt = tk.Label(MapsPage, text=mymower.House,font=("Arial", 30), fg='red')
InfoHouseNrtxt.place(x=515,y=30, height=60, width=60)

ButtonDeleteMap = tk.Button(MapsPage,text="Delete This Map", wraplength=80,  command = delete_map)
ButtonDeleteMap.place(x=680,y=50,height=60, width=110)

ButtonExportMap = tk.Button(MapsPage,text="EXPORT TO ROBOT", wraplength=80,  command = ButtonExportMap_click)
ButtonExportMap.place(x=680,y=115,height=60, width=110)

ButtonImportMap = tk.Button(MapsPage,text="IMPORT FROM ROBOT", wraplength=80,  command = import_map_from_mower)
ButtonImportMap.place(x=680,y=180,height=60, width=110)

ButtonBackHome = tk.Button(MapsPage, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=119, width=115)
       
    

""" THE CAMERA PAGE ***************************************************"""
def visionThread(num):
    import os
    import cv2
    import numpy as np
    from picamera.array import PiRGBArray
    from picamera import PiCamera
    import tensorflow as tf
    

    import sys

    count=0
    consoleInsertText("Initialise Vision " + '\n')
# Set up camera constants
    #IM_WIDTH = 1280
    #IM_HEIGHT = 720
    #IM_WIDTH = 640   # Use smaller resolution for
    #IM_HEIGHT = 480  # slightly faster framerate
    IM_WIDTH = 544   # Use smaller resolution for
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
    PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,'frozen_inference_graph.pb')

# Path to label map file
    PATH_TO_LABELS = os.path.join(CWD_PATH,'data','mscoco_label_map.pbtxt')
    
# Path to save image
    folder_name=time.strftime("%Y%m%d%H%M%S")
    PATH_TO_SAVE_IMG = cwd + "/vision/" + folder_name +"/"
    os.mkdir(PATH_TO_SAVE_IMG)
    


# Number of classes the object detector can identify
    NUM_CLASSES = 99

## Load the label map.
# Label maps map indices to category names, so that when the convolution
# network predicts `5`, we know that this corresponds to `airplane`.
# Here we use internal utility functions, but anything that returns a
# dictionary mapping integers to appropriate string labels would be fine
    label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
    categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
    category_index = label_map_util.create_category_index(categories)

# Load the Tensorflow model into memory.
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
    #print(category_index) 
##    for index in category_index :
##        
##        print(category_index[index]['id'],category_index[index]['name'])
        

    # Initialize Picamera and grab reference to the raw capture
    camera = PiCamera()
    camera.resolution = (IM_WIDTH,IM_HEIGHT)
    camera.framerate = 10
    rawCapture = PiRGBArray(camera, size=(IM_WIDTH,IM_HEIGHT))
    rawCapture.truncate(0)
    consoleInsertText("Vision Started "+ '\n')
       #while(1):
        #videocapture.read(frame1);
        #cv2.videocapture(0)
    for frame1 in camera.capture_continuous(rawCapture, format="bgr",use_video_port=True):
        
        if (time.time() > mymower.visionDetectAt + 2):  #wait before next detection
            
        
            t1 = cv2.getTickCount()
            
        # Acquire frame and expand frame dimensions to have shape: [1, None, None, 3]
        # i.e. a single-column array, where each item in the column has the pixel RGB value
            frame = np.copy(frame1.array)
            frame.setflags(write=1)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_expanded = np.expand_dims(frame_rgb, axis=0)
            
        # Perform the actual detection by running the model with the image as input
            #if True:
            if ((myRobot.stateNames[mymower.state]=="OFF") or (myRobot.stateNames[mymower.state]=="FRWODO") or (myRobot.stateNames[mymower.state]=="PFND")):
                
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
                    min_score_thresh=0.60)#draw box only if score >60%
        
        
        
                cv2.putText(frame,"FPS: {0:.2f}".format(frame_rate_calc),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)

        # All  results have been drawn on the frame, so it's time to display it.
                #photo1=ImageTk.PhotoImage(image=Image.fromarray(frame))
                #Video_canvas.create_image(0,0,image = photo1,anchor=tk.NW)
                cv2.imshow('Live Video', frame)
                
                #multiple detection are possible into one frame
                #test only index 0 because it's the higher detection score
                if (scores[0][0] > visionDetectMinScore/100): # see config.py for global min score value
             
                     x = int(((boxes[0][0][1]+boxes[0][0][3])/2)*IM_WIDTH)
                     y = int(((boxes[0][0][0]+boxes[0][0][2])/2)*IM_HEIGHT)

                     lx= int(boxes[0][0][3]*IM_WIDTH-boxes[0][0][1]*IM_WIDTH)
                     ly= int(boxes[0][0][2]*IM_HEIGHT+boxes[0][0][0]*IM_HEIGHT)
                     
                     consoleInsertText("vision detect something here : " + str(x) + "  " + str(y) + '\n')
                     consoleInsertText("size is : " + str(lx) + "  " + str(ly) + '\n')
                     mymower.visionScore=100*scores[0][0]
                     mymower.objectDetectedID=int(classes[0][0])
                     mymower.surfaceDetected=100*(lx*ly)/(IM_WIDTH*IM_HEIGHT)
                     #mymower.visionDetectAt = time.time()
                     mymower.visionDetect=True
                     if (x > IM_WIDTH/2) :
                         mymower.VisionRollRight = 1
                         #consoleInsertText("Mower need to rotate RIGHT" + '\n')

                     else :
                         mymower.VisionRollRight = 0
                         #consoleInsertText("Mower need to rotate LEFT" + '\n')
                                         
                     fileName1=PATH_TO_SAVE_IMG + str(count) + ".jpg"
                     print(fileName1)

                     cv2.imwrite(fileName1, frame)
               #     cv2.imwrite("/home/pi/Documents/PiArdumower/vision/vision/person%d.jpg" % count, frame)
                     count=count+1

                     photo1=ImageTk.PhotoImage(image=Image.fromarray(frame))
                
                     Video_canvas.create_image(0,0,image = photo1,anchor=tk.NW)

            

            else:
                #t1 = cv2.getTickCount()
                #photo1=ImageTk.PhotoImage(image=Image.fromarray(frame))
                #print(1000*(cv2.getTickCount()-t1)/freq)
                #Video_canvas.create_image(0,0,image = photo1,anchor=tk.NW)
                     
                #print(1000*(cv2.getTickCount()-t1)/freq)
                cv2.imshow('Live Video', frame)
                #print(1000*(cv2.getTickCount()-t1)/freq)

             
             
            t2 = cv2.getTickCount()
            time1 = (t2-t1)/freq
            frame_rate_calc = 1/time1
        
        # Press 'q' to quit
            if (cv2.waitKey(1) == ord('q') or not(mymower.visionRunning)):
                send_pfo_message('rd0','1','2','3','4','5','6',)
                break
        #clear frame capture for next loop
        rawCapture.truncate(0)

    camera.close()

    cv2.destroyAllWindows()

    consoleInsertText("Vision Stopped "+ '\n')
       
    
def BtnStreamVideoStart_click():
    consoleInsertText("Start Video streaming" + '\n')
    if CamVar1.get()==1:
        myStreamVideo.start(1)
        
        #webbrowser.open("http://localhost:8000/index.html")

    else:
        myStreamVideo.start(0)
        
def BtnStreamVideoStop_click():
    consoleInsertText("Stop Video streaming" + '\n')
    myStreamVideo.stop()

def BtnVisionStart_click():
    if not (mymower.visionRunning):
        BtnVisionStart.place_forget()
        BtnVisionStop.place(x=650,y=200, height=25, width=80)
        print("start new thread")
        mymower.visionRunning=True
        t1 = threading.Thread(target=visionThread, args=(10,))
        t1.start() 
    
def BtnVisionStop_click():
    if (mymower.visionRunning):
        mymower.visionRunning=False
        print("stop vision")
        BtnVisionStop.place_forget()
        BtnVisionStart.place(x=650,y=200, height=25, width=80)
        
    
        
StreamVideoPage =tk.Frame(fen1)
StreamVideoPage.place(x=0, y=0, height=400, width=800)
FrameStreamVideo = tk.Frame(StreamVideoPage,borderwidth="1",relief=tk.SOLID)
FrameStreamVideo.place(x=0, y=0, width=544, height=400)

Video_canvas=tk.Canvas(FrameStreamVideo,bg='red')
Video_canvas.place(x=0, y=0, width=544, height=400)


OptBtnStreamVideo1=tk.Radiobutton(StreamVideoPage, text="320*240",relief=tk.SOLID,variable=CamVar1,value=0,anchor='nw').place(x=620,y=10,width=100, height=20)
OptBtnStreamVideo2=tk.Radiobutton(StreamVideoPage, text="640*480",relief=tk.SOLID,variable=CamVar1,value=1,anchor='nw').place(x=620,y=30,width=100, height=20)
#tk.Label(FrameStreamVideo, text='To view the vidéo stream use a browser http://(Your PI IP Adress and):8000/index.html').place(x=10,y=180)
#tk.Label(FrameStreamVideo, text='Do not forget to activate the Camera into the RaspiConfig').place(x=10,y=200)

BtnStreamVideoStart= tk.Button(StreamVideoPage,command = BtnStreamVideoStart_click,text="Start Streaming")
BtnStreamVideoStart.place(x=650,y=100, height=25, width=110)
BtnStreamVideoStop= tk.Button(StreamVideoPage,command = BtnStreamVideoStop_click,text="Stop Streaming")
BtnStreamVideoStop.place(x=650,y=130, height=25, width=110)

BtnVisionStart= tk.Button(StreamVideoPage,command = BtnVisionStart_click,text="Start Vision")
if (useVision):
    BtnVisionStart.place(x=650,y=200, height=25, width=80)
BtnVisionStop= tk.Button(StreamVideoPage,command = BtnVisionStop_click,text="Stop Vision")

ButtonBackHome = tk.Button(StreamVideoPage, image=imgBack, command = ButtonBackToMain_click)
ButtonBackHome.place(x=680, y=280, height=120, width=120)

        
        
""" THE TEST PAGE ***************************************************"""




def ButtonOdo10TurnFw_click():
    message="AT+E1"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)
    ConsolePage.tkraise()
    fen1.update()
    
def ButtonFanStart_click():
    message="AT+E4"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)
    
def ButtonFanStop_click():
    message="AT+E5"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)
        
def ButtonOdo3MlFw_click():
    message="AT+E3"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)
    ConsolePage.tkraise()
    fen1.update()
    
def ButtonTestSensor_click():
    message="AT+F"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)
    ConsolePage.tkraise()
    fen1.update()

def ButtonOdoRot360_click():
    message="AT+E2"
    message=str(message)
    message=message + '\r'
    send_serial_message(message)
    ConsolePage.tkraise()
    fen1.update()


TestPage =tk.Frame(fen1)
TestPage.place(x=0, y=0, height=400, width=800)

ButtonOdo10TurnFw = tk.Button(TestPage)
ButtonOdo10TurnFw.place(x=30,y=15, height=25, width=200)
ButtonOdo10TurnFw.configure(command = ButtonOdo10TurnFw_click)
ButtonOdo10TurnFw.configure(text="TEST 10 Turn")


ButtonFanStart = tk.Button(TestPage)
ButtonFanStart.place(x=300,y=115, height=25, width=200)
ButtonFanStart.configure(command = ButtonFanStart_click)
ButtonFanStart.configure(text="Start Fan")
ButtonFanStop = tk.Button(TestPage)
ButtonFanStop.place(x=300,y=150, height=25, width=200)
ButtonFanStop.configure(command = ButtonFanStop_click)
ButtonFanStop.configure(text="Stop Fan")

ButtonTestSensor= tk.Button(TestPage)
ButtonTestSensor.place(x=550,y=165, height=25, width=200)
ButtonTestSensor.configure(command = ButtonTestSensor_click)
ButtonTestSensor.configure(text="Test Sensor")
##
ButtonOdo3MlFw = tk.Button(TestPage)
ButtonOdo3MlFw.place(x=30,y=115, height=25, width=200)
ButtonOdo3MlFw.configure(command = ButtonOdo3MlFw_click)
ButtonOdo3MlFw.configure(text="3 Meters Forward")

##ButtonOdoRot180= tk.Button(TestPage)
##ButtonOdoRot180.place(x=300,y=65, height=25, width=200)
##ButtonOdoRot180.configure(command = ButtonOdoRot180_click)
##ButtonOdoRot180.configure(text="Rotate 180 Degree")

ButtonOdoRot360= tk.Button(TestPage)
ButtonOdoRot360.place(x=30,y=165, height=25, width=200)
ButtonOdoRot360.configure(command = ButtonOdoRot360_click)
ButtonOdoRot360.configure(text="Rotate 360 Degree")

##ButtonOdoRotNonStop= tk.Button(TestPage)
##ButtonOdoRotNonStop.place(x=300,y=165, height=25, width=200)
##ButtonOdoRotNonStop.configure(command = ButtonOdoRotNonStop_click)
##ButtonOdoRotNonStop.configure(text="Rotate Non Stop 100 Turns")




def ButtonWifiOn_click():
    #returnval=messagebox.askyesno('Info',"Turn On the Wifi")
    #if returnval :
        subprocess.Popen("sudo systemctl start hostapd", shell=True)
        subprocess.Popen("sudo systemctl start dnsmasq", shell=True)
        subprocess.Popen("sudo rfkill unblock wifi", shell=True)
        consoleInsertText('WIFI is ON'+ '\n')

def ButtonWifiOff_click():
    #returnval=messagebox.askyesno('Info',"Turn Off the Wifi")
    #if returnval :
        subprocess.Popen("sudo systemctl stop hostapd", shell=True)
        subprocess.Popen("sudo systemctl stop dnsmasq", shell=True)
        subprocess.Popen("sudo rfkill block wifi", shell=True)
        consoleInsertText('WIFI is OFF'+ '\n')

ButtonWifiOn= tk.Button(TestPage)
ButtonWifiOn.place(x=550,y=65, height=25, width=100)
ButtonWifiOn.configure(command = ButtonWifiOn_click)
ButtonWifiOn.configure(text="Wifi On")      


ButtonWifiOff= tk.Button(TestPage)
ButtonWifiOff.place(x=660,y=65, height=25, width=100)
ButtonWifiOff.configure(command = ButtonWifiOff_click)
ButtonWifiOff.configure(text="Wifi Off")

def ButtonBTOn_click():
    returnval=messagebox.askyesno('Info',"Turn On the Bluetooth")
    if returnval :
        subprocess.Popen("sudo rfkill unblock bluetooth", shell=True)
def ButtonBTOff_click():
    returnval=messagebox.askyesno('Info',"Turn Off the Bluetooth")
    if returnval :
        subprocess.Popen("sudo rfkill block bluetooth", shell=True)

ButtonBTOn= tk.Button(TestPage)
ButtonBTOn.place(x=550,y=15, height=25, width=100)
ButtonBTOn.configure(command = ButtonBTOn_click)
ButtonBTOn.configure(text="BT On")      


ButtonBTOff= tk.Button(TestPage)
ButtonBTOff.place(x=660,y=15, height=25, width=100)
ButtonBTOff.configure(command = ButtonBTOff_click)
ButtonBTOff.configure(text="BT Off")

ButtonBackHome = tk.Button(TestPage, image=imgBack, command = ButtonBackToMain_click)
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
    if (SliderStartMap[tabTimerSelected].get()==0):
        print("map 0 is not viewable")
        return
    fileName=cwd + "/House" + "{0:0>2}".format(SliderStartHouse[tabTimerSelected].get()) + \
                          "/maps"+"{0:0>2}".format(SliderStartMap[tabTimerSelected].get())+"/MAIN.npy"
    if (os.path.exists(fileName)):   
        image02= imgHouseMap[SliderStartHouse[tabTimerSelected].get()]
        ImageMap.configure(image=image02)       
        ImageMap.update()            
    else:   
        ImageMap.configure(image=imgHouseNoMap)       
        ImageMap.update()

def SliderHourStartGroup_click(var1):
    pass
    #print("heure change "+str(var1))
     
def checkTimerStop():
    if (mymower.timerUse):
        print("checktimer stop : ",mymower.ActualRunningTimer, " / " , mymower.lastRunningTimer)
        print("++/"+mymower.opNames+"/++")
        #stop part
        #reset timer value if end mowing at end of map or on low bat
        #warning if go to dock duration in < 1 minute ????????
        if ((mymower.ActualRunningTimer<98) and (mymower.opNames == "Go TO DOCK")):
            print("go to dock : timer is reset")
            mymower.lastRunningTimer=mymower.ActualRunningTimer
            mymower.ActualRunningTimer = 99
            tk_infoTimer.set("")
            
            
        if (mymower.ActualRunningTimer<98): #one timer is active
            stopmin =  60*myRobot.TimerstopTime_hour[mymower.ActualRunningTimer] + myRobot.TimerstopTime_minute[mymower.ActualRunningTimer]
            currmin =  60*dt.datetime.today().hour+dt.datetime.today().minute
            if ((currmin >= stopmin) ) :
                print("Timer stop mowing cycle")
                button_home_click()
                tk_infoTimer.set("")
                
            else:
                
                print("Continue mowing for ",str(stopmin-currmin)," minutes.")

            
                
        else:
            print("no timer active and not actual mowing ")
            


def checkTimerStart():
    if (mymower.opNames != "DOCK"):
        return
    if (mymower.timerUse):
        print("checktimer start : ",mymower.ActualRunningTimer, " / " , mymower.lastRunningTimer)
        #start part
        info01="checktimer : " + str(mymower.ActualRunningTimer) + " / " + str(mymower.lastRunningTimer)
        tk_infoTimer.set(info01)
        
        
        for i in range(15):
            
            if ((mymower.opNames == "DOCK") & (myRobot.Timeractive[i])):

                if  (myRobot.TimerdaysOfWeek[i] & (1<< dt.datetime.today().weekday())):
                    startmin = 60*myRobot.TimerstartTime_hour[i] + myRobot.TimerstartTime_minute[i]
                    stopmin =  60*myRobot.TimerstopTime_hour[i] + myRobot.TimerstopTime_minute[i]
                    #print(startmin)          
                    currmin =  60*dt.datetime.today().hour+dt.datetime.today().minute
                    #reset the last running timer at 0H00 each day
                    if ((currmin == 0) & (mymower.lastRunningTimer!=99)):
                        mymower.lastRunningTimer=99
                        
                    #print(currmin)
                    #print(stopmin)
                    if ((currmin >= startmin) & (currmin < stopmin)) :
                        if (mymower.lastRunningTimer==i):
                            print("Timer " , i," already run for this day")
                            info01="Timer " +str(i) + " already run for this day"
                            tk_infoTimer.set(info01)

                        else:
                            if (mymower.batSense > -1*myRobot.batFullCurrent):
                                print("Start timer nr " , i)
                                mymower.ActualRunningTimer = i
                                mymower.TimerHouseToStart=tk_timerHouse[i].get()
                                mymower.TimerMapToStart=tk_TimerstartMap[i].get()
                                print(mymower.TimerHouseToStart,mymower.TimerMapToStart)
                                mymower.mapSelected=mymower.TimerMapToStart
                                mymower.House=mymower.TimerHouseToStart
                                textHouseNr.configure(text=mymower.House)
                                #rebuildHouseMap()
                                export_map_to_mower(mymower.House,mymower.mapSelected)
                                mymower.startAfterUploadFinish=True
                                info01="Timer "+str(i) +" is running"
                                tk_infoTimer.set(info01)
                            else:
                                print("Battery not enought charged ",mymower.batSense)
                                info01="Bat charging "
                                tk_infoTimer.set(info01)
                                
                    
    else:
        info01="Timer not active "
        tk_infoTimer.set(info01)            
        
            
            
                              
                             
                           
               
 





TabTimer=ttk.Notebook(fen1)
SheetTimer= [None]*15
for i in range(15):
    SheetTimer[i]=tk.Frame(TabTimer,width=800,height=380)
    TabTimer.add(SheetTimer[i],text="T "+str(i))

    
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
tk_Random= []
tk_ByLane= []
tk_Perimeter= []

    
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
    


tk_timerDayVar=[[None] * 7 for i in range(15)]

ChkBtnDayGroup = [[None] * 7 for i in range(15)]
ChkBtnEnableGroup=[None]*15
FrameStartGroup=[None]*15
FrameStopGroup=[None]*15
SliderHourStartGroup=[None]*15
SliderMinuteStartGroup=[None]*15
SliderHourStopGroup=[None]*15
SliderMinuteStopGroup=[None]*15
SliderStartNrLaneGroup=[None]*15
SliderStartArea=[None]*15
SliderStartMowPatternGroup=[None]*15
SliderStartLaneMaxlenghGroup=[None]*15
SliderStartHouse=[None]*15
SliderStartMap=[None]*15

FrameRollDir=[None]*15
FrameMowPattern=[None]*15
imageMapCanvas=[tk.Canvas]*15
itemImageMapCanvas=[None]*15
FrameLaneParameter=[None]*15

#RdBtn_Random=[None]*15
#RdBtn_ByLane=[None]*15
#RdBtn_Perimeter=[None]*15


#create the house map image for fast preview into timer page
imgHouseNoMap=tk.PhotoImage(file=cwd + "/icons/noMap.png")
imgHouseMap=[ImageTk.PhotoImage]*10
for i in range (10):
   fileName=cwd + "/House" + "{0:0>2}".format(i) + "/fullHouseMap.png"
   if (os.path.exists(fileName)):
      img2 = Image.open(fileName)
      img3=img2.resize((250, 200), Image.LANCZOS)
      imgHouseMap[i] = ImageTk.PhotoImage(img3)
      
       

for i in range(15):
    ChkBtnEnableGroup[i] = tk.Checkbutton(SheetTimer[i],text="Enable this Timer",font=("Arial", 14), fg='red',variable=tk_timerActive[i],anchor = 'w')
    ChkBtnEnableGroup[i].place(x=20, y=0, height=25, width=380)
    
    FrameStartGroup[i] = tk.Frame(SheetTimer[i],borderwidth="1",relief=tk.SUNKEN)
    FrameStartGroup[i].place(x=20, y=30, height=115, width=350)
    startText="Mower START at " + str(tk_timerStartTimehour[i].get()) +":" +str(tk_timerStartTimeMinute[i].get())
    tk.Label(FrameStartGroup[i], text=startText,font=("Arial", 12), fg='green').place(x=0,y=10, height=15, width=300)
    SliderHourStartGroup[i] = tk.Scale(FrameStartGroup[i],command = SliderHourStartGroup_click(i),from_=0, to=23,variable=tk_timerStartTimehour[i],relief=tk.SOLID,orient='horizontal')
    SliderHourStartGroup[i].place(x=80, y=30, height=40, width=265)
    SliderMinuteStartGroup[i] = tk.Scale(FrameStartGroup[i],from_=0, to=59,variable=tk_timerStartTimeMinute[i],relief=tk.SOLID,orient='horizontal')
    SliderMinuteStartGroup[i].place(x=80, y=70, height=40, width=265)
    tk.Label(FrameStartGroup[i], text='Hour :',font=("Arial", 14), fg='green').place(x=10,y=30, height=40, width=70)
    tk.Label(FrameStartGroup[i], text='Minute :',font=("Arial", 14), fg='green').place(x=10,y=70, height=40, width=70)
    

    FrameStopGroup[i] = tk.Frame(SheetTimer[i],borderwidth="1",relief=tk.SUNKEN)
    FrameStopGroup[i].place(x=380, y=30, height=115, width=350)
    stopText="Mower STOP at " + str(tk_timerStopTimehour[i].get()) +":" +str(tk_timerStopTimeMinute[i].get())
    tk.Label(FrameStopGroup[i], text=stopText,font=("Arial", 12), fg='green').place(x=0,y=10, height=15, width=300)
    SliderHourStopGroup[i] = tk.Scale(FrameStopGroup[i],from_=0, to=23,variable=tk_timerStopTimehour[i],relief=tk.SOLID,orient='horizontal')
    SliderHourStopGroup[i].place(x=80, y=30, height=40, width=265)
    SliderMinuteStopGroup[i] = tk.Scale(FrameStopGroup[i],from_=0, to=59,variable=tk_timerStopTimeMinute[i],relief=tk.SOLID,orient='horizontal')
    SliderMinuteStopGroup[i].place(x=80, y=70, height=40, width=265)
    tk.Label(FrameStopGroup[i], text='Hour :',font=("Arial", 14), fg='green').place(x=10,y=30, height=40, width=70)
    tk.Label(FrameStopGroup[i], text='Minute :',font=("Arial", 14), fg='green').place(x=10,y=70, height=40, width=70)

    tk.Label(SheetTimer[i],text="House :",fg='green').place(x=5,y=180, height=20, width=80)
    SliderStartHouse[i]= tk.Scale(SheetTimer[i],from_=0, to=10,font=("Arial", 8),variable=tk_timerHouse[i],relief=tk.SOLID,orient='horizontal')
    SliderStartHouse[i].place(x=10,y=200, height=35, width=180)
    SliderStartHouse[i].bind("<ButtonRelease-1>", onSliderHouseChange)
    tk.Label(SheetTimer[i],text="Map :",fg='green').place(x=195,y=180, height=20, width=100)
    SliderStartMap[i]= tk.Scale(SheetTimer[i],from_=0, to=10,font=("Arial", 8),variable=tk_TimerstartMap[i],relief=tk.SOLID,orient='horizontal')
    SliderStartMap[i].place(x=200,y=200, height=35, width=200)
    SliderStartMap[i].bind("<ButtonRelease-1>", onSliderMapChange)


    
   
##    FrameMowPattern[i] = tk.Frame(SheetTimer[i],borderwidth="1",relief=tk.SUNKEN)
##    FrameMowPattern[i].place(x=410, y=180, height=200, width=250)
##    imageMapCanvas[i] = tk.Canvas(FrameMowPattern[i])
##    
##    imageMapCanvas[i].create_image(0,0, image = image01, anchor = "nw")
    
##    itemImageMapCanvas[i]=imageMapCanvas[i].create_image(0,0, image = image02, anchor = "nw")
##    imageMapCanvas[i].pack(expand = tk.YES, fill = tk.BOTH)
    #itemImageMapCanvas[i].pack(expand = tk.YES, fill = tk.BOTH)
        

    
    #tk.Label(FrameMowPattern[i],text="MOW PATTERN :",fg='green').pack(side='top',anchor='w')
    #RdBtn_Random[i]=tk.Radiobutton(FrameMowPattern[i], text="Random", variable=tk_TimerstartMap[i], value=0).pack(side='top',anchor='w')
    #RdBtn_ByLane[i]=tk.Radiobutton(FrameMowPattern[i], text="By Lane", variable=tk_TimerstartMap[i], value=1).pack(side='top',anchor='w')
    #RdBtn_Perimeter[i]=tk.Radiobutton(FrameMowPattern[i], text="Perimeter", variable=tk_TimerstartMap[i], value=2).pack(side='top',anchor='w')
    
    #FrameLaneParameter[i] = tk.Frame(SheetTimer[i],borderwidth="1",relief=tk.SUNKEN)
    #FrameLaneParameter[i].place(x=10, y=240, height=140, width=390)
    #tk.Label(FrameLaneParameter[i],text="Maximum Lane Lenght :",fg='green').place(x=10,y=0, height=20, width=360)
    #SliderStartLaneMaxlenghGroup[i]= tk.Scale(FrameLaneParameter[i],from_=1, to=50,font=("Arial", 8),variable=tk_timerStartLaneMaxlengh[i],relief=tk.SOLID,orient='horizontal')
    #SliderStartLaneMaxlenghGroup[i].place(x=10,y=20, height=35, width=360)
    
    #tk.Label(FrameLaneParameter[i], text='Roll Dir',font=("Arial", 12), fg='green').place(x=10,y=60, height=15, width=80)
    #RdBtn_Right=tk.Radiobutton(FrameLaneParameter[i], text="Right",variable=tk_timerStartRollDir[i], value=0).place(x=10,y=75, height=20, width=80)
    #RdBtn_Left=tk.Radiobutton(FrameLaneParameter[i], text="Left ",variable=tk_timerStartRollDir[i], value=1).place(x=10,y=95, height=20, width=80)


##    tk.Label(FrameLaneParameter[i],text="START Lane",font=("Arial", 12), fg='green').place(x=150,y=60, height=15, width=90)
##    SliderStartNrLaneGroup[i]= tk.Scale(FrameLaneParameter[i],from_=1, to=3,variable=tk_timerStartNrLane[i],relief=tk.SOLID,orient='horizontal').place(x=160,y=75, height=40, width=70)
##
##    tk.Label(FrameLaneParameter[i],text="START Area",font=("Arial", 12), fg='green').place(x=260,y=60, height=15, width=90)
##    SliderStartArea[i]= tk.Scale(FrameLaneParameter[i],from_=1, to=3,variable=tk_timerStartArea[i],relief=tk.SOLID,orient='horizontal').place(x=260,y=75, height=40, width=70)

    for j in range(7):
        tk_timerDayVar[i][j]=tk.BooleanVar()
        ChkBtnDayGroup[i][j] = tk.Checkbutton(SheetTimer[i],text=days_list[j],variable=tk_timerDayVar[i][j],relief=tk.GROOVE,borderwidth="1",anchor = 'w')
        ChkBtnDayGroup[i][j].place(x=110*j+10, y=150, height=25, width=120) 




    
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
    #Read the file and create the list
    with open("timer_list.bin","rb") as fp :
        timer_list=pickle.load(fp)
        #print("Timer file loaded")
        #print(timer_list)

        for i in range(15):
            myRobot.Timeractive[i]=timer_list[i][0]
            tk_timerActive[i].set(myRobot.Timeractive[i])
            myRobot.TimerstartTime_hour[i]=timer_list[i][1]
            tk_timerStartTimehour[i].set(myRobot.TimerstartTime_hour[i])
            myRobot.TimerstartTime_minute[i]=timer_list[i][2]
            tk_timerStartTimeMinute[i].set(myRobot.TimerstartTime_minute[i])
            myRobot.TimerstopTime_hour[i]=timer_list[i][3]
            tk_timerStopTimehour[i].set(myRobot.TimerstopTime_hour[i])
            myRobot.TimerstopTime_minute[i]=timer_list[i][4]
            tk_timerStopTimeMinute[i].set(myRobot.TimerstopTime_minute[i])
            myRobot.TimerstartHouse[i]=timer_list[i][5]
            tk_timerHouse[i].set(myRobot.TimerstartHouse[i])
            myRobot.TimerstartMap[i]=timer_list[i][6]
            tk_TimerstartMap[i].set(myRobot.TimerstartMap[i])
            myRobot.TimerstartNrLane[i]=timer_list[i][7]
            tk_timerStartNrLane[i].set(myRobot.TimerstartNrLane[i])
            myRobot.TimerstartRollDir[i]=timer_list[i][8]
            tk_timerStartRollDir[i].set(myRobot.TimerstartRollDir[i])
            myRobot.TimerstartLaneMaxlengh[i]=timer_list[i][9]
            tk_timerStartLaneMaxlengh[i].set(myRobot.TimerstartLaneMaxlengh[i])
            #the 7 days of the week as byte
            tk_timerDayVar[i][0].set(timer_list[i][10])
            tk_timerDayVar[i][1].set(timer_list[i][11])
            tk_timerDayVar[i][2].set(timer_list[i][12])
            tk_timerDayVar[i][3].set(timer_list[i][13])
            tk_timerDayVar[i][4].set(timer_list[i][14])
            tk_timerDayVar[i][5].set(timer_list[i][15])
            tk_timerDayVar[i][6].set(timer_list[i][16])
        
        
            myRobot.TimerdaysOfWeek[i]=1*int(tk_timerDayVar[i][0].get())+2*int(tk_timerDayVar[i][1].get())+4*int(tk_timerDayVar[i][2].get())+\
                  8*int(tk_timerDayVar[i][3].get())+16*int(tk_timerDayVar[i][4].get())+32*int(tk_timerDayVar[i][5].get())+\
                  64*int(tk_timerDayVar[i][6].get()) 

        

def save_TimerList():
    #timer_list=[[0,10,10,11,11,0,0,0,0,0,1,1,1,1,1,1,1],[0,10,10,11,11,0,0,0,0,0,0,1,0,1,0,1,0,1],[0,10,10,11,11,0,0,0,0,0,0,1,0,1,0,1,0,1],[0,10,10,11,11,0,0,0,0,0,0,1,0,1,0,1,0,1],[0,10,10,11,11,0,0,0,0,0,0,1,0,1,0,1,0,1]]
    timer_list=[]
    #refresh variable to page change
    for i in range(15):
        myRobot.Timeractive[i]=tk_timerActive[i].get()
        myRobot.TimerstartTime_hour[i]=tk_timerStartTimehour[i].get()
        myRobot.TimerstartTime_minute[i]=tk_timerStartTimeMinute[i].get()
        myRobot.TimerstopTime_hour[i]=tk_timerStopTimehour[i].get()
        myRobot.TimerstopTime_minute[i]=tk_timerStopTimeMinute[i].get()
        myRobot.TimerstartHouse[i]=tk_timerHouse[i].get()
        myRobot.TimerstartMap[i]=tk_TimerstartMap[i].get()
        myRobot.TimerstartNrLane[i]=tk_timerStartNrLane[i].get()
        myRobot.TimerstartRollDir[i]=tk_timerStartRollDir[i].get()
        myRobot.TimerstartLaneMaxlengh[i]=tk_timerStartLaneMaxlengh[i].get()
        #the 7 days of the week as byte
        myRobot.TimerdaysOfWeek[i]=1*int(tk_timerDayVar[i][0].get())+2*int(tk_timerDayVar[i][1].get())+4*int(tk_timerDayVar[i][2].get())+\
                  8*int(tk_timerDayVar[i][3].get())+16*int(tk_timerDayVar[i][4].get())+32*int(tk_timerDayVar[i][5].get())+\
                  64*int(tk_timerDayVar[i][6].get())


    
    maTimerLigne=[]
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
        #print(maTimerLigne)
        timer_list.append(maTimerLigne)
        maTimerLigne=[]
                
        
    with open("timer_list.bin","wb") as fp :
        pickle.dump(timer_list,fp)
        #print("Timer file saved")
    

    
ButtonLoadTimer = tk.Button(TabTimer)
ButtonLoadTimer.place(x=680,y=225, height=25, width=150)
ButtonLoadTimer.configure(command = load_TimerList)
ButtonLoadTimer.configure(text="Load Timer")

ButtonSaveTimer = tk.Button(TabTimer)
ButtonSaveTimer.place(x=680,y=265, height=25, width=150)
ButtonSaveTimer.configure(command = save_TimerList)
ButtonSaveTimer.configure(text="Save Timer")





ImageMap=tk.Label(TabTimer, font=("Arial", 20), fg='red')
ImageMap.place(x=410,y=210, height=200, width=250)

#FrameLaneParameter[i] = tk.Frame(SheetTimer[i],borderwidth="1",relief=tk.SUNKEN)
#FrameLaneParameter[i].place(x=10, y=240, height=140, width=390)
#ButtonCheckTimer = tk.Button(TabTimer)
#ButtonCheckTimer.place(x=300,y=400, height=25, width=150)
#ButtonCheckTimer.configure(command = checkTimerStart,text="checkTimerStart")

ButtonBackHome = tk.Button(TabTimer, image=imgBack, command = ButtonBackToMain_click)
#ButtonBackHome = tk.Button(TabTimer, image=imgHouseMap[0], command = ButtonBackToMain_click)


ButtonBackHome.place(x=680, y=310, height=120, width=120)






 
""" THE MAIN PAGE ***************************************************"""


def ButtonPowerOff_click():
    returnval=messagebox.askyesno('Info',"Are you sure you want to shutdown all the PCB ?")
    if returnval :
        message="AT+Y3"
        message=str(message)
        message=message + '\r'
        send_serial_message(message)        

MainPage = tk.Frame(fen1)
MainPage.place(x=0, y=0, height=480, width=800)

ButtonAuto = tk.Button(MainPage,image=imgAuto,command = ButtonAuto_click)
ButtonAuto.place(x=10,y=10, height=130, width=100)

ButtonManual = tk.Button(MainPage,image=imgManual)
ButtonManual.place(x=145,y=10, height=130, width=100)
ButtonManual.configure(command = ButtonManual_click)

ButtonSetting = tk.Button(MainPage,image=imgSetting)
ButtonSetting.place(x=280,y=10, height=130, width=100)
ButtonSetting.configure(command = ButtonSetting_click)


ButtonConsole = tk.Button(MainPage,image=imgConsole)
ButtonConsole.place(x=415,y=10, height=130, width=100)
ButtonConsole.configure(command = ButtonConsole_click)


ButtonTest = tk.Button(MainPage,image=imgTest)
ButtonTest.place(x=550,y=10, height=130, width=100)
ButtonTest.configure(command = ButtonTest_click)

#ButtonPlot = tk.Button(MainPage, image=imgPlot, command = ButtonPlot_click)
#ButtonPlot.place(x=10,y=145,width=100, height=130)

ButtonSchedule = tk.Button(MainPage, image=imgSchedule, command = ButtonSchedule_click)
ButtonSchedule.place(x=145,y=145,width=100, height=130)

#ButtonCamera = tk.Button(MainPage, image=imgCamera, command = ButtonCamera_click)
#ButtonCamera.place(x=280,y=145,width=100, height=130)

#ButtonGps = tk.Button(MainPage, image=imgGps, command = ButtonGps_click)
#ButtonGps.place(x=415,y=145,width=100, height=130)

ButtonMaps = tk.Button(MainPage, image=imgMaps, command = ButtonMaps_click)
ButtonMaps.place(x=415,y=145,width=100, height=130)

ButtonPowerOff = tk.Button(MainPage, image=imgPowerOff, command = ButtonPowerOff_click)
ButtonPowerOff.place(x=685,y=280,width=100, height=120)

Buttonimgardu=tk.Button(MainPage,image=imgArdumower,command = ButtonInfo_click)
Buttonimgardu.place(x=10,y=280,height=120,width=650)

Datetext = tk.Label(MainPage, text='',textvariable=tk_date_Now,font=("Arial", 20), fg='red')
Datetext.place(x=10,y=400, height=25, width=240)

Statustext = tk.Label(MainPage, text='',textvariable=tk_MainStatusLine,font=("Arial", 20), fg='red')
Statustext.place(x=240,y=400, height=25, width=400)

################## DESACTIVATE THE BT TO SAVE BATTERY
#subprocess.call(["rfkill","block","bluetooth"])
subprocess.Popen("sudo rfkill block bluetooth", shell=True)
subprocess.Popen("sudo iw wlan0 set power_save off", shell=True)
#subprocess.Popen("sudo rfkill block wifi", shell=True)
#sudo rfkill block wifi
#sudo rfkill unblock wifi
#sudo rfkill block bluetooth
#sudo rfkill unblock bluetooth

# use to drive the mower using the keyboard in manual
def check_keyboard(e) :   
    if (page_list[mymower.focusOnPage]=="MANUAL") & (ManualKeyboardUse.get()==1):
        if (e.char=="m"):  
            buttonBlade_start_click()            
        if (e.char=='q'):
            buttonBlade_stop_click()
              
        
def kbd_spaceKey(e) :
    if (page_list[mymower.focusOnPage]=="MANUAL") & (ManualKeyboardUse.get()==1):
        button_stop_all_click()    
def kbd_leftKey(e) :
    if (page_list[mymower.focusOnPage]=="MANUAL") & (ManualKeyboardUse.get()==1):
        ButtonLeft_click()
def kbd_rightKey(e) :
    if (page_list[mymower.focusOnPage]=="MANUAL") & (ManualKeyboardUse.get()==1):
        ButtonRight_click()
def kbd_upKey(e) :
    if (page_list[mymower.focusOnPage]=="MANUAL") & (ManualKeyboardUse.get()==1):
        ButtonForward_click()
def kbd_downKey(e) :
    if (page_list[mymower.focusOnPage]=="MANUAL") & (ManualKeyboardUse.get()==1):
        ButtonStop_click()    





fen1.bind("<KeyPress>", check_keyboard)
fen1.bind('<Left>', kbd_leftKey)
fen1.bind('<Right>', kbd_rightKey)
fen1.bind('<Up>', kbd_upKey)
fen1.bind('<Down>', kbd_downKey)
fen1.bind('<space>', kbd_spaceKey)

##mymower.focusOnPage=1
##AutoPage.tkraise()
#fen1.wm_attributes("-transparentcolor", 'grey')



#on startup PI update Date time from Internet
#subprocess.Popen("sudo systemctl stop ntp.service", shell=True)
#subprocess.Popen("sudo systemctl disable ntp.service", shell=True)
#time.sleep(10)




##
##if (streamVideoOnPower):
##    BtnStreamVideoStart_click()

ButtonReadSettingFromFile_click()
load_TimerList()
checkSerial()
fen1.mainloop()




            
            
        
