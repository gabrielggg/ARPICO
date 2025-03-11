# Simple HTTP Server Example
# Control an LED and read a Button using a web browser

import time
import math
import network


import socket
from machine import Pin,UART
from rotary_irq_rp2 import RotaryIRQ

J1rotdir = 1
J2rotdir = 1
J3rotdir = 1
J4rotdir = 0
J5rotdir = 1
J6rotdir = 1

TRACKrotdir = 0

SpeedMult = 200

J1PosAngLim = 170
J1NegAngLim = -170
J1StepLim = 15200

J2PosAngLim = 0
J2NegAngLim = -132
J2StepLim = 7300

J3PosAngLim = 141
J3NegAngLim = 1
J3StepLim = 7850

J4PosAngLim = 165
J4NegAngLim = -165
J4StepLim = 15200

J5PosAngLim = 105
J5NegAngLim = -105
J5StepLim = 4575

J6PosAngLim = 155
J6NegAngLim = -155
J6StepLim = 6625

J1DegPerStep = float((J1PosAngLim - J1NegAngLim)/float(J1StepLim))
J2DegPerStep = float((J2PosAngLim - J2NegAngLim)/float(J2StepLim))
J3DegPerStep = float((J3PosAngLim - J3NegAngLim)/float(J3StepLim))
J4DegPerStep = float((J4PosAngLim - J4NegAngLim)/float(J4StepLim))
J5DegPerStep = float((J5PosAngLim - J5NegAngLim)/float(J5StepLim))
J6DegPerStep = float((J6PosAngLim - J6NegAngLim)/float(J6StepLim))
MotDir = "000000"
J1motdir = MotDir[:-5]
J2motdir = MotDir[1:-4]
J3motdir = MotDir[2:-3]
J4motdir = MotDir[3:-2]
J5motdir = MotDir[4:-1]
J6motdir = MotDir[5:]

CalDir = "001001"
J1caldir = CalDir[:-5]
J2caldir = CalDir[1:-4]
J3caldir = CalDir[2:-3]
J4caldir = CalDir[3:-2]
J5caldir = CalDir[4:-1]
J6caldir = CalDir[5:] 

rotary = RotaryIRQ(14, 15)
current_val = 0  # Track the last known value of the encoder
led = Pin(17, Pin.OUT)
ledState = 'LED State Unknown'
button = Pin(16, Pin.IN, Pin.PULL_UP)


J1stepPin = Pin(5, Pin.OUT)
J1dirPin = Pin(6, Pin.OUT)
J2stepPin = Pin(7, Pin.OUT)
J2dirPin = Pin(8, Pin.OUT)
J3stepPin = Pin(9, Pin.OUT)
J3dirPin = Pin(10, Pin.OUT)
J4stepPin = Pin(11, Pin.OUT)
J4dirPin = Pin(12, Pin.OUT)
J5stepPin = Pin(13, Pin.OUT)
J5dirPin = Pin(14, Pin.OUT)
J6stepPin = Pin(15, Pin.OUT)
J6dirPin = Pin(16, Pin.OUT)
TRstepPin = Pin(17, Pin.OUT)
TRdirPin = Pin(18, Pin.OUT)

ssid = ''
password = ''

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(ssid, password)

uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))
uart.init(bits=8, parity=None, stop=2)
#ledx = Pin("LED", Pin.OUT)


# replace the "html" variable with the following to create a more user-friendly control panel
html = """<!DOCTYPE html><html>
<head><meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="icon" href="data:,">
<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}
.buttonGreen { background-color: #4CAF50; border: 2px solid #000000;; color: white; padding: 15px 32px; text-align: center; text-decoration: none; display: inline-block; font-size: 16px; margin: 4px 2px; cursor: pointer; }
.buttonRed { background-color: #D11D53; border: 2px solid #000000;; color: white; padding: 15px 32px; text-align: center; text-decoration: none; display: inline-block; font-size: 16px; margin: 4px 2px; cursor: pointer; }
text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}
</style></head>
<body><center><h1>Control Panel</h1></center><br><br>
<form><center>
<center> <button class="buttonGreen" name="led" value="on" type="submit">LED ON</button>
<br><br>
<center> <button class="buttonRed" name="led" value="off" type="submit">LED OFF</button>
</form>
<br><br>
<br><br>
<p>%s<p></body></html>
"""

# Wait for connect or fail
max_wait = 10
while max_wait > 0:
    if wlan.status() < 0 or wlan.status() >= 3:
        break
    max_wait -= 1
    print('waiting for connection...')
    time.sleep(1)
    
# Handle connection error
if wlan.status() != 3:
    raise RuntimeError('network connection failed')
else:
    print('Connected')
    status = wlan.ifconfig()
    print( 'ip = ' + status[0] )
    
    
# Open socket
addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]
s = socket.socket()
s.bind(addr)
s.listen(1)
print('listening on', addr)

##################################

global JogStepsStat
global xboxUse





def setCom():
  global ser  
  port = "COM5"  
  baud = 115200    
  #ser = serial.Serial(port,baud)



def J1jogNeg():
  print("test2")
  global JogStepsStat
  global J1StepCur
  global J1AngCur
  global xboxUse

  Speed = "25"
  ACCdur = "15"
  ACCspd = "10"
  DECdur = "20"
  DECspd = "5"
  J1Degs = 10
  if JogStepsStat.get() == 0:
    J1jogSteps = int(J1Degs/J1DegPerStep)
  else:
    #switch from degs to steps
    J1jogSteps = J1Degs
    J1Degs = J1Degs*J1DegPerStep

  command = "MJA"+J1motdir+str(J1jogSteps)+"S"+Speed+"G"+ACCdur+"H"+ACCspd+"I"+DECdur+"K"+DECspd+"\n"
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.2)
  ser.read()  

    

    
def J1jogPos():
  print("test1")
  global JogStepsStat
  global J1StepCur
  global J1AngCur
  global xboxUse
    
  Speed = "25"
  ACCdur = "15"
  ACCspd = "10"
  DECdur = "20"
  DECspd = "5"
  J1Degs = 10
  if JogStepsStat.get() == 0:
    J1jogSteps = int(J1Degs/J1DegPerStep)
  else:
    #switch from degs to steps
    J1jogSteps = J1Degs
    J1Degs = J1Degs*J1DegPerStep
  #calc pos dir output
  if (J1motdir == "0"):
    J1drivedir = "1"
  else:
    J1drivedir = "0"	
  
  command = "MJA"+J1drivedir+str(J1jogSteps)+"S"+Speed+"G"+ACCdur+"H"+ACCspd+"I"+DECdur+"K"+DECspd+"\n"
  ser.write(command.encode())    
  ser.flushInput()
  time.sleep(.2)
  ser.read()  
    


def calRobotAll():
  calaxis = "111111"
  speed = "50"
  calRobot(calaxis,speed)
  ### calc correct calibration direction
  if(J1caldir == J1motdir):
    J1caldrive = "1"
  else:
    J1caldrive = "0"      
  if(J2caldir == J2motdir):
    J2caldrive = "1"
  else:
    J2caldrive = "0" 
  if(J3caldir == J3motdir):
    J3caldrive = "1"
  else:
    J3caldrive = "0" 
  if(J4caldir == J4motdir):
    J4caldrive = "1"
  else:
    J4caldrive = "0" 	
  if(J5caldir == J5motdir):
    J5caldrive = "1"
  else:
    J5caldrive = "0" 	
  if(J6caldir == J6motdir):
    J6caldrive = "1"
  else:
    J6caldrive = "0"        
  command = "MJA"+J1caldrive+"500"+"B"+J2caldrive+"500"+"C"+J3caldrive+"500"+"D"+J4caldrive+"500"+"E"+J5caldrive+"500"+"F"+J6caldrive+"500"+"S15G10H10I10K10"+"\n"
  #ser.write(command.encode())
  #ser.flushInput()
  print(command)
  speed = "8"
  time.sleep(2.5)
  calRobot(calaxis,speed)




def calRobot(calaxis,speed):
  J1axis = calaxis[:-5]
  J2axis = calaxis[1:-4]
  J3axis = calaxis[2:-3]
  J4axis = calaxis[3:-2]
  J5axis = calaxis[4:-1]
  J6axis = calaxis[5:]
  ###
  if (J1axis == "1"):
    J1step = str(J1StepLim)
  else:
    J1step = "0"
  if (J2axis == "1"):
    J2step = str(J2StepLim)
  else:
    J2step = "0" 	
  if (J3axis == "1"):
    J3step = str(J3StepLim)
  else:
    J3step = "0" 	
  if (J4axis == "1"):
    J4step = str(J4StepLim)
  else:
    J4step = "0" 
  if (J5axis == "1"):
    J5step = str(J5StepLim)
  else:
    J5step = "0" 
  if (J6axis == "1"):
    J6step = str(J6StepLim)
  else:
    J6step = "0" 	
  ### calc correct calibration direction
  if(J1caldir == J1motdir):
    J1caldrive = "0"
  else:
    J1caldrive = "1"      
  if(J2caldir == J2motdir):
    J2caldrive = "0"
  else:
    J2caldrive = "1" 
  if(J3caldir == J3motdir):
    J3caldrive = "0"
  else:
    J3caldrive = "1" 
  if(J4caldir == J4motdir):
    J4caldrive = "0"
  else:
    J4caldrive = "1" 	
  if(J5caldir == J5motdir):
    J5caldrive = "0"
  else:
    J5caldrive = "1" 	
  if(J6caldir == J6motdir):
    J6caldrive = "0"
  else:
    J6caldrive = "1"    
  command = "LL"+"A"+J1caldrive+J1step+"B"+J2caldrive+J2step+"C"+J3caldrive+J3step+"D"+J4caldrive+J4step+"E"+J5caldrive+J5step+"F"+J6caldrive+J6step+"S"+str(speed)+"\n"  
  #ser.write(command.encode())
  #ser.flushInput()
  print(command)
  calvalue = "P"
  #manEntryField.delete(0, 'end')
  #manEntryField.insert(0,calvalue)
  global calStat
  if (calvalue == "P"):
    calStat = 1
    #calibration.delete(0, END)
    ##J1##
    global J1StepCur
    global J1AngCur
    if (J1axis == "1"):
      if (J1caldir == "0"):
        J1StepCur = 0
        J1AngCur = J1NegAngLim
      else:
        J1StepCur = J1StepLim
        J1AngCur = J1PosAngLim
      #J1curAngEntryField.delete(0, 'end')
      #J1curAngEntryField.insert(0,str(J1AngCur))
    ###########
    ##J2##
    global J2StepCur
    global J2AngCur
    if (J2axis == "1"):
      if (J2caldir == "0"):
        J2StepCur = 0
        J2AngCur = J2NegAngLim
      else:
        J2StepCur = J2StepLim
        J2AngCur = J2PosAngLim
      #J2curAngEntryField.delete(0, 'end')
      #J2curAngEntryField.insert(0,str(J2AngCur))
    ###########
    ##J3##
    global J3StepCur
    global J3AngCur
    if (J3axis == "1"):
      if (J3caldir == "0"):
        J3StepCur = 0
        J3AngCur = J3NegAngLim
      else:
        J3StepCur = J3StepLim
        J3AngCur = J3PosAngLim
      #J3curAngEntryField.delete(0, 'end')
      #J3curAngEntryField.insert(0,str(J3AngCur))
    ###########
    ##J4##
    global J4StepCur
    global J4AngCur
    if (J4axis == "1"):
      if (J4caldir == "0"):
        J4StepCur = 0
        J4AngCur = J4NegAngLim
      else:
        J4StepCur = J4StepLim
        J4AngCur = J4PosAngLim
      #J4curAngEntryField.delete(0, 'end')
      #J4curAngEntryField.insert(0,str(J4AngCur))
    ###########	
    ##J5##
    global J5StepCur
    global J5AngCur
    if (J5axis == "1"):
      if (J5caldir == "0"):
        J5StepCur = 0
        J5AngCur = J5NegAngLim
      else:
        J5StepCur = J5StepLim
        J5AngCur = J5PosAngLim
      #J5curAngEntryField.delete(0, 'end')
      #J5curAngEntryField.insert(0,str(J5AngCur))
    ###########	
    ##J6##
    global J6StepCur
    global J6AngCur
    if (J6axis == "1"):
      if (J6caldir == "0"):
        J6StepCur = 0
        J6AngCur = J6NegAngLim
      else:
        J6StepCur = J6StepLim
        J6AngCur = J6PosAngLim
      #J6curAngEntryField.delete(0, 'end')
      #J6curAngEntryField.insert(0,str(J6AngCur))
    ###########		
    #value=calibration.get(0,END)
    #pickle.dump(value,open("ARbot.cal","wb"))
    print("calibrado")
    #almStatusLab2.config(text='CALIBRATION SUCCESSFUL', bg = "cornflowerblue")	
    #DisplaySteps()
  else:
    if (calvalue == b'F'):
      calStat = 0
      print("calibrado fallo")
      #almStatusLab2.config(text="CALIBRATION FAILED", bg = "red")
    else:
      print("no feedback from arduino")
      #almStatusLab2.config(text="NO CAL FEEDBACK FROM ARDUINO", bg = "red")	  
  #CalcFwdKin()	  
  #savePosData()









 

##############################################################################################################################################################
### KINEMATIC DEFS ######################################################################################################################## KINEMATIC DEFS ###
##############################################################################################################################################################

def gotoFineCalPos():
  command = "Move J [*]  X) 68.76   Y) -0.024   Z) 733.607   W) -89.978   P) 0.95   R) -90.002   T) 40.0   Speed-25 Ad 15 As 10 Dd 20 Ds 5 $N"
  J1newIndex = command.find("X) ")
  J2newIndex = command.find("Y) ")
  J3newIndex = command.find("Z) ")
  J4newIndex = command.find("W) ")
  J5newIndex = command.find("P) ")
  J6newIndex = command.find("R) ")
  TRnewIndex = command.find("T) ")	
  SpeedIndex = command.find("Speed-")
  ACCdurIndex = command.find("Ad")
  ACCspdIndex = command.find("As")
  DECdurIndex = command.find("Dd")
  DECspdIndex = command.find("Ds")
  WristConfIndex = command.find("$")
  CX = float(command[J1newIndex+3:J2newIndex-1])
  CY = float(command[J2newIndex+3:J3newIndex-1])
  CZ = float(command[J3newIndex+3:J4newIndex-1])
  CRx = float(command[J4newIndex+3:J5newIndex-1])
  CRy = float(command[J5newIndex+3:J6newIndex-1])
  CRz = float(command[J6newIndex+3:TRnewIndex-1])
  Track = float(command[TRnewIndex+3:SpeedIndex-1])
  newSpeed = str(command[SpeedIndex+6:ACCdurIndex-1])
  ACCdur = command[ACCdurIndex+3:ACCspdIndex-1]
  ACCspd = command[ACCspdIndex+3:DECdurIndex-1]
  DECdur = command[DECdurIndex+3:DECspdIndex-1]
  DECspd = command[DECspdIndex+3:WristConfIndex-1]
  WC = command[WristConfIndex+1:]
  TCX = 0
  TCY = 0 
  TCZ = 0
  TCRx = 0
  TCRy = 0
  TCRz = 0
  Code = 0  
  MoveXYZ(CX,CY,CZ,CRx,CRy,CRz,newSpeed,ACCdur,ACCspd,DECdur,DECspd,WC,TCX,TCY,TCZ,TCRx,TCRy,TCRz,Track,Code)
  print("a posicion de inicio")
  #almStatusLab2.config(text="MOVED TO FINE CALIBRATION POSITION", bg = "yellow")

  
def CalcRevKin(CX,CY,CZ,CRx,CRy,CRz,WC,TCX,TCY,TCZ,TCRx,TCRy,TCRz):
  global J1out
  global J2out
  global J3out
  global J4out
  global J5out
  global J6out
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  if (J1AngCur == 0):
    J1AngCur = .0001
  if (J2AngCur == 0):
    J2AngCur = .0001
  if (J3AngCur == 0):
    J3AngCur = .0001
  if (J4AngCur == 0):
    J4AngCur = .0001
  if (J5AngCur == 0):
    J5AngCur = .0001
  if (J6AngCur == 0):
    J6AngCur = .0001  
  #input
  O4 = CX
  O5 = CY 
  O6 = CZ 
  O9 = CRx
  O8 = CRy
  O7 = CRz
  V8 = WC
  if (O4 == 0):
    O4 = .0001
  if (O5 == 0):
    O5 = .0001
  if (O6 == 0):
    O6 = .0001
  if (O7 == 0):
    O7 = .0001
  if (O8 == 0):
    O8 = .0001
  if (O9 == 0):
    O9 = .0001  
  #quadrant
  if (O4>0 and O5>0):
    V9 = 1
  elif (O4>0 and O5<0):
    V9 = 2
  elif (O4<0 and O5<0):
    V9 = 3
  elif (O4<0 and O5>0):
    V9 = 4	
  ## DH TABLE
  D13 = math.radians(-90)
  D14 = math.radians(0)
  D15 = math.radians(90)
  D16 = math.radians(-90)
  D17 = math.radians(90)
  D18 = math.radians(0)
  E13 = 169.77
  E14 = 0
  E15 = 0
  E16 = -222.63
  E17 = 0
  E18 = -36.25
  F13 = 64.2
  F14 = 305.0
  F15 = 0
  F16 = 0
  F17 = 0
  F18 = 0	
  ## WORK FRAME INPUT
  H13 = -float(0) 
  H14 = -float(0)  
  H15 = -float(0)
  H16 = -float(0) 
  H17 = -float(0) 
  H18 = -float(0) 
  ## TOOL FRAME INPUT
  J13 = -float(0) + TCX
  J14 = -float(0) + TCY
  J15 = -float(0) + TCZ
  J16 = -float(0) + TCRx
  J17 = -float(0) + TCRy
  J18 = -float(0) + TCRz
  ## WORK FRAME TABLE
  N30 = math.cos(math.radians(H18))*math.cos(math.radians(H17))
  O30 = -math.sin(math.radians(H18))*math.cos(math.radians(H16))+math.cos(math.radians(H18))*math.sin(math.radians(H17))*math.sin(math.radians(H16))
  P30 = math.sin(math.radians(H18))*math.sin(math.radians(H16))+math.cos(math.radians(H18))*math.sin(math.radians(H17))*math.cos(math.radians(H16))
  Q30 = H13
  N31 = math.sin(math.radians(H18))*math.cos(math.radians(H17))
  O31 = math.cos(math.radians(H18))*math.cos(math.radians(H16))+math.sin(math.radians(H18))*math.sin(math.radians(H17))*math.sin(math.radians(H16))
  P31 = -math.cos(math.radians(H18))*math.sin(math.radians(H16))+math.sin(math.radians(H18))*math.sin(math.radians(H17))*math.cos(math.radians(H16))
  Q31 = H14
  N32 = -math.sin(math.radians(H18))
  O32 = math.cos(math.radians(H17))*math.sin(math.radians(H16))
  P32 = math.cos(math.radians(H17))*math.cos(math.radians(H16))
  Q32 = H15
  N33 = 0
  O33 = 0
  P33 = 0
  Q33 = 1   
  ## R 0-T
  X30 = math.cos(math.radians(O7))*math.cos(math.radians(O9))-math.cos(math.radians(O8))*math.sin(math.radians(O7))*math.sin(math.radians(O9)) 
  Y30 = math.cos(math.radians(O9))*math.sin(math.radians(O7))+math.cos(math.radians(O7))*math.cos(math.radians(O8))*math.sin(math.radians(O9))
  Z30 = math.sin(math.radians(O8))*math.sin(math.radians(O9))
  AA30 = O4  
  X31 = math.cos(math.radians(O8))*math.cos(math.radians(O9))*math.sin(math.radians(O7))+math.cos(math.radians(O7))*math.sin(math.radians(O9))
  Y31 = math.cos(math.radians(O7))*math.cos(math.radians(O8))*math.cos(math.radians(O9))-math.sin(math.radians(O7))*math.sin(math.radians(O9))
  Z31 = math.cos(math.radians(O9))*math.sin(math.radians(O8))
  AA31 = O5   	
  X32 = math.sin(math.radians(O7))*math.sin(math.radians(O8))
  Y32 = math.cos(math.radians(O7))*math.sin(math.radians(O8))
  Z32 = -math.cos(math.radians(O8))
  AA32 = O6 
  X33 = 0
  Y33 = 0
  Z33 = 0
  AA33 = 1     
  ## R 0-T   offset by work frame
  X36 = ((N30*X30)+(O30*X31)+(P30*X32)+(Q30*X33))*-1
  Y36 = (N30*Y30)+(O30*Y31)+(P30*Y32)+(Q30*Y33)
  Z36 = (N30*Z30)+(O30*Z31)+(P30*Z32)+(Q30*Z33)
  AA36 = (N30*AA30)+(O30*AA31)+(P30*AA32)+(Q30*AA33)
  X37 = (N31*X30)+(O31*X31)+(P31*X32)+(Q31*X33)
  Y37 = (N31*Y30)+(O31*Y31)+(P31*Y32)+(Q31*Y33)
  Z37 = (N31*Z30)+(O31*Z31)+(P31*Z32)+(Q31*Z33)
  AA37 = (N31*AA30)+(O31*AA31)+(P31*AA32)+(Q31*AA33) 
  X38 = (N32*X30)+(O32*X31)+(P32*X32)+(Q32*X33)
  Y38 = (N32*Y30)+(O32*Y31)+(P32*Y32)+(Q32*Y33)
  Z38 = (N32*Z30)+(O32*Z31)+(P32*Z32)+(Q32*Z33)
  AA38 = (N32*AA30)+(O32*AA31)+(P32*AA32)+(Q32*AA33) 
  X39 = (N33*X30)+(O33*X31)+(P33*X32)+(Q33*X33)
  Y39 = (N33*Y30)+(O33*Y31)+(P33*Y32)+(Q33*Y33)
  Z39 = (N33*Z30)+(O33*Z31)+(P33*Z32)+(Q33*Z33)
  AA39 = (N33*AA30)+(O33*AA31)+(P33*AA32)+(Q33*AA33)
  ## TOOL FRAME
  X42 = math.cos(math.radians(J18))*math.cos(math.radians(J17))
  Y42 = -math.sin(math.radians(J18))*math.cos(math.radians(J16))+math.cos(math.radians(J18))*math.sin(math.radians(J17))*math.sin(math.radians(J16)) 
  Z42 = math.sin(math.radians(J18))*math.sin(math.radians(J16))+math.cos(math.radians(J18))*math.sin(math.radians(J17))*math.cos(math.radians(J16)) 
  AA42 = (J13)
  X43 = math.sin(math.radians(J18))*math.cos(math.radians(J17))
  Y43 = math.cos(math.radians(J18))*math.cos(math.radians(J16))+math.sin(math.radians(J18))*math.sin(math.radians(J17))*math.sin(math.radians(J16)) 
  Z43 = -math.cos(math.radians(J18))*math.sin(math.radians(J16))+math.sin(math.radians(J18))*math.sin(math.radians(J17))*math.cos(math.radians(J16)) 
  AA43 = (J14)  
  X44 = -math.sin(math.radians(J18))
  Y44 = math.cos(math.radians(J17))*math.sin(math.radians(J16)) 
  Z44 = math.cos(math.radians(J17))*math.cos(math.radians(J16)) 
  AA44 = (J15)  
  X45 = 0
  Y45 = 0 
  Z45 = 0 
  AA45 = 1    
  ## INVERT TOOL FRAME
  X48 = X42
  Y48 = X43
  Z48 = X44
  AA48 = (X48*AA42)+(Y48*AA43)+(Z48*AA44)
  X49 = Y42
  Y49 = Y43
  Z49 = Y44
  AA49 = (X49*AA42)+(Y49*AA43)+(Z49*AA44)
  X50 = Z42
  Y50 = Z43
  Z50 = Z44
  AA50 = (X50*AA42)+(Y50*AA43)+(Z50*AA44)
  X51 = 0
  Y51 = 0
  Z51 = 0
  AA51 = 1
  ## R 0-6
  X54 =(X36*X48)+(Y36*X49)+(Z36*X50)+(AA36*X51)
  Y54 =(X36*Y48)+(Y36*Y49)+(Z36*Y50)+(AA36*Y51)
  Z54 =(X36*Z48)+(Y36*Z49)+(Z36*Z50)+(AA36*Z51)
  AA54 =(X36*AA48)+(Y36*AA49)+(Z36*AA50)+(AA36*AA51)
  X55 =(X37*X48)+(Y37*X49)+(Z37*X50)+(AA37*X51)
  Y55 =(X37*Y48)+(Y37*Y49)+(Z37*Y50)+(AA37*Y51)
  Z55 =(X37*Z48)+(Y37*Z49)+(Z37*Z50)+(AA37*Z51)
  AA55 =(X37*AA48)+(Y37*AA49)+(Z37*AA50)+(AA37*AA51)
  X56 =(X38*X48)+(Y38*X49)+(Z38*X50)+(AA38*X51)
  Y56 =(X38*Y48)+(Y38*Y49)+(Z38*Y50)+(AA38*Y51)
  Z56 =(X38*Z48)+(Y38*Z49)+(Z38*Z50)+(AA38*Z51)
  AA56 =(X38*AA48)+(Y38*AA49)+(Z38*AA50)+(AA38*AA51)
  X57 =(X39*X48)+(Y39*X49)+(Z39*X50)+(AA39*X51)
  Y57 =(X39*Y48)+(Y39*Y49)+(Z39*Y50)+(AA39*Y51)
  Z57 =(X39*Z48)+(Y39*Z49)+(Z39*Z50)+(AA39*Z51)
  AA57 =(X39*AA48)+(Y39*AA49)+(Z39*AA50)+(AA39*AA51)
  ## REMOVE R 0-6
  X60 =math.cos(math.radians(180))
  Y60 =math.sin(math.radians(180))
  Z60 = 0
  AA60 = 0
  X61 =-math.sin(math.radians(180))*math.cos(D18)
  Y61 =math.cos(math.radians(180))*math.cos(D18)
  Z61 =math.sin(D18)
  AA61 = 0
  X62 =math.sin(math.radians(180))*math.sin(D18)
  Y62 =-math.cos(math.radians(180))*math.sin(D18)
  Z62 =math.cos(D18)
  AA62 = -E18
  X63 = 0
  Y63 = 0
  Z63 = 0
  AA63 = 1
  ## R 0-5 (center spherica wrist)
  X66 =(X54*X60)+(Y54*X61)+(Z54*X62)+(AA54*X63)
  Y66 =(X54*Y60)+(Y54*Y61)+(Z54*Y62)+(AA54*Y63)
  Z66 =(X54*Z60)+(Y54*Z61)+(Z54*Z62)+(AA54*Z63)
  AA66 =(X54*AA60)+(Y54*AA61)+(Z54*AA62)+(AA54*AA63)
  X67 =(X55*X60)+(Y55*X61)+(Z55*X62)+(AA55*X63)
  Y67 =(X55*Y60)+(Y55*Y61)+(Z55*Y62)+(AA55*Y63)
  Z67 =(X55*Z60)+(Y55*Z61)+(Z55*Z62)+(AA55*Z63)
  AA67 =(X55*AA60)+(Y55*AA61)+(Z55*AA62)+(AA55*AA63)
  X68 =(X56*X60)+(Y56*X61)+(Z56*X62)+(AA56*X63)
  Y68 =(X56*Y60)+(Y56*Y61)+(Z56*Y62)+(AA56*Y63)
  Z68 =(X56*Z60)+(Y56*Z61)+(Z56*Z62)+(AA56*Z63)
  AA68 =(X56*AA60)+(Y56*AA61)+(Z56*AA62)+(AA56*AA63)
  X69 =(X57*X60)+(Y57*X61)+(Z57*X62)+(AA57*X63)
  Y69 =(X57*Y60)+(Y57*Y61)+(Z57*Y62)+(AA57*Y63)
  Z69 =(X57*Z60)+(Y57*Z61)+(Z57*Z62)+(AA57*Z63)
  AA69 =(X57*AA60)+(Y57*AA61)+(Z57*AA62)+(AA57*AA63)
  ## CALCULATE J1 ANGLE
  O13 = math.atan((AA67)/(AA66))
  if (V9 == 1):
    P13 = math.degrees(O13)
  if (V9 == 2):
    P13 = math.degrees(O13)
  if (V9 == 3):
    P13 = -180 + math.degrees(O13)
  if (V9 == 4):
    P13 = 180 + math.degrees(O13)
  ## CALCULATE J2 ANGLE	FWD

  O16 = math.sqrt(((abs(AA67))**2)+((abs(AA66))**2))
  O17 = AA68-E13
  O18 = O16-F13
  O19 = math.sqrt((O17**2)+(O18**2))
  O20 = math.sqrt((E16**2)+(F15**2))
  O21 = math.degrees(math.atan(O17/O18))
  O22 = math.degrees(math.acos(((F14**2)+(O19**2)-(abs(O20)**2))/(2*F14*O19)))
  try:
    O25 = math.degrees(math.atan(abs(E16)/F15))
  except:
    O25 = 90
  O23 = 180-math.degrees(math.acos(((abs(O20)**2)+(F14**2)-(O19**2))/(2*abs(O20)*F14)))+(90-O25)
  O26 = -(O21+O22)
  O27 = O23
  ## CALCULATE J2 ANGLE	MID
  P18 = -O18
  P19 = math.sqrt((O17**2)+(P18**2))   
  P21 = math.degrees(math.acos(((F14**2)+(P19**2)-(abs(O20)**2))/(2*F14*P19)))
  P22 = math.degrees(math.atan(P18/O17))
  P23 = 180-math.degrees(math.acos(((abs(O20)**2)+(F14**2)-(P19**2))/(2*abs(O20)*F14)))+(90-O25)
  P24 = 90-(P21+P22)
  P26 = -180+P24
  P27 = P23
  ## J1,J2,J3
  Q4 = P13
  if (O18<0):
    Q5 = P26
    Q6 = P27
  else:
    Q5 = O26
    Q6 = O27
  ## J1
  N36 =math.cos(math.radians(Q4))
  O36 =-math.sin(math.radians(Q4))*math.cos(D13)
  P36 =math.sin(math.radians(Q4))*math.sin(D13)
  Q36 =F13*math.cos(math.radians(Q4))
  N37 =math.sin(math.radians(Q4))
  O37 =math.cos(math.radians(Q4))*math.cos(D13)
  P37 =-math.cos(math.radians(Q4))*math.sin(D13)
  Q37 =F13*math.sin(math.radians(Q4)) 
  N38 = 0
  O38 =math.sin(D13)
  P38 =math.cos(D13)
  Q38 =E13  
  N39 = 0
  O39 = 0
  P39 = 0
  Q39 = 1
  ## J2
  N42 =math.cos(math.radians(Q5))
  O42 =-math.sin(math.radians(Q5))*math.cos(D14)
  P42 =math.sin(math.radians(Q5))*math.sin(D14)
  Q42 =F14*math.cos(math.radians(Q5))  
  N43 =math.sin(math.radians(Q5))
  O43 =math.cos(math.radians(Q5))*math.cos(D14)
  P43 =-math.cos(math.radians(Q5))*math.sin(D14)
  Q43 =F14*math.sin(math.radians(Q5))  
  N44 = 0
  O44 =math.sin(D14)
  P44 =math.cos(D14)
  Q44 =E14  
  N45 = 0
  O45 = 0
  P45 = 0
  Q45 = 1
  ## J3  
  N48 =math.cos(math.radians((Q6)-90))
  O48 =-math.sin(math.radians((Q6)-90))*math.cos(D15)
  P48 =math.sin(math.radians((Q6)-90))*math.sin(D15)
  Q48 =F15*math.cos(math.radians((Q6)-90))
  N49 =math.sin(math.radians((Q6)-90))
  O49 =math.cos(math.radians((Q6)-90))*math.cos(D15)
  P49 =-math.cos(math.radians((Q6)-90))*math.sin(D15)
  Q49 =F15*math.sin(math.radians((Q6)-90))
  N50 =0
  O50 =math.sin(D15)
  P50 =math.cos(D15)
  Q50 =E15
  N51 =0
  O51 =0
  P51 =0
  Q51 =0 
  ## R 0-1
  S33 =(N30*N36)+(O30*N37)+(P30*N38)+(Q30*N39)
  T33 =(N30*O36)+(O30*O37)+(P30*O38)+(Q30*O39)
  U33 =(N30*P36)+(O30*P37)+(P30*P38)+(Q30*P39)
  V33 =(N30*Q36)+(O30*Q37)+(P30*Q38)+(Q30*Q39) 
  S34 =(N31*N36)+(O31*N37)+(P31*N38)+(Q31*N39)
  T34 =(N31*O36)+(O31*O37)+(P31*O38)+(Q31*O39)
  U34 =(N31*P36)+(O31*P37)+(P31*P38)+(Q31*P39)
  V34 =(N31*Q36)+(O31*Q37)+(P31*Q38)+(Q31*Q39)
  S35 =(N32*N36)+(O32*N37)+(P32*N38)+(Q32*N39)
  T35 =(N32*O36)+(O32*O37)+(P32*O38)+(Q32*O39)
  U35 =(N32*P36)+(O32*P37)+(P32*P38)+(Q32*P39)
  V35 =(N32*Q36)+(O32*Q37)+(P32*Q38)+(Q32*Q39)
  S36 =(N33*N36)+(O33*N37)+(P33*N38)+(Q33*N39)
  T36 =(N33*O36)+(O33*O37)+(P33*O38)+(Q33*O39)
  U36 =(N33*P36)+(O33*P37)+(P33*P38)+(Q33*P39)
  V36 =(N33*Q36)+(O33*Q37)+(P33*Q38)+(Q33*Q39)
  ## R 0-2  
  S39 =(S33*N42)+(T33*N43)+(U33*N44)+(V33*N45)
  T39 =(S33*O42)+(T33*O43)+(U33*O44)+(V33*O45)
  U39 =(S33*P42)+(T33*P43)+(U33*P44)+(V33*P45)
  V39 =(S33*Q42)+(T33*Q43)+(U33*Q44)+(V33*Q45)  
  S40 =(S34*N42)+(T34*N43)+(U34*N44)+(V34*N45)
  T40 =(S34*O42)+(T34*O43)+(U34*O44)+(V34*O45)
  U40 =(S34*P42)+(T34*P43)+(U34*P44)+(V34*P45)
  V40 =(S34*Q42)+(T34*Q43)+(U34*Q44)+(V34*Q45)
  S41 =(S35*N42)+(T35*N43)+(U35*N44)+(V35*N45)
  T41 =(S35*O42)+(T35*O43)+(U35*O44)+(V35*O45)
  U41 =(S35*P42)+(T35*P43)+(U35*P44)+(V35*P45)
  V41 =(S35*Q42)+(T35*Q43)+(U35*Q44)+(V35*Q45)
  S42 =(S36*N42)+(T36*N43)+(U36*N44)+(V36*N45)
  T42 =(S36*O42)+(T36*O43)+(U36*O44)+(V36*O45)
  U42 =(S36*P42)+(T36*P43)+(U36*P44)+(V36*P45)
  V42 =(S36*Q42)+(T36*Q43)+(U36*Q44)+(V36*Q45)
  ## R 0-3 
  S45 =(S39*N48)+(T39*N49)+(U39*N50)+(V39*N51)
  T45 =(S39*O48)+(T39*O49)+(U39*O50)+(V39*O51)
  U45 =(S39*P48)+(T39*P49)+(U39*P50)+(V39*P51)
  V45 =(S39*Q48)+(T39*Q49)+(U39*Q50)+(V39*Q51) 
  S46 =(S40*N48)+(T40*N49)+(U40*N50)+(V40*N51)
  T46 =(S40*O48)+(T40*O49)+(U40*O50)+(V40*O51)
  U46 =(S40*P48)+(T40*P49)+(U40*P50)+(V40*P51)
  V46 =(S40*Q48)+(T40*Q49)+(U40*Q50)+(V40*Q51)
  S47 =(S41*N48)+(T41*N49)+(U41*N50)+(V41*N51)
  T47 =(S41*O48)+(T41*O49)+(U41*O50)+(V41*O51)
  U47 =(S41*P48)+(T41*P49)+(U41*P50)+(V41*P51)
  V47 =(S41*Q48)+(T41*Q49)+(U41*Q50)+(V41*Q51)
  S48 =(S42*N48)+(T42*N49)+(U42*N50)+(V42*N51)
  T48 =(S42*O48)+(T42*O49)+(U42*O50)+(V42*O51)
  U48 =(S42*P48)+(T42*P49)+(U42*P50)+(V42*P51)
  V48 =(S42*Q48)+(T42*Q49)+(U42*Q50)+(V42*Q51)
  ## R 0-3 transposed
  S51 =S45
  T51 =S46
  U51 =S47
  S52 =T45
  T52 =T46
  U52 =T47
  S53 =U45
  T53 =U46
  U53 =U47
  ## R 3-6 (spherical wrist  orietation)
  X72 =(S51*X66)+(T51*X67)+(U51*X68) 
  Y72 =(S51*Y66)+(T51*Y67)+(U51*Y68)
  Z72 =(S51*Z66)+(T51*Z67)+(U51*Z68)
  X73 =(S52*X66)+(T52*X67)+(U52*X68)
  Y73 =(S52*Y66)+(T52*Y67)+(U52*Y68)
  Z73 =(S52*Z66)+(T52*Z67)+(U52*Z68)
  X74 =(S53*X66)+(T53*X67)+(U53*X68)
  Y74 =(S53*Y66)+(T53*Y67)+(U53*Y68)
  Z74 =(S53*Z66)+(T53*Z67)+(U53*Z68)
  ## WRIST ORENTATION
  R7 = math.degrees(math.atan2(Z73,Z72))
  R8 = math.degrees(math.atan2(+math.sqrt(1-Z74**2),Z74))
  if (Y74 < 0):
    R9 = math.degrees(math.atan2(-Y74,X74))-180
  else:
    R9 = math.degrees(math.atan2(-Y74,X74))+180  	
  S7 = math.degrees(math.atan2(-Z73,-Z72))
  S8 = math.degrees(math.atan2(-math.sqrt(1-Z74**2),Z74))
  if (Y74 < 0):
    S9 = math.degrees(math.atan2(Y74,-X74))+180
  else:
    S9 = math.degrees(math.atan2(Y74,-X74))-180
  if (V8 == "F"):
    Q8 = R8
  else:
    Q8 = S8
  if(Q8>0):
    Q7 = R7 
  else:
    Q7 = S7
  if(Q8<0):
    Q9 = S9
  else:
    Q9 = R9
  ## FINAL OUTPUT
  J1out = Q4
  J2out = Q5
  J3out = Q6
  J4out = Q7
  J5out = Q8
  J6out = Q9
  return (J1out,J2out,J3out,J4out,J5out,J6out)
##############################################################################################################################################################  
### MOVE DEFS ################################################################################################################################## MOVE DEFS ###  
##############################################################################################################################################################  

def MoveXYZ(CX,CY,CZ,CRx,CRy,CRz,newSpeed,ACCdur,ACCspd,DECdur,DECspd,WC,TCX,TCY,TCZ,TCRx,TCRy,TCRz,Track,Code):
  global commandCalc
  CalcRevKin(CX,CY,CZ,CRx,CRy,CRz,WC,TCX,TCY,TCZ,TCRx,TCRy,TCRz)
  MoveNew(J1out,J2out,J3out,J4out,J5out,J6out,newSpeed,ACCdur,ACCspd,DECdur,DECspd,Track,Code)
  if Code == 2:
    return(commandCalc)

def driveMotorsJ(J1dir,J2dir,J3dir,J4dir,J5dir,J6dir,TRdir,J1step,J2step,J3step,J4step,J5step,J6step,TRstep,SpeedIn,ACCdur,ACCspd,DCCdur,DCCspd):
          print("hola mundo")
          HighStep = J1step
          if (J2step > HighStep):          
            HighStep = J2step
          
          if (J3step > HighStep):         
            HighStep = J3step
          
          if (J4step > HighStep):          
            HighStep = J4step
          
          if (J5step > HighStep):         
            HighStep = J5step
          
          if (J6step > HighStep):          
            HighStep = J6step
          
          if (TRstep > HighStep):          
            HighStep = TRstep

          #FIND ACTIVE JOINTS
          J1active = 0
          J2active = 0
          J3active = 0
          J4active = 0
          J5active = 0
          J6active = 0
          TRactive = 0
          Jactive = 0

          if (J1step >= 1):
          
            J1active = 1;
          
          if (J2step >= 1):
          
            J2active = 1;
          
          if (J3step >= 1):
          
            J3active = 1;
          
          if (J4step >= 1):
          
            J4active = 1;
          
          if (J5step >= 1):
          
            J5active = 1;
          
          if (J6step >= 1):
          
            J6active = 1;
          
          if (TRstep >= 1):
          
            TRactive = 1;
          
          Jactive = (J1active + J2active + J3active + J4active + J5active + J6active + TRactive);

          J1_PE = 0;
          J2_PE = 0;
          J3_PE = 0;
          J4_PE = 0;
          J5_PE = 0;
          J6_PE = 0;
          TR_PE = 0;

          J1_SE_1 = 0;
          J2_SE_1 = 0;
          J3_SE_1 = 0;
          J4_SE_1 = 0;
          J5_SE_1 = 0;
          J6_SE_1 = 0;
          TR_SE_1 = 0;

          J1_SE_2 = 0;
          J2_SE_2 = 0;
          J3_SE_2 = 0;
          J4_SE_2 = 0;
          J5_SE_2 = 0;
          J6_SE_2 = 0;
          TR_SE_2 = 0;

          J1_LO_1 = 0;
          J2_LO_1 = 0;
          J3_LO_1 = 0;
          J4_LO_1 = 0;
          J5_LO_1 = 0;
          J6_LO_1 = 0;
          TR_LO_1 = 0;

          J1_LO_2 = 0;
          J2_LO_2 = 0;
          J3_LO_2 = 0;
          J4_LO_2 = 0;
          J5_LO_2 = 0;
          J6_LO_2 = 0;
          TR_LO_2 = 0;

          #reset
          J1cur = 0;
          J2cur = 0;
          J3cur = 0;
          J4cur = 0;
          J5cur = 0;
          J6cur = 0;
          TRcur = 0;

          J1_PEcur = 0;
          J2_PEcur = 0;
          J3_PEcur = 0;
          J4_PEcur = 0;
          J5_PEcur = 0;
          J6_PEcur = 0;
          TR_PEcur = 0;

          J1_SE_1cur = 0;
          J2_SE_1cur = 0;
          J3_SE_1cur = 0;
          J4_SE_1cur = 0;
          J5_SE_1cur = 0;
          J6_SE_1cur = 0;
          TR_SE_1cur = 0;

          J1_SE_2cur = 0;
          J2_SE_2cur = 0;
          J3_SE_2cur = 0;
          J4_SE_2cur = 0;
          J5_SE_2cur = 0;
          J6_SE_2cur = 0;
          TR_SE_2cur = 0;

          highStepCur = 0;
          curDelay = 0;

          #SET DIRECTIONS

          ##////// J1 /////////
          if (J1dir == 1 & J1rotdir == 1):
            J1dirPin.value(0)
            #digitalWrite(J1dirPin, LOW);
          
          elif (J1dir == 1 & J1rotdir == 0):
            J1dirPin.value(1)
            #digitalWrite(J1dirPin, HIGH);
          
          elif (J1dir == 0 & J1rotdir == 1):
            J1dirPin.value(1)
            #digitalWrite(J1dirPin, HIGH);
          
          elif (J1dir == 0 & J1rotdir == 0):
            J1dirPin.value(0)
            #digitalWrite(J1dirPin, LOW);
          

          ##/////// J2 /////////
          if (J2dir == 1 & J2rotdir == 1):
            J2dirPin.value(0)
            #digitalWrite(J2dirPin, LOW);
          
          elif (J2dir == 1 & J2rotdir == 0):
            J2dirPin.value(1)
            #digitalWrite(J2dirPin, HIGH);
          
          elif (J2dir == 0 & J2rotdir == 1):
            J2dirPin.value(1)
            #digitalWrite(J2dirPin, HIGH);
          
          elif (J2dir == 0 & J2rotdir == 0):
            J2dirPin.value(0)
            #digitalWrite(J2dirPin, LOW);
          

          ##/////// J3 /////////
          if (J3dir == 1 & J3rotdir == 1):
            J3dirPin.value(0)
            #digitalWrite(J3dirPin, LOW);
          
          elif (J3dir == 1 & J3rotdir == 0):
            J3dirPin.value(1)
            #digitalWrite(J3dirPin, HIGH);
          
          elif (J3dir == 0 & J3rotdir == 1):
            J3dirPin.value(1)
            #digitalWrite(J3dirPin, HIGH);
          
          elif (J3dir == 0 & J3rotdir == 0):
            J3dirPin.value(0)
            #digitalWrite(J3dirPin, LOW);
          

          ##/////// J4 /////////
          if (J4dir == 1 & J4rotdir == 1):
            J4dirPin.value(0)
            #digitalWrite(J4dirPin, LOW);
          
          elif (J4dir == 1 & J4rotdir == 0):
            J4dirPin.value(1)
            #digitalWrite(J4dirPin, HIGH);
          
          elif (J4dir == 0 & J4rotdir == 1):
            J4dirPin.value(1)
            #digitalWrite(J4dirPin, HIGH);
          
          elif (J4dir == 0 & J4rotdir == 0):
            J4dirPin.value(0)
            #digitalWrite(J4dirPin, LOW);
          

          ##/////// J5 /////////
          if (J5dir == 1 & J5rotdir == 1):
            J5dirPin.value(0)
            #digitalWrite(J5dirPin, LOW);
          
          elif (J5dir == 1 & J5rotdir == 0):
            J5dirPin.value(1)
            #digitalWrite(J5dirPin, HIGH);
          
          elif (J5dir == 0 & J5rotdir == 1):
            J5dirPin.value(1)
            #digitalWrite(J5dirPin, HIGH);
          
          elif (J5dir == 0 & J5rotdir == 0):
            J5dirPin.value(0)
            #digitalWrite(J5dirPin, LOW);
          

          ##/////// J6 /////////
          if (J6dir == 1 & J6rotdir == 1):
            J6dirPin.value(0)
            #digitalWrite(J6dirPin, LOW);
          
          elif (J6dir == 1 & J6rotdir == 0):
            J6dirPin.value(1)
            #digitalWrite(J6dirPin, HIGH);
          
          elif (J6dir == 0 & J6rotdir == 1):
            J6dirPin.value(1)
            #digitalWrite(J6dirPin, HIGH);
          
          elif (J6dir == 0 & J6rotdir == 0):
            J6dirPin.value(0)
            #digitalWrite(J6dirPin, LOW);
          

          ##/////// TRACK /////////
          if (TRdir == 1 & TRACKrotdir == 1):
            TRdirPin.value(0)
            #digitalWrite(TRdirPin, LOW);
          
          elif (TRdir == 1 & TRACKrotdir == 0):
            TRdirPin.value(1)
            #digitalWrite(TRdirPin, HIGH);
          
          elif (TRdir == 0 & TRACKrotdir == 1):
            TRdirPin.value(1)
            #digitalWrite(TRdirPin, HIGH);
          
          elif (TRdir == 0 & TRACKrotdir == 0):
            TRdirPin.value(0)
            #digitalWrite(TRdirPin, LOW);

          
          #/////CALC SPEEDS//////
          ACCStep = (HighStep * (ACCdur / 100));
          DCCStep = HighStep - (HighStep * (DCCdur / 100));
          AdjSpeed = (SpeedIn / 100);
          #//REG SPEED
          CalcRegSpeed = (SpeedMult / AdjSpeed);
          REGSpeed = int(CalcRegSpeed);

          ##//ACC SPEED
          ACCspdT = (ACCspd / 100);
          CalcACCSpeed = ((SpeedMult + (SpeedMult / ACCspdT)) / AdjSpeed);
          ACCSpeed = (CalcACCSpeed);
          ACCinc = (REGSpeed - ACCSpeed) / ACCStep;

          ##//DCC SPEED
          DCCspdT = (DCCspd / 100);
          CalcDCCSpeed = ((SpeedMult + (SpeedMult / DCCspdT)) / AdjSpeed);
          DCCSpeed = (CalcDCCSpeed);
          DCCinc = (REGSpeed + DCCSpeed) / DCCStep;
          DCCSpeed = REGSpeed;

          #///// DRIVE MOTORS /////
          while (J1cur < J1step | J2cur < J2step | J3cur < J3step | J4cur < J4step | J5cur < J5step | J6cur < J6step | TRcur < TRstep):
        

            ##////DELAY CALC/////
            if (highStepCur <= ACCStep):
            
                curDelay = (ACCSpeed / Jactive);
                ACCSpeed = ACCSpeed + ACCinc;
            
            elif (highStepCur >= DCCStep):
            
                curDelay = (DCCSpeed / Jactive);
                DCCSpeed = DCCSpeed + DCCinc;
            
            else:
                
                curDelay = (REGSpeed / Jactive);
                

            ##/////// J1 ////////////////////////////////
            ##///find pulse every
            if (J1cur < J1step):
                
                J1_PE = (HighStep / J1step);
                ##///find left over 1
                J1_LO_1 = (HighStep - (J1step * J1_PE));
                ##///find skip 1
                if (J1_LO_1 > 0):
                
                    J1_SE_1 = (HighStep / J1_LO_1);
                
                else:
                
                    J1_SE_1 = 0;
                
                ##///find left over 2
                if (J1_SE_1 > 0):
                
                    J1_LO_2 = HighStep - ((J1step * J1_PE) + ((J1step * J1_PE) / J1_SE_1));
                
                else:
                
                    J1_LO_2 = 0;
                
                ##///find skip 2
                if (J1_LO_2 > 0):
                
                    J1_SE_2 = (HighStep / J1_LO_2);
                
                else:
                
                    J1_SE_2 = 0;
                
                ##/////////  J1  ///////////////
                if (J1_SE_2 == 0):
                
                    J1_SE_2cur = (J1_SE_2 + 1);
                
                if (J1_SE_2cur != J1_SE_2):
                
                    J1_SE_2cur += 1
                    if (J1_SE_1 == 0):
                    
                        J1_SE_1cur = (J1_SE_1 + 1);
                    
                    if (J1_SE_1cur != J1_SE_1):
                        
                        J1_SE_1cur += 1
                        J1_PEcur += 1
                        if (J1_PEcur == J1_PE):
                        
                            J1cur += 1
                            J1_PEcur = 0;
                            J1stepPin.value(0)
                            time.sleep(curDelay / 1000000)
                            J1stepPin.value(1)
                            #digitalWrite(J1stepPin, LOW);
                            #delayMicroseconds(curDelay);
                            #digitalWrite(J1stepPin, HIGH);
                        
                        
                    else:
                        
                        J1_SE_1cur = 0;
                        
                
                else:
                
                    J1_SE_2cur = 0;
                
                

            #/////// J2 ////////////////////////////////
            #///find pulse every
            if (J2cur < J2step):
                
                J2_PE = (HighStep / J2step);
                #///find left over 1
                J2_LO_1 = (HighStep - (J2step * J2_PE));
                ///find skip 1
                if (J2_LO_1 > 0)
                {
                    J2_SE_1 = (HighStep / J2_LO_1);
                }
                else
                {
                    J2_SE_1 = 0;
                }
                ///find left over 2
                if (J2_SE_1 > 0)
                {
                    J2_LO_2 = HighStep - ((J2step * J2_PE) + ((J2step * J2_PE) / J2_SE_1));
                }
                else
                {
                    J2_LO_2 = 0;
                }
                ///find skip 2
                if (J2_LO_2 > 0)
                {
                    J2_SE_2 = (HighStep / J2_LO_2);
                }
                else
                {
                    J2_SE_2 = 0;
                }
                /////////  J2  ///////////////
                if (J2_SE_2 == 0)
                {
                    J2_SE_2cur = (J2_SE_2 + 1);
                }
                if (J2_SE_2cur != J2_SE_2)
                {
                    J2_SE_2cur = ++J2_SE_2cur;
                    if (J2_SE_1 == 0)
                    {
                    J2_SE_1cur = (J2_SE_1 + 1);
                    }
                    if (J2_SE_1cur != J2_SE_1)
                    {
                    J2_SE_1cur = ++J2_SE_1cur;
                    J2_PEcur = ++J2_PEcur;
                    if (J2_PEcur == J2_PE)
                    {
                        J2cur = ++J2cur;
                        J2_PEcur = 0;
                        digitalWrite(J2stepPin, LOW);
                        delayMicroseconds(curDelay);
                        digitalWrite(J2stepPin, HIGH);
                    }
                    }
                    else
                    {
                    J2_SE_1cur = 0;
                    }
                }
                else
                {
                    J2_SE_2cur = 0;
                }
                

            /////// J3 ////////////////////////////////
            ///find pulse every
            if (J3cur < J3step)
            {
            J3_PE = (HighStep / J3step);
            ///find left over 1
            J3_LO_1 = (HighStep - (J3step * J3_PE));
            ///find skip 1
            if (J3_LO_1 > 0)
            {
                J3_SE_1 = (HighStep / J3_LO_1);
            }
            else
            {
                J3_SE_1 = 0;
            }
            ///find left over 2
            if (J3_SE_1 > 0)
            {
                J3_LO_2 = HighStep - ((J3step * J3_PE) + ((J3step * J3_PE) / J3_SE_1));
            }
            else
            {
                J3_LO_2 = 0;
            }
            ///find skip 2
            if (J3_LO_2 > 0)
            {
                J3_SE_2 = (HighStep / J3_LO_2);
            }
            else
            {
                J3_SE_2 = 0;
            }
            /////////  J3  ///////////////
            if (J3_SE_2 == 0)
            {
                J3_SE_2cur = (J3_SE_2 + 1);
            }
            if (J3_SE_2cur != J3_SE_2)
            {
                J3_SE_2cur = ++J3_SE_2cur;
                if (J3_SE_1 == 0)
                {
                J3_SE_1cur = (J3_SE_1 + 1);
                }
                if (J3_SE_1cur != J3_SE_1)
                {
                J3_SE_1cur = ++J3_SE_1cur;
                J3_PEcur = ++J3_PEcur;
                if (J3_PEcur == J3_PE)
                {
                    J3cur = ++J3cur;
                    J3_PEcur = 0;
                    digitalWrite(J3stepPin, LOW);
                    delayMicroseconds(curDelay);
                    digitalWrite(J3stepPin, HIGH);
                }
                }
                else
                {
                J3_SE_1cur = 0;
                }
            }
            else
            {
                J3_SE_2cur = 0;
            }
            }

            /////// J4 ////////////////////////////////
            ///find pulse every
            if (J4cur < J4step)
            {
            J4_PE = (HighStep / J4step);
            ///find left over 1
            J4_LO_1 = (HighStep - (J4step * J4_PE));
            ///find skip 1
            if (J4_LO_1 > 0)
            {
                J4_SE_1 = (HighStep / J4_LO_1);
            }
            else
            {
                J4_SE_1 = 0;
            }
            ///find left over 2
            if (J4_SE_1 > 0)
            {
                J4_LO_2 = HighStep - ((J4step * J4_PE) + ((J4step * J4_PE) / J4_SE_1));
            }
            else
            {
                J4_LO_2 = 0;
            }
            ///find skip 2
            if (J4_LO_2 > 0)
            {
                J4_SE_2 = (HighStep / J4_LO_2);
            }
            else
            {
                J4_SE_2 = 0;
            }
            /////////  J4  ///////////////
            if (J4_SE_2 == 0)
            {
                J4_SE_2cur = (J4_SE_2 + 1);
            }
            if (J4_SE_2cur != J4_SE_2)
            {
                J4_SE_2cur = ++J4_SE_2cur;
                if (J4_SE_1 == 0)
                {
                J4_SE_1cur = (J4_SE_1 + 1);
                }
                if (J4_SE_1cur != J4_SE_1)
                {
                J4_SE_1cur = ++J4_SE_1cur;
                J4_PEcur = ++J4_PEcur;
                if (J4_PEcur == J4_PE)
                {
                    J4cur = ++J4cur;
                    J4_PEcur = 0;
                    digitalWrite(J4stepPin, LOW);
                    delayMicroseconds(curDelay);
                    digitalWrite(J4stepPin, HIGH);
                }
                }
                else
                {
                J4_SE_1cur = 0;
                }
            }
            else
            {
                J4_SE_2cur = 0;
            }
            }

            /////// J5 ////////////////////////////////
            ///find pulse every
            if (J5cur < J5step)
            {
            J5_PE = (HighStep / J5step);
            ///find left over 1
            J5_LO_1 = (HighStep - (J5step * J5_PE));
            ///find skip 1
            if (J5_LO_1 > 0)
            {
                J5_SE_1 = (HighStep / J5_LO_1);
            }
            else
            {
                J5_SE_1 = 0;
            }
            ///find left over 2
            if (J5_SE_1 > 0)
            {
                J5_LO_2 = HighStep - ((J5step * J5_PE) + ((J5step * J5_PE) / J5_SE_1));
            }
            else
            {
                J5_LO_2 = 0;
            }
            ///find skip 2
            if (J5_LO_2 > 0)
            {
                J5_SE_2 = (HighStep / J5_LO_2);
            }
            else
            {
                J5_SE_2 = 0;
            }
            /////////  J5  ///////////////
            if (J5_SE_2 == 0)
            {
                J5_SE_2cur = (J5_SE_2 + 1);
            }
            if (J5_SE_2cur != J5_SE_2)
            {
                J5_SE_2cur = ++J5_SE_2cur;
                if (J5_SE_1 == 0)
                {
                J5_SE_1cur = (J5_SE_1 + 1);
                }
                if (J5_SE_1cur != J5_SE_1)
                {
                J5_SE_1cur = ++J5_SE_1cur;
                J5_PEcur = ++J5_PEcur;
                if (J5_PEcur == J5_PE)
                {
                    J5cur = ++J5cur;
                    J5_PEcur = 0;
                    digitalWrite(J5stepPin, LOW);
                    delayMicroseconds(curDelay);
                    digitalWrite(J5stepPin, HIGH);
                }
                }
                else
                {
                J5_SE_1cur = 0;
                }
            }
            else
            {
                J5_SE_2cur = 0;
            }
            }

            /////// J6 ////////////////////////////////
            ///find pulse every
            if (J6cur < J6step)
            {
            J6_PE = (HighStep / J6step);
            ///find left over 1
            J6_LO_1 = (HighStep - (J6step * J6_PE));
            ///find skip 1
            if (J6_LO_1 > 0)
            {
                J6_SE_1 = (HighStep / J6_LO_1);
            }
            else
            {
                J6_SE_1 = 0;
            }
            ///find left over 2
            if (J6_SE_1 > 0)
            {
                J6_LO_2 = HighStep - ((J6step * J6_PE) + ((J6step * J6_PE) / J6_SE_1));
            }
            else
            {
                J6_LO_2 = 0;
            }
            ///find skip 2
            if (J6_LO_2 > 0)
            {
                J6_SE_2 = (HighStep / J6_LO_2);
            }
            else
            {
                J6_SE_2 = 0;
            }
            /////////  J6  ///////////////
            if (J6_SE_2 == 0)
            {
                J6_SE_2cur = (J6_SE_2 + 1);
            }
            if (J6_SE_2cur != J6_SE_2)
            {
                J6_SE_2cur = ++J6_SE_2cur;
                if (J6_SE_1 == 0)
                {
                J6_SE_1cur = (J6_SE_1 + 1);
                }
                if (J6_SE_1cur != J6_SE_1)
                {
                J6_SE_1cur = ++J6_SE_1cur;
                J6_PEcur = ++J6_PEcur;
                if (J6_PEcur == J6_PE)
                {
                    J6cur = ++J6cur;
                    J6_PEcur = 0;
                    digitalWrite(J6stepPin, LOW);
                    delayMicroseconds(curDelay);
                    digitalWrite(J6stepPin, HIGH);
                }
                }
                else
                {
                J6_SE_1cur = 0;
                }
            }
            else
            {
                J6_SE_2cur = 0;
            }
            }

            /////// TR ////////////////////////////////
            ///find pulse every
            if (TRcur < TRstep)
            {
            TR_PE = (HighStep / TRstep);
            ///find left over 1
            TR_LO_1 = (HighStep - (TRstep * TR_PE));
            ///find skip 1
            if (TR_LO_1 > 0)
            {
                TR_SE_1 = (HighStep / TR_LO_1);
            }
            else
            {
                TR_SE_1 = 0;
            }
            ///find left over 2
            if (TR_SE_1 > 0)
            {
                TR_LO_2 = HighStep - ((TRstep * TR_PE) + ((TRstep * TR_PE) / TR_SE_1));
            }
            else
            {
                TR_LO_2 = 0;
            }
            ///find skip 2
            if (TR_LO_2 > 0)
            {
                TR_SE_2 = (HighStep / TR_LO_2);
            }
            else
            {
                TR_SE_2 = 0;
            }
            /////////  TR  ///////////////
            if (TR_SE_2 == 0)
            {
                TR_SE_2cur = (TR_SE_2 + 1);
            }
            if (TR_SE_2cur != TR_SE_2)
            {
                TR_SE_2cur = ++TR_SE_2cur;
                if (TR_SE_1 == 0)
                {
                TR_SE_1cur = (TR_SE_1 + 1);
                }
                if (TR_SE_1cur != TR_SE_1)
                {
                TR_SE_1cur = ++TR_SE_1cur;
                TR_PEcur = ++TR_PEcur;
                if (TR_PEcur == TR_PE)
                {
                    TRcur = ++TRcur;
                    TR_PEcur = 0;
                    digitalWrite(TRstepPin, LOW);
                    delayMicroseconds(curDelay);
                    digitalWrite(TRstepPin, HIGH);
                }
                }
                else
                {
                TR_SE_1cur = 0;
                }
            }
            else
            {
                TR_SE_2cur = 0;
            }
            }


            // inc cur step
            highStepCur = ++highStepCur;


        
          

def MoveNew(J1out,J2out,J3out,J4out,J5out,J6out,newSpeed,ACCdur,ACCspd,DECdur,DECspd,Track,Code):
  global xboxUse
  #if xboxUse != 1:
  #  almStatusLab.config(text="SYSTEM READY", bg = "cornflowerblue")
    #almStatusLab2.config(text="SYSTEM READY", bg = "cornflowerblue")
  global J1AngCur
  global J2AngCur
  global J3AngCur
  global J4AngCur
  global J5AngCur
  global J6AngCur
  global J1StepCur
  global J2StepCur
  global J3StepCur
  global J4StepCur
  global J5StepCur
  global J6StepCur
  global TrackcurPos
  global TrackLength
  global TrackStepLim
  global commandCalc  
  J1newAng = J1out
  J2newAng = J2out
  J3newAng = J3out
  J4newAng = J4out
  J5newAng = J5out
  J6newAng = J6out
  TrackNew = Track
  ###CHECK WITHIN ANGLE LIMITS
  if (J1newAng < J1NegAngLim or J1newAng > J1PosAngLim) or (J2newAng < J2NegAngLim or J2newAng > J2PosAngLim) or (J3newAng < J3NegAngLim or J3newAng > J3PosAngLim) or (J4newAng < J4NegAngLim or J4newAng > J4PosAngLim) or (J5newAng < J5NegAngLim or J5newAng > J5PosAngLim) or (J6newAng < J6NegAngLim or J6newAng > J6PosAngLim):
    #almStatusLab.config(text="AXIS LIMIT", bg = "red")
    print(J1newAng,J1NegAngLim,J1PosAngLim)
    print(J2newAng,J2NegAngLim,J2PosAngLim)
    print(J3newAng,J3NegAngLim,J3PosAngLim)
    print(J4newAng,J4NegAngLim,J4PosAngLim)
    print(J5newAng,J5NegAngLim,J5PosAngLim)
    print(J6newAng,J6NegAngLim,J6PosAngLim)
    print("axis limit") #mejora colocar de cual o cuales axis es el limite
    #almStatusLab2.config(text="AXIS LIMIT", bg = "red")
  else:  
    ##J1 calc##
    if (float(J1newAng) >= float(J1AngCur)):   
      #calc pos dir output
      if (J1motdir == "0"):
        J1drivedir = "1"
      else:
        J1drivedir = "0"
      J1dir = J1drivedir
      J1calcAng = float(J1newAng) - float(J1AngCur)
      J1steps = int(J1calcAng / J1DegPerStep)
      J1StepCur = J1StepCur + J1steps #Invert       
      J1AngCur = round(J1NegAngLim + (J1StepCur * J1DegPerStep),2)
      J1steps = str(J1steps) 
    elif (float(J1newAng) < float(J1AngCur)):
      J1dir = J1motdir
      J1calcAng = float(J1AngCur) - float(J1newAng)
      J1steps = int(J1calcAng / J1DegPerStep)
      J1StepCur = J1StepCur - J1steps #Invert       
      J1AngCur = round(J1NegAngLim + (J1StepCur * J1DegPerStep),2)
      J1steps = str(J1steps) 
    ##J2 calc##
    if (float(J2newAng) >= float(J2AngCur)):   
      #calc pos dir output
      if (J2motdir == "0"):
        J2drivedir = "1"
      else:
        J2drivedir = "0"
      J2dir = J2drivedir
      J2calcAng = float(J2newAng) - float(J2AngCur)
      J2steps = int(J2calcAng / J2DegPerStep)
      J2StepCur = J2StepCur + J2steps #Invert       
      J2AngCur = round(J2NegAngLim + (J2StepCur * J2DegPerStep),2)
      J2steps = str(J2steps) 
    elif (float(J2newAng) < float(J2AngCur)):
      J2dir = J2motdir
      J2calcAng = float(J2AngCur) - float(J2newAng)
      J2steps = int(J2calcAng / J2DegPerStep)
      J2StepCur = J2StepCur - J2steps #Invert       
      J2AngCur = round(J2NegAngLim + (J2StepCur * J2DegPerStep),2)
      J2steps = str(J2steps) 
    ##J3 calc##
    if (float(J3newAng) >= float(J3AngCur)):   
      #calc pos dir output
      if (J3motdir == "0"):
        J3drivedir = "1"
      else:
        J3drivedir = "0"
      J3dir = J3drivedir
      J3calcAng = float(J3newAng) - float(J3AngCur)
      J3steps = int(J3calcAng / J3DegPerStep)
      J3StepCur = J3StepCur + J3steps #Invert       
      J3AngCur = round(J3NegAngLim + (J3StepCur * J3DegPerStep),2)
      J3steps = str(J3steps) 
    elif (float(J3newAng) < float(J3AngCur)):
      J3dir = J3motdir
      J3calcAng = float(J3AngCur) - float(J3newAng)
      J3steps = int(J3calcAng / J3DegPerStep)
      J3StepCur = J3StepCur - J3steps #Invert       
      J3AngCur = round(J3NegAngLim + (J3StepCur * J3DegPerStep),2)
      J3steps = str(J3steps) 
    ##J4 calc##
    if (float(J4newAng) >= float(J4AngCur)):   
      #calc pos dir output
      if (J4motdir == "0"):
        J4drivedir = "1"
      else:
        J4drivedir = "0"
      J4dir = J4drivedir
      J4calcAng = float(J4newAng) - float(J4AngCur)
      J4steps = int(J4calcAng / J4DegPerStep)
      J4StepCur = J4StepCur + J4steps #Invert       
      J4AngCur = round(J4NegAngLim + (J4StepCur * J4DegPerStep),2)
      J4steps = str(J4steps) 
    elif (float(J4newAng) < float(J4AngCur)):
      J4dir = J4motdir
      J4calcAng = float(J4AngCur) - float(J4newAng)
      J4steps = int(J4calcAng / J4DegPerStep)
      J4StepCur = J4StepCur - J4steps #Invert       
      J4AngCur = round(J4NegAngLim + (J4StepCur * J4DegPerStep),2)
      J4steps = str(J4steps) 
    ##J5 calc##
    if (float(J5newAng) >= float(J5AngCur)):   
      #calc pos dir output
      if (J5motdir == "0"):
        J5drivedir = "1"
      else:
        J5drivedir = "0"
      J5dir = J5drivedir
      J5calcAng = float(J5newAng) - float(J5AngCur)
      J5steps = int(J5calcAng / J5DegPerStep)
      J5StepCur = J5StepCur + J5steps #Invert       
      J5AngCur = round(J5NegAngLim + (J5StepCur * J5DegPerStep),2)
      J5steps = str(J5steps) 
    elif (float(J5newAng) < float(J5AngCur)):
      J5dir = J5motdir
      J5calcAng = float(J5AngCur) - float(J5newAng)
      J5steps = int(J5calcAng / J5DegPerStep)
      J5StepCur = J5StepCur - J5steps #Invert       
      J5AngCur = round(J5NegAngLim + (J5StepCur * J5DegPerStep),2)
      J5steps = str(J5steps) 
    ##J6 calc##
    if (float(J6newAng) >= float(J6AngCur)):   
      #calc pos dir output
      if (J6motdir == "0"):
        J6drivedir = "1"
      else:
        J6drivedir = "0"
      J6dir = J6drivedir
      J6calcAng = float(J6newAng) - float(J6AngCur)
      J6steps = int(J6calcAng / J6DegPerStep)
      J6StepCur = J6StepCur + J6steps #Invert       
      J6AngCur = round(J6NegAngLim + (J6StepCur * J6DegPerStep),2)
      J6steps = str(J6steps) 
    elif (float(J6newAng) < float(J6AngCur)):
      J6dir = J6motdir
      J6calcAng = float(J6AngCur) - float(J6newAng)
      J6steps = int(J6calcAng / J6DegPerStep)
      J6StepCur = J6StepCur - J6steps #Invert       
      J6AngCur = round(J6NegAngLim + (J6StepCur * J6DegPerStep),2)
      J6steps = str(J6steps)
    ##Track calc##
    # if (TrackNew >= TrackcurPos):
    #   TRdir = "1"
    #   TRdist = TrackNew - TrackcurPos
    #   TRstep = str(int((TrackStepLim/TrackLength)*TRdist))
    # else:
    #   TRdir = "0"
    #   TRdist = TrackcurPos - TrackNew	
    #   TRstep = str(int((TrackStepLim/TrackLength)*TRdist))
    # TrackcurPos = TrackNew
    # TrackcurEntryField.delete(0, 'end')  
    # TrackcurEntryField.insert(0,str(TrackcurPos))	
    commandCalc = "MJA"+J1dir+J1steps+"B"+J2dir+J2steps+"C"+J3dir+J3steps+"D"+J4dir+J4steps+"E"+J5dir+J5steps+"F"+J6dir+J6steps+"T"+"1"+"0"+"S"+newSpeed+"G"+ACCdur+"H"+ACCspd+"I"+DECdur+"K"+DECspd+"\n"
    if Code == 0:
      print(commandCalc)
      #######nuevo modulo
      driveMotorsJ(J1dir,J2dir,J3dir,J4dir,J5dir,J6dir,1,J1steps,J2steps,J3steps,J4steps,J5steps,J6steps,0,newSpeed,ACCdur,ACCspd,DECdur,DECspd)

      
      #######
      #ser.write(commandCalc.encode())
      #print(commandCalc)
      #ser.flushInput()
      time.sleep(.01)
      #ser.read() 
    #J1curAngEntryField.delete(0, 'end')
    #J1curAngEntryField.insert(0,str(J1AngCur))
    #J2curAngEntryField.delete(0, 'end')
    #J2curAngEntryField.insert(0,str(J2AngCur))
    #J3curAngEntryField.delete(0, 'end')
    #J3curAngEntryField.insert(0,str(J3AngCur))
    #J4curAngEntryField.delete(0, 'end')
    #J4curAngEntryField.insert(0,str(J4AngCur))
    #J5curAngEntryField.delete(0, 'end')
    #J5curAngEntryField.insert(0,str(J5AngCur))
    #J6curAngEntryField.delete(0, 'end')
    #J6curAngEntryField.insert(0,str(J6AngCur))
    #CalcFwdKin()
    #DisplaySteps()
    #savePosData() 
    if Code == 2:
      return(commandCalc)	


###################################
calRobotAll()
# Listen for connections, serve client
while True:
    try:       
        cl, addr = s.accept()
        print('client connected from', addr)
        request = cl.recv(1024)
        print("request:")
        print(request)
        request = str(request)
        led_on = request.find('led=on')
        led_off = request.find('led=off')
        
        print( 'led on = ' + str(led_on))
        print( 'led off = ' + str(led_off))
        
        if led_on == 8:
            print("led on")
            led.value(1)
        if led_off == 8:
            print("led off")
            led.value(0)

        #################################################
        cmdType = "Move J"
        command = "Move J [*]  X) 68.76   Y) -0.024   Z) 733.607   W) -89.978   P) 0.95   R) -90.002   T) 40.0   Speed-25 Ad 15 As 10 Dd 20 Ds 5 $N"

        #command = "Move J [*]  X) 205.737   Y) 11.523   Z) 100.038   W) -120.064   P) 172.634   R) -110.886   T) 40.0   Speed-15 Ad 20 As 85 Dd 20 Ds 85 $N"
        
        if (True):
            print("hola")
            J1newIndex = command.find("X) ")
            J2newIndex = command.find("Y) ")
            J3newIndex = command.find("Z) ")
            J4newIndex = command.find("W) ")
            J5newIndex = command.find("P) ")
            J6newIndex = command.find("R) ")
            TRnewIndex = command.find("T) ")	
            SpeedIndex = command.find("Speed-")
            ACCdurIndex = command.find("Ad")
            ACCspdIndex = command.find("As")
            DECdurIndex = command.find("Dd")
            DECspdIndex = command.find("Ds")
            WristConfIndex = command.find("$")
            CX = float(command[J1newIndex+3:J2newIndex-1])
            CY = float(command[J2newIndex+3:J3newIndex-1])
            CZ = float(command[J3newIndex+3:J4newIndex-1])
            CRx = float(command[J4newIndex+3:J5newIndex-1])
            CRy = float(command[J5newIndex+3:J6newIndex-1])
            CRz = float(command[J6newIndex+3:TRnewIndex-1])
            Track = float(command[TRnewIndex+3:SpeedIndex-1])
            newSpeed = str(command[SpeedIndex+6:ACCdurIndex-1])
            ACCdur = command[ACCdurIndex+3:ACCspdIndex-1]
            ACCspd = command[ACCspdIndex+3:DECdurIndex-1]
            DECdur = command[DECdurIndex+3:DECspdIndex-1]
            DECspd = command[DECspdIndex+3:WristConfIndex-1]
            WC = command[WristConfIndex+1:]
            TCX = 0
            TCY = 0 
            TCZ = 0
            TCRx = 0
            TCRy = 0
            TCRz = 0
            Code = 0	
            MoveXYZ(CX,CY,CZ,CRx,CRy,CRz,newSpeed,ACCdur,ACCspd,DECdur,DECspd,WC,TCX,TCY,TCZ,TCRx,TCRy,TCRz,Track,Code)

        
        #######################################
        
        
        
        ledState = "LED is OFF" if led.value() == 0 else "LED is ON" # a compact if-else statement
        
        if button.value() == 1: # button not pressed
            print("button NOT pressed")
            buttonState = "Button is NOT pressed"
        else:
            print("button pressed")
            buttonState = "Button is pressed"
        
        # Create and send response
        stateis = ledState + " and " + buttonState
        response = html % stateis
        cl.send('HTTP/1.0 200 OK\r\nContent-type: text/html\r\n\r\n')
        cl.send(response)
        cl.close()
        
    except OSError as e:
        cl.close()
        print('connection closed')

