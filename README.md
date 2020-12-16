#!/usr/bin/env python3

from PCF8574 import PCF8574_GPIO
from Adafruit_LCD1602 import Adafruit_CharLCD
import RPi.GPIO as GPIO
import Keypad
import time
from time import sleep

GPIO.setmode(GPIO.BOARD)        # use PHYSICAL GPIO Numbering

#----------------------------------------------------------------------------------------------#

def pulseIn(pin,level,timeOut): # obtain pulse time of a pin under timeOut
    t0 = time.time()
    while(GPIO.input(pin) != level):
        if((time.time() - t0) > timeOut*0.000001):
            return 0;
    t0 = time.time()
    while(GPIO.input(pin) == level):
        if((time.time() - t0) > timeOut*0.000001):
            return 0;
    pulseTime = (time.time() - t0)*1000000
    return pulseTime

#----------------------------------------------------------------------------------------------#
    
def getSonar():     # get the measurement results of ultrasonic module,with unit: cm
    GPIO.output(trigPin,GPIO.HIGH)      # make trigPin output 10us HIGH level 
    time.sleep(0.00001)     # 10us
    GPIO.output(trigPin,GPIO.LOW) # make trigPin output LOW level 
    pingTime = pulseIn(echoPin,GPIO.HIGH,timeOut)   # read plus time of echoPin
    distance = pingTime * 340.0 / 2.0 / 10000.0     # calculate distance with sound speed 340m/s 
    return distance

#----------------------------------------------------------------------------------------------#

def password(key, passwd):
	default = "!AB12"
	passwd = passwd + key
	print(passwd)
	if(passwd == default):
		print("success")
		unarm = 0
	else:
		unarm = 1
	return(passwd, unarm)

#----------------------------------------------------------------------------------------------#

def loop():
	passwd = "!"
	mcp.output(3,1)     # turn on LCD backlight
	lcd.begin(16,2)     # set number of LCD lines and columns
	unarm = 1		
		
	while(unarm == 1):
		GPIO.output(ledPin,GPIO.HIGH) # turn on led
		lcd.setCursor(0,0)  # set cursor position
		lcd.message('Alarm Armed!\n')
		distance = getSonar() # get distance			
		if(distance < 50 and distance != 0):
			while(True):
				key = keypad.getKey()
				lcd.setCursor(0,0)  # set cursor position
				lcd.message('Intruder alert!')
				GPIO.output(ledPin,GPIO.LOW) # turn off led
				GPIO.output(buzzerPin,GPIO.HIGH) # turn on buzzer
				if(key != keypad.NULL):     #if there is key pressed, print its key code.
					passwd, unarm = password(key, passwd)
					if(key == '#'):
						passwd = "!"
					if(unarm == 0):
						GPIO.output(ledPin,GPIO.LOW) # turn off led
						GPIO.output(buzzerPin,GPIO.HIGH) # turn on buzzer
						lcd.clear()
						lcd.message('Hello\nFavouriteStudent')
						return 0
		else:
			GPIO.output(ledPin,GPIO.HIGH) # turn on led
			GPIO.output(buzzerPin,GPIO.LOW) # turn off buzzer


#---------------------------------------------------------------------------------------------#

PCF8574_address = 0x27  # I2C address of the PCF8574 chip.
PCF8574A_address = 0x3F  # I2C address of the PCF8574A chip.
# Create PCF8574 GPIO adapter.
try:
    mcp = PCF8574_GPIO(PCF8574_address)
except:
    try:
        mcp = PCF8574_GPIO(PCF8574A_address)
    except:
        print ('I2C Address Error !')
        exit(1)

#---------------------------------------------------------------------------------------------#

ROWS = 4        # number of rows of the Keypad
COLS = 4        #number of columns of the Keypad
keys =  [   '1','2','3','A',    #key code
            '4','5','6','B',
            '7','8','9','C',
            '*','0','#','D'     ]
rowsPins = [12,16,18,22]        #connect to the row pinouts of the keypad
colsPins = [19,15,13,11]        #connect to the column pinouts of the keypad

keypad = Keypad.Keypad(keys,rowsPins,colsPins,ROWS,COLS)    #creat Keypad object
keypad.setDebounceTime(50)      #set the debounce time

#---------------------------------------------------------------------------------------------#

buzzerPin = 37    # define buzzerPin

GPIO.setup(buzzerPin, GPIO.OUT)   # set buzzerPin to OUTPUT mode

#---------------------------------------------------------------------------------------------#

ledPin = 40       # define ledPin

GPIO.setup(ledPin, GPIO.OUT)    # set ledPin to OUTPUT mode

#----------------------------------------------------------------------------------------------#

trigPin = 38
echoPin = 32
MAX_DISTANCE = 220          # define the maximum measuring distance, unit: cm
timeOut = MAX_DISTANCE*60   # calculate timeout according to the maximum measuring distance

GPIO.setup(trigPin, GPIO.OUT)   # set trigPin to OUTPUT mode
GPIO.setup(echoPin, GPIO.IN)    # set echoPin to INPUT mode

#----------------------------------------------------------------------------------------------#

# Create LCD, passing in MCP GPIO adapter.
lcd = Adafruit_CharLCD(pin_rs=0, pin_e=2, pins_db=[4,5,6,7], GPIO=mcp)

if __name__ == '__main__':
    print ('Program is starting ... ')
    try:
        loop()
    except KeyboardInterrupt:
        lcd.clear()
        GPIO.cleanup()
