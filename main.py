#import pyb
import os, time
from machine import Pin

#usb = pyb.USB_VCP()
#red_led = pyb.LED(1)
#green_led = pyb.LED(2)
#blue_led = pyb.LED(3)
#red_led.off()
#green_led.on()
#blue_led.off()

red_led = Pin('P0', Pin.OUT)
green_led = Pin('P1', Pin.OUT)
blue_led = Pin('P2', Pin.OUT)
red_led.value(0)  # 1 to turn on, 0 to turn off
green_led.value(0)
blue_led.value(1)

# Name of the new script file
new_script_filename = "/h7_cam_exec.py"
exec(open(new_script_filename).read(), globals())
