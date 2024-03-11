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
green_led.value(1)
blue_led.value(0)

def receive_script(filename):
    #while not usb.isconnected():
    #    pass
    
    pipe_r_path = "/pipe_r"
    if not os.path.exists(pipe_r_path):
        os.mkfifo(pipe_r_path)
    
    #red_led.off()
    #green_led.off()
    #blue_led.on()
    red_led.value(0)  # 1 to turn on, 0 to turn off
    green_led.value(0)
    blue_led.value(1)

    #while not usb.any():
    #    pass  # Wait for data

    with open(filename, 'wb') as file:
        
        #while usb.any():
        #    data = usb.recv(4096)  # Receive 64 bytes at a time
        #    file.write(data)

        with open(pipe_r_path, "r") as fifo:
            file.write(fifo.read())

# Name of the new script file
new_script_filename = "/h7_cam_exec.py"

# Receive and save the new script
receive_script(new_script_filename)
time.sleep(10)

# Execute the new script
exec(open(new_script_filename).read(), globals())
