import pyb, time
import os

usb = pyb.USB_VCP()
red_led = pyb.LED(1)
green_led = pyb.LED(2)
blue_led = pyb.LED(3)

def receive_script(filename):
    while not usb.isconnected(): pass


    if usb.any(): with open(filename, 'wb') as file:
            data = usb.recv(4096)  # Receive 64 bytes at a time
            file.write(data)

# Name of the new script file
new_script_filename = "/h7_cam_exec.py"

while True:

    red_led.off()
    green_led.on()
    blue_led.off()

    # Receive and save the new script
    receive_script(new_script_filename)
    #time.sleep(10)
    # Execute new or old script
    exec(open(new_script_filename).read(), globals())
