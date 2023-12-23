import pyb
import os

usb = pyb.USB_VCP()
red_led = pyb.LED(1)
green_led = pyb.LED(2)
blue_led = pyb.LED(3)

red_led.on()
green_led.off()
blue_led.off()

def receive_script(filename):
    if usb.isconnected():
        print("Waiting for data...")
        while not usb.any():
            pass  # Wait for data

        with open(filename, 'wb') as file:
            while usb.any():
                data = usb.recv(64)  # Receive 64 bytes at a time
                file.write(data)

        print("Script received and saved as", filename)

# Name of the new script file
new_script_filename = "/h7_cam_exec.py"

# Receive and save the new script
receive_script(new_script_filename)

red_led.off()
green_led.off()
blue_led.on()

# Execute the new script
exec(open(new_script_filename).read(), globals())
