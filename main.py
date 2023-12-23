import pyb
import os

usb = pyb.USB_VCP()

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
new_script_filename = "h7_cam_exec.py"

# Receive and save the new script
receive_script(new_script_filename)

# Execute the new script
exec(open(new_script_filename).read(), globals())
