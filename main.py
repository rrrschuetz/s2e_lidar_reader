import pyb, time
import os

usb = pyb.USB_VCP()
red_led = pyb.LED(1)
green_led = pyb.LED(2)
blue_led = pyb.LED(3)

red_led.off()
green_led.on()
blue_led.off()

def receive_script(filename):
    while not usb.isconnected():
        pass

    red_led.off()
    green_led.off()
    blue_led.on()

    while not usb.any():
        pass  # Wait for data

    params = {}
    with open(filename, 'wb') as file:
        db_gain_set =  False
        gamma_corr_set = False
        db_gain_line = ""
        gamma_line = ""
        while usb.any():
            if not db_gain_set:
                char = usb.recv(1).decode()
                if char == '\n':
                    params["db_gain"] = db_gain_line.strip()
                    db_gain_set = True
                else:
                    db_gain_line += char
            elif not gamma_corr_set:
                char = usb.recv(1).decode()
                if char == '\n':
                    params["gamma_corr"] = float(gamma_line)
                    gamma_corr_set = True
                else:
                    gamma_line += char
            else:
                data = usb.recv(4096)  # Receive 64 bytes at a time
                file.write(data)
    return params

# Name of the new script file
new_script_filename = "/h7_cam_exec.py"

# Receive and save the new script
parameters = receive_script(new_script_filename)
time.sleep(10)

# Execute the new script
globals().update(parameters)
exec(open(new_script_filename).read(), globals())
