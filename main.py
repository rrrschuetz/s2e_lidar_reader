import sensor, image, time, math, pyb, os
import machine

def log_message(message):
    with open('/logfile.log', 'a') as logfile:
        timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
        logfile.write(f"{timestamp} - {message}\n")

class USBReceiver:
    def __init__(self, usb):
        self.usb = usb

    def wait_for_connection(self):
        while not self.usb.isconnected():
            log_message("Waiting for USB connection...")
            time.sleep(0.1)

    def read_line(self):
        line = ''
        while True:
            if self.usb.any():
                try:
                    char = self.usb.recv(1).decode()
                    if char == '\n':
                        break
                    line += char
                except UnicodeDecodeError:
                    log_message("Decode error: replacing unknown character.")
                    line += '?'
        return line.strip()

    def receive_params(self):
        self.wait_for_connection()
        log_message("Connection established.")
        params = {}
        try:
            params['db_gain'] = self.read_line()
            params['gamma_corr'] = self.read_line()
         except Exception as e:
             log_message(f"Error during file reception: {e}")
        return params

if __name__ == '__main__':
    usb = pyb.USB_VCP()
    receiver = USBReceiver(usb)
    params = receiver.receive_params()
    db_gain = float(params['db_gain'])
    gamma_corr = float(params['gamma_corr'])

    unique_id = machine.unique_id()
    unique_id_hex = ''.join(['{:02x}'.format(byte) for byte in unique_id])

    red_led = pyb.LED(1)
    green_led = pyb.LED(2)
    blue_led = pyb.LED(3)

    red_led.on()
    green_led.off()
    blue_led.off()

    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)

    sensor.set_auto_gain(False, gain_db = db_gain) # must be turned off for color tracking
    #sensor.set_auto_whitebal(False, (1.5,1.5,1.0)) # must be turned off for color tracking
    sensor.set_auto_whitebal(False)
    sensor.set_saturation(3)
    sensor.skip_frames(time = 2000)

    green =  (30, 100, -64, -8, -32, 32)  # generic green
    #green = (38,60,-9,-128,-128, 127)

    red =  (30, 100, 15, 127, 15, 127)    # generic red
    #red = (0,49,127,6,-2,20)

    magenta = (0, 100, 32, 127, 127, -94)

    thresholds=[green, red, magenta]
    roi = [0,0,320,140]

    while True:
        try:
            img = sensor.snapshot()
            img.lens_corr(strength=2.6, zoom=1.0)
            img.gamma_corr(gamma = gamma_corr)

            blob_entries = []
            blobs = img.find_blobs(thresholds,0,roi,pixels_threshold=160, merge=False)
            for blob in blobs:
                (b_x,b_y,b_w,b_h) = blob.rect()
                b_c = blob.code()
                if (b_c == 4 and b_h/b_w < 1) or (b_c in [1,2] and b_h/b_w > 1):
                    #img.draw_rectangle(blob.rect(),color=(0,0,255),thickness=3)
                    #img.draw_cross(blob.cx(), blob.cy())
                    blob_entries.append("{},{},{}".format(b_c, b_x, b_x+b_w))

            bloblist = ','.join(blob_entries)
            if bloblist:
                data = "{},".format(unique_id_hex) + bloblist +"\n"
                usb.write(data)
                usb.flush()

        except Exception as e:
                log_message(f"Failed to capture frame: {e}")
