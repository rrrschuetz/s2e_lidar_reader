import time, pyb

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

    def receive_script(self, filename):
        self.wait_for_connection()
        log_message("Connection established.")

        params = {}
        with open(filename, 'wb') as file:
            try:
                params['db_gain'] = self.read_line()
                params['gamma_corr'] = self.read_line()
                length = int(self.read_line())
                log_message(f"Parameters received: {params}")
                log_message(f"Receiving script of length: {length}")

                count = 0
                while count < length:
                    if self.usb.any():
                        data_needed = length - count
                        log_message(f"Data needed: {data_needed}")
                        data = self.usb.recv(min(64, data_needed))
                        if data:
                            file.write(data)
                            count += len(data)
                        else:
                            log_message("No data received, waiting...")
                            time.sleep(0.1)  # Pause briefly to wait for more data
            except Exception as e:
                log_message(f"Error during file reception: {e}")

        log_message(f"Received {count} bytes.")
        return params

# Example of using the USBReceiver
if __name__ == '__main__':
    usb = pyb.USB_VCP()
    receiver = USBReceiver(usb)
    new_script_filename = '/h7_cam_exec.py'
    params = receiver.receive_script(new_script_filename)
    globals().update(params)
    exec(open(new_script_filename).read(), globals())
