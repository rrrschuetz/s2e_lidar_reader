import time, pyb

class USBReceiver:
    def __init__(self, usb):
        self.usb = usb

    def wait_for_connection(self):
        while not self.usb.isconnected():
            time.sleep(0.1)

    def read_line(self):
        line = ''
        while True:
            char = self.usb.recv(1).decode()
            if char == '\n':
                break
            line += char
        return line.strip()

    def receive_script(self, filename):
        self.wait_for_connection()

        params = {}
        with open(filename, 'wb') as file:
            params['db_gain'] = self.read_line()
            params['gamma_corr'] = self.read_line()
            length = int(self.read_line())

            count = 0
            while count < length:
                data_needed = length - count
                data = self.usb.recv(min(64, data_needed))
                if data:
                    file.write(data)
                    count += len(data)
                else:
                    time.sleep(0.1)  # Pause briefly to wait for more data

        return params

# Example of using the USBReceiver
if __name__ == '__main__':
    usb = pyb.USB_VCP()
    receiver = USBReceiver(usb)
    params = receiver.receive_script('/h7_cam_exec.py')
    globals().update(params)
    exec(open(new_script_filename).read(), globals())
