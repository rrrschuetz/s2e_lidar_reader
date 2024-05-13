import time, pyb
import hashlib

class USBReceiver:
    def __init__(self, usb):
        self.usb = usb

    def wait_for_connection(self, timeout=60):
        start_time = time.time()
        while not self.usb.isconnected():
            time.sleep(0.1)
            if time.time() - start_time > timeout:
                raise Exception("Timeout waiting for USB connection.")

    def read_line(self, timeout=5):
        line = ''
        start_time = time.time()
        while True:
            if self.usb.any():
                char = self.usb.recv(1).decode()
                if char == '\n':
                    break
                line += char
            else:
                if time.time() - start_time > timeout:
                    raise Exception("Timeout reading line from USB.")
                time.sleep(0.1)
        return line.strip()

    def receive_script(self, filename):
        self.wait_for_connection()
        params = {}
        with open(filename, 'wb') as file:
            params['db_gain'] = self.read_line()
            params['gamma_corr'] = self.read_line()
            expected_hash = self.read_line()
            length = int(self.read_line())

            received_data = bytearray()
            while len(received_data) < length:
                data_needed = length - len(received_data)
                data = self.usb.recv(min(64, data_needed))
                if data:
                    received_data.extend(data)
                else:
                    time.sleep(0.1)  # Pause briefly to wait for more data

            received_hash = hashlib.sha256(received_data).hexdigest()
            if received_hash != expected_hash:
                raise ValueError("Data corruption: hash mismatch.")
            file.write(received_data)

        return params

# Example of using the USBReceiver
if __name__ == '__main__':
    usb = pyb.USB_VCP()
    receiver = USBReceiver(usb)
    new_script_filename = '/h7_cam_exec.py'
    try:
        params = receiver.receive_script(new_script_filename)
        globals().update(params)
        exec(open(new_script_filename).read(), globals())
    except Exception as e:
        pass
