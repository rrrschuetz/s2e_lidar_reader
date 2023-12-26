from sense_hat import SenseHat
import csv
import math, numpy as np

dt = 0.1
velocity = [0.0,0.0]
acceleration = [0.0,0.0]
min_x = 0
min_y = 0
max_x = 0
max_y = 0

# Initialize sense hat
sense = SenseHat()
sense.clear()
sense.show_message("OK", text_colour=[255, 0, 0])

# Open a CSV file to write data
with open('/home/rrrschuetz/test/speed_data.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    # Write CSV header
    writer.writerow(['Speed (m/s)', 'Min X', 'Max X', 'Min Y', 'Max Y'])

    while True:
        accel = sense.get_accelerometer_raw()
        acceleration = [accel['x'],accel['y']]
        min_x = min(min_x,accel['x'])
        max_x = max(max_x,accel['x'])
        min_y = min(min_y,accel['y'])
        max_y = max(max_y,accel['y'])
        velocity = [v + a * dt for v, a in zip(velocity, acceleration)]
        speed = sum(v**2 for v in velocity)**0.5
        print(f'current speed m/s: {speed:.6f}, min_x: {min_x:.6f}, max_x: {max_x:.6f}, min_y: {min_y:.6f}, max_y: {max_y:.6f}')

        # Write data to CSV
        writer.writerow([f'{speed:.6f}', f'{min_x:.6f}', f'{max_x:.6f}', f'{min_y:.6f}', f'{max_y:.6f}'])

