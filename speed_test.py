from sense_hat import SenseHat
import csv
import time
import locale

velocity = [0.0]
acceleration = [0.0]
min_x = 0.0
min_y = 0.0
max_x = 0.0
max_y = 0.0
count = 0

def format_decimal(value):
    """Format a decimal number as a string with a comma as the decimal separator."""
    return f"{value:.6f}".replace('.', ',')

# Initialize sense hat
sense = SenseHat()
sense.clear()
sense.show_message("OK", text_colour=[255, 0, 0])

# Open a CSV file to write data
with open('/home/rrrschuetz/test/speed_data.csv', 'w', newline='') as file:
    writer = csv.writer(file, delimiter=';')  # Use semicolon as the delimiter
    # Write CSV header
    writer.writerow(['Count', 'Timestamp', 'Speed (m/s)', 'Min X', 'Max X', 'Y'])

    last_time = time.time()  # Initialize the last_time variable
    start_time = last_time

    while True:
        current_time = time.time()
        dt = current_time - last_time  # Calculate dt
        last_time = current_time  # Update last_time for the next iteration
        delta_time = current_time-start_time

        count += 1
        accel = sense.get_accelerometer_raw()
        x = accel['x']
        y = accel['y']
        min_x = min(min_x, x)
        max_x = max(max_x, x)
        min_y = min(min_y, y)
        max_y = max(max_y, y)
        acceleration = [y]
        velocity = [v + a * dt for v, a in zip(velocity, acceleration)]
        speed = sum(v**2 for v in velocity)**0.5
        print(f'Count: {count}, Timestamp: {delta_time:.6f}, Speed: {speed:.6f}, Y: {y:.6f}, Min Y: {min_y:.6f}, Max Y: {max_y:.6f}, Y: {y:.6f}')

        # Write data to CSV
        #writer.writerow([count, f'{delta_time:.6f}', f'{speed:.6f}', f'{y:.6f}', f'{min_y:.6f}', f'{max_y:.6f}'])
        writer.writerow([count, format_decimal(delta_time), format_decimal(speed), format_decimal(min_y), format_decimal(max_y), format_decimal(max_y)])

