import matplotlib.pyplot as plt
import numpy as np

# Global variable to keep track of current line index
current_line_index = 0
# Global variable to keep track of current chunk index
current_chunk_index = 0


def plot_radar_chart(data):
    global current_line_index

    def update_chart(event):
        global current_line_index
        if current_line_index < len(data) - 1:
            current_line_index += 1
            ax.clear()
            plot_data(data[current_line_index])
            fig.canvas.draw()

    def plot_data(line_data):
        # Extend the data to wrap around the plot
        extended_angles = np.concatenate((angles, [angles[0]]))
        extended_line_data = np.concatenate((line_data, [line_data[0]]))

        # Plot points using scatter
        ax.scatter(extended_angles, extended_line_data)

    # Each value represents a fraction of the 180-degree range
    angles = np.linspace(-np.pi/2, np.pi/2, len(data[0]), endpoint=False)

    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    fig.canvas.mpl_connect('button_press_event', update_chart)

    plot_data(data[current_line_index])

    ax.set_theta_zero_location("N")  # Zero pointing East
    ax.set_theta_direction(1)  # Counterclockwise

    plt.show()


def read_data(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()[1:]  # Skip the first line

    scan_data = []
    color_data = []

    for line in lines:
        scan_values = line.strip().split(',')[2:1622]
        scan_data.append(np.array(scan_values, dtype=float))
        color_values = line.strip().split(',')[-1280:]
        color_data.append(np.array(color_values, dtype=int))

    return scan_data,color_data


def plot_color_data(color_data, chunk_size=1000):
    global current_chunk_index

    def update_chart(event):
        global current_chunk_index
        if current_chunk_index < len(color_data) // chunk_size:
            current_chunk_index += 1
            ax.clear()
            plot_chunk(color_data[current_chunk_index * chunk_size : (current_chunk_index + 1) * chunk_size])
            fig.canvas.draw()

    def plot_chunk(chunk):
        # Create an empty white canvas
        canvas = np.ones((chunk_size, len(chunk[0]), 3))
        # Set pixels based on color values (0: white, 1: red, 2: green)
        for i, line in enumerate(chunk):
            for j, value in enumerate(line):
                if value == 1:
                    canvas[i, j] = [1, 0, 0]  # Red
                elif value == 2:
                    #canvas[i, j] = [0, 1, 0]  # Green
                    canvas[i, j] = [0, 0, 1]  # Blue

        ax.imshow(canvas)

    fig, ax = plt.subplots()
    fig.canvas.mpl_connect('button_press_event', update_chart)

    plot_chunk(color_data[current_chunk_index * chunk_size : (current_chunk_index + 1) * chunk_size])

    plt.show()

# File reading and plotting
filename = '/home/rrrschuetz/test/file_converted.csv'
scan_data, color_data = read_data(filename)
plot_radar_chart(scan_data)
plot_color_data(color_data)

