import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import socket
import datetime
import logging
from collections import deque
import csv
import os
# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Configuration
config = {
    # "ADCS_IP_ADDRESS": "192.168.1.5",
    "ADCS_IP_ADDRESS": "10.0.0.10",
    "PORT": 23,
    "NUM_POINTS": 200,
    "DECLINATION": -5.55,
    "DIR": "/Users/ainur/PycharmProjects/plotMagnetometer/measurement_bdot_log",
    "CSVFILE": ""
}
os.makedirs(config["DIR"], exist_ok=True)

data_points = np.zeros((config["NUM_POINTS"], 9))

# Deques for rolling logs
time_log = deque(maxlen=config["NUM_POINTS"])
angular_velocity_log = deque(maxlen=config["NUM_POINTS"])
angular_acceleration_log = deque(maxlen=config["NUM_POINTS"])

# Initialize plot
fig, (ax, ax_2d) = plt.subplots(1, 2, figsize=(10, 5))
ax = fig.add_subplot(1, 2, 1, projection='3d')
ax_2d = fig.add_subplot(1, 2, 2)

# Text elements for data display
text_elements = {
    "text_x": fig.text(0.28, 0.95, '', ha='center', va='top', fontsize=12, fontweight='bold', color='black'),
    "text_y": fig.text(0.40, 0.95, '', ha='center', va='top', fontsize=12, fontweight='bold', color='black'),
    "text_z": fig.text(0.52, 0.95, '', ha='center', va='top', fontsize=12, fontweight='bold', color='black'),
    "text_bearing": fig.text(0.68, 0.95, '', ha='center', va='top', fontsize=12, fontweight='bold', color='black'),
    "text_angular_velocity": fig.text(0.90, 0.99, '', ha='center', va='top', fontsize=8, fontweight='bold', color='black'),
    "text_angular_acceleration": fig.text(0.90, 0.96, '', ha='center', va='top', fontsize=8, fontweight='bold', color='black'),
    "text_azimuth_M": fig.text(0.90, 0.93, '', ha='center', va='top', fontsize=8, fontweight='bold', color='black'),
    "text_Mx_My": fig.text(0.90, 0.90, '', ha='center', va='top', fontsize=8, fontweight='bold', color='black')
}


def get_csv_filename():
    if not config["CSVFILE"]:
        date_str = datetime.datetime.now().strftime("%Y-%m-%d")
        time_str = datetime.datetime.now().strftime("%H-%M-%S")
        directory_path = os.path.join(config["DIR"], date_str)
        if not os.path.exists(directory_path):
            os.makedirs(directory_path)

        filename = f"{time_str}.csv"
        config["CSVFILE"] = os.path.join(directory_path, filename)

    return config["CSVFILE"]

get_csv_filename()
def connect_socket(ip_address, port):
    """ Connect to the socket and return the socket object. """
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ip_address, port))
    return sock

def data_gen():
    """ A generator that yields data points received from the socket. """
    with connect_socket(config["ADCS_IP_ADDRESS"], config["PORT"]) as sock:
        while True:
            data = sock.recv(1024).decode().strip()
            print("Received Data:", data)
            if data:
                try:
                    yield process_data(data)
                except ValueError as e:
                    logging.error(f"Data conversion error: {e}")
            else:
                yield data_points

def process_data(data):
    """ Process a single line of received data. """
    global data_points  # Ensure we are modifying the global data_points

    elements = data.split(';')
    x, y, z = map(float, elements[:3])

    # Roll the data_points array first
    data_points = np.roll(data_points, -1, axis=0)

    # Now update the newly freed last row (after roll, it's a fresh space for new data)
    print("Before Update Last Row:", data_points[-1])  # Should show zeros if this is the new space

    data_points[-1, :3] = [x, y, z]

    if len(elements) > 4:
        angular_velocity = float(elements[3])
        data_points[-1, 3] = angular_velocity
        angular_acceleration = float(elements[4])
        data_points[-1, 4] = angular_acceleration
        current_time = datetime.datetime.now()

        time_log.append(current_time)
        angular_velocity_log.append(angular_velocity)
        angular_acceleration_log.append(angular_acceleration)

        azimuth_M = float(elements[5])
        Mx, My = map(float, elements[6:8])
        azimuth_B = float(elements[8])

        data_points[-1, 5:9] = [azimuth_M, Mx, My, azimuth_B]

        write_to_csv(
            [current_time, x, y, z, angular_velocity, angular_acceleration, azimuth_M, Mx, My, azimuth_B])

    print("After Update Last Row:", data_points[-1])  # This should now show the updated data

    return data_points

def write_to_csv(data):
    with open(config["CSVFILE"], 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(data)

def init():
    """ Initialize the 3D and 2D plots. """
    ax.set_xlim(-50, 50)
    ax.set_ylim(-50, 50)
    ax.set_zlim(-50, 50)
    ax_2d.set_aspect('equal')
    ax_2d.grid(True)

def run(data):
    """ Update function for the animation. """
    ax.clear()
    ax.scatter(data[:, 0], data[:, 1], data[:, 2])
    ax.plot(data[:, 0], data[:, 1], data[:, 2], color='blue')
    ax.set_title("3D Trajectory")

    ax_2d.clear()
    ax_2d.scatter(data[:, 0], data[:, 1], color='red', label='XY Plane')
    ax_2d.scatter(data[:, 1], data[:, 2], color='green', label='YZ Plane')
    ax_2d.scatter(data[:, 0], data[:, 2], color='blue', label='XZ Plane')
    ax_2d.legend()
    ax_2d.grid(True)
    ax_2d.set_title("2D Projections")

    update_texts(data[-1])

def update_texts(last_data_point):
    """ Update text elements with the latest data. """
    print("Debug last_data_point in update_texts:", last_data_point)
    text_elements["text_x"].set_text(f"Bx: {last_data_point[0]:.2f} Gauss")
    text_elements["text_y"].set_text(f"By: {last_data_point[1]:.2f} Gauss")
    text_elements["text_z"].set_text(f"Bz: {last_data_point[2]:.2f} Gauss")

    angle_diff = np.abs(last_data_point[5] - last_data_point[8]) % 360
    if angle_diff > 180:
        angle_diff = 360 - angle_diff
    torque = (last_data_point[6]*last_data_point[1]) - (last_data_point[7]-last_data_point[0])

    fig.suptitle(f"Azimuth B: {last_data_point[8]:.1f}°, Angular velocity: {last_data_point[3]:.2f}rad/s, " \
                 f"Angular acceleration: {last_data_point[4]:.2f}rad/s², Azimuth M: {last_data_point[5]:.1f}°, " \
                 f"Mx: {last_data_point[6]:.2f}, My: {last_data_point[7]:.2f}, Torque: {torque:.2f}")

# Animation setup
ani = animation.FuncAnimation(fig, run, data_gen, init_func=init, interval=80, blit=False)

plt.show()
