import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import socket

# Define the IP address and port of the socket
ADCS_IP_ADDRESS = "192.168.1.118"
PORT = 23
# Define the number of data points to keep track of
NUM_POINTS = 200
DECLINATION = -5.55

# Create an array to store the data points
data_points = np.zeros((NUM_POINTS, 3))

# Initialize the plot
fig, (ax, ax_2d) = plt.subplots(1, 2, figsize=(10, 5))
ax = fig.add_subplot(1, 2, 1, projection='3d')
ax_2d = fig.add_subplot(1, 2, 2)

text_x = fig.text(0.38, 0.95, '', ha='center', va='top', fontsize=12, fontweight='bold', color='black')
text_y = fig.text(0.50, 0.95, '', ha='center', va='top', fontsize=12, fontweight='bold', color='black')
text_z = fig.text(0.62, 0.95, '', ha='center', va='top', fontsize=12, fontweight='bold', color='black')
text_bearing = fig.text(0.78, 0.95, '', ha='center', va='top', fontsize=12, fontweight='bold', color='black')

def data_gen():
    """ Generator function for fetching data from a socket. """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.connect((ADCS_IP_ADDRESS, PORT))
        while True:
            data = sock.recv(1024).decode().strip()
            if data:
                try:
                    x, y, z = map(float, data.split(';'))
                    data_points[:-1] = data_points[1:]
                    data_points[-1] = [x, y, z]
                    yield data_points
                except ValueError as e:
                    print(f"Data conversion error: {e}")
            else:
                yield data_points

def init():
    """ Initialize the 3D and 2D plots. """
    ax.clear()
    ax.set_xlim(-50, 50)
    ax.set_ylim(-50, 50)
    ax.set_zlim(-50, 50)
    ax_2d.clear()
    ax_2d.set_aspect('equal')

def run(data):
    """ Update function for the animation. """
    ax.clear()
    ax.scatter(data[:, 0], data[:, 1], data[:, 2])
    ax.plot(data[:, 0], data[:, 1], data[:, 2], color='blue')

    ax.set_title(f"3D Trajectory")

    ax_2d.clear()
    ax_2d.scatter(data[:, 0], data[:, 1], color='red', label='XY Plane')
    ax_2d.scatter(data[:, 1], data[:, 2], color='green', label='YZ Plane')
    ax_2d.scatter(data[:, 0], data[:, 2], color='blue', label='XZ Plane')
    ax_2d.legend()
    ax_2d.grid(True)
    ax_2d.set_title("2D Projections")


    text_x.set_text(f"X = {data[-1, 0]:.2f} μT")
    text_y.set_text(f"Y = {data[-1, 1]:.2f} μT")
    text_z.set_text(f"Z = {data[-1, 2]:.2f} μT")

    bearing = np.round((np.arctan2(data[-1, 1], data[-1, 0]) * 180 / np.pi - DECLINATION + 180) % 360, 1)
    text_bearing.set_text(f"Bearing = {bearing}°")

# Animation setup
ani = animation.FuncAnimation(fig, run, data_gen, init_func=init, interval=50, blit=False)

plt.show()

