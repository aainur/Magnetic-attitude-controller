import csv
import matplotlib.pyplot as plt
from datetime import datetime


def read_data(filename):
    times = []
    angular_velocities = []
    angular_accelerations = []
    Bxs = []
    Bys = []
    azimuth_Ms =[]
    Mxs =[]
    Mys =[]
    azimuth_Bs =[]

    with open(filename, 'r') as file:
        csv_reader = csv.reader(file)
        initial_time = None

        for row in csv_reader:
            if row:
                try:
                    current_time = datetime.strptime(row[0], "%Y-%m-%d %H:%M:%S.%f")
                    angular_velocity = float(row[4])
                    angular_acceleration = float(row[5])
                    Bx = float(row[1])
                    By = float(row[2])
                    azimuth_M = float(row[6])
                    Mx = float(row[7])
                    My = float(row[8])
                    azimuth_B = float(row[9])

                    if initial_time is None:
                        initial_time = current_time
                    elapsed_time = (current_time - initial_time).total_seconds()

                    times.append(elapsed_time)
                    angular_velocities.append(angular_velocity)
                    angular_accelerations.append(angular_acceleration)
                    Bxs.append(Bx)
                    Bys.append(By)
                    azimuth_Ms.append(azimuth_M)
                    Mxs.append(Mx)
                    Mys.append(My)
                    azimuth_Bs.append(azimuth_B)

                except ValueError as e:
                    print(f"Error parsing row: {row} with error: {e}")

    return times, angular_velocities, angular_accelerations, Bxs, Bys, azimuth_Ms, Mxs, Mys, azimuth_Bs


def plot_angular_velocity(times, angular_velocities):
    plt.figure(figsize=(10, 6))
    plt.plot(times, angular_velocities, marker='o', linestyle='-', color='b')
    plt.title('Change of Angular Velocity Over Time')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.grid(True)
    plt.show()


filename = '/Users/ainur/PycharmProjects/plotMagnetometer/measurement_bdot_log/2024-05-10/18-51-28.csv'

times, angular_velocities, angular_accelerations, Bxs, Bys, azimuth_Ms, Mxs, Mys, azimuth_Bs = read_data(filename)

plot_angular_velocity(times, angular_velocities)

