import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv("file.csv", header=None, delim_whitespace=True)
caldata = pd.read_csv("calfile.csv", header=None, delim_whitespace=True)

data.columns = ['X', 'Y', 'Z']
caldata.columns = ['Xcal', 'Ycal', 'Zcal']
constant_z = data['Z'].iloc[0]

# use scatter instead of plot
plt.scatter(0, 0, c='red', marker='o')
plt.scatter(data['X'], data['Y'], color='blue')
plt.scatter(caldata['Xcal'], caldata['Ycal'], color='green')
plt.xlabel('X axis, [Gauss]', fontsize=14)
plt.ylabel('Y axis, [Gauss]', fontsize=14)
plt.title(f'Data Visualization (Z axis = {constant_z})', fontsize=16)
 #function to set the ratio matplot lib aspect ratio 1:1

x_range = data['X'].max() - data['X'].min()
y_range = data['Y'].max() - data['Y'].min()
aspect_ratio = y_range / x_range


plt.gca().set_aspect(aspect_ratio, adjustable='box')
plt.tick_params(axis='both', which='major', labelsize=12)
plt.show()




