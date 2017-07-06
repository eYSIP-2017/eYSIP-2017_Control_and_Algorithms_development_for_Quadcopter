from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import csv

plot_list = 1
filename = 'csv/test2.csv'

with open(filename, 'rb') as f:
    reader = csv.reader(f)
    data = list(reader)

x_raw = []
y_raw = []
z_raw = []

for i, sample in enumerate(data):
    x_raw.append(float(sample[0]))
    y_raw.append(float(sample[1]))
    z_raw.append(float(sample[2]))

min_ = [0, 0, 0]
max_ = [0, 0, 0]
bias = [0, 0, 0]
range_ = [0, 0, 0]
scale = [0, 0, 0]

max_[0] = max(x_raw)
max_[1] = max(y_raw)
max_[2] = max(z_raw)

min_[0] = min(x_raw)
min_[1] = min(y_raw)
min_[2] = min(z_raw)

for i in range(0,3):
    bias[i] = (max_[i] + min_[i])/2
    range_[i] = (max_[i] - min_[i])/2

avg = (range_[0] + range_[1] + range_[2])/3
for i in range(0,3):
    scale[i] = avg/range_[i]

print 'Hard Iron Calibration: X: %7.3f\tY: %7.3f\tZ: %7.3f'%(bias[0], bias[1], bias[2])
print 'Soft Iron Calibration: X: %7.3f\tY: %7.3f\tZ: %7.3f'%(scale[0], scale[1], scale[2])

x_data = []
y_data = []
z_data = []

for i, sample in enumerate(data):
    if i%1==0:
        x_data.append((float(sample[0]) - bias[0]) * scale[0])
        y_data.append((float(sample[1]) - bias[1]) * scale[1])
        z_data.append((float(sample[2]) - bias[2]) * scale[2])

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_aspect('equal')

if plot_list:
    X = np.array(x_data)
    Y = np.array(y_data)
    Z = np.array(z_data)
else:
    X = np.array(x_raw)
    Y = np.array(y_raw)
    Z = np.array(z_raw)

scat = ax.scatter(X, Y, Z, c='r')
max_range = np.array([X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min()]).max() / 2.0

mid_x = (X.max()+X.min()) * 0.5
mid_y = (Y.max()+Y.min()) * 0.5
mid_z = (Z.max()+Z.min()) * 0.5
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

plt.show()

