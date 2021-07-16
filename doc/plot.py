import csv
import matplotlib.pyplot as plt
import numpy as np

real_time = []
names = []
# load the data
with open("doc/data.csv", "r") as cf:
    plots = csv.DictReader(cf, delimiter=",")
    for row in plots:
        real_time.append(float(row['cpu_time']))
        names.append(row['name'])

# create subplots
fig, axs = plt.subplots(2, 2)

# convert the time form nanoseconds to milliseconds.
real_time = np.array(real_time)
real_time /= 1e6

axs[0, 0].plot(real_time[:10], '--o', label='sob_layer')
axs[0, 0].plot(real_time[20:30], '--x', label='infaltion_layer')
axs[0, 0].set_title('100x100 square')

axs[0, 1].plot(real_time[40:49], '--o', label='sob_layer')
axs[0, 1].plot(real_time[58:67], '--x', label='infaltion_layer')
axs[0, 1].set_title('100x100 sparse')

axs[1, 0].plot(real_time[10:20], '--o', label='sob_layer')
axs[1, 0].plot(real_time[30:40], '--x', label='infaltion_layer')
axs[1, 0].set_title('1000x1000 square')

axs[1, 1].plot(real_time[49:58], '--o',label='sob_layer')
axs[1, 1].plot(real_time[67:76], '--x',label='infaltion_layer')
axs[1, 1].set_title('1000x1000 sparse')

for ax in axs.flat:
    ax.set(ylabel='cpu_time [ms]', xlabel='occupancy')
    ax.set_ylim(ymin=0)
    ax.set_xticklabels([])
    ax.legend()

fig.tight_layout()
fig.set_size_inches(10, 10)
fig.savefig('doc/stats.png', dpi=100)

# now get the relative
rel1 = np.array(real_time[20:30]) / np.array(real_time[0:10])
rel2 = np.array(real_time[30:40]) / np.array(real_time[10:20])
rel3 = np.array(real_time[58:67]) / np.array(real_time[40:49])
rel4 = np.array(real_time[67:76]) / np.array(real_time[49:58])

fig, axs = plt.subplots(1)
plt.plot(rel1, label='relative 100x100 square')
plt.plot(rel2, label='relative 1000x1000 square')
plt.plot(rel3, label='relative 100x100 sparse')
plt.plot(rel4, label='relative 1000x1000 sparse')

plt.legend()
fig.savefig('doc/relative.png', dpi=100)
