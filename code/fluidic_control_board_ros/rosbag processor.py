import rosbag
import matplotlib.pyplot as plt
import os

# Find ROS Bag Files in the Current Folder
bag_files = [file for file in os.listdir('.') if file.endswith('.bag')]

if not bag_files:
    print("No ROS bag files found in the current folder.")
    exit()

# Display Available ROS Bag Files
print("Available ROS bag files:")
for i, bag_file in enumerate(bag_files):
    print(f"{i + 1}. {bag_file}")

# Ask User to Select a ROS Bag File
selection = input("Enter the number of the ROS bag file you want to load: ")
try:
    selection = int(selection)
    if 1 <= selection <= len(bag_files):
        selected_bag = bag_files[selection - 1]
    else:
        print("Invalid selection.")
        exit()
except ValueError:
    print("Invalid input.")
    exit()

# Load Selected ROS Bag Data
bag = rosbag.Bag(selected_bag)
topic = '/capacitance'  # Replace with the topic you want to plot
values1 = []
values2 = []
values3 = []
avg = []
timestamps = []

for topic, msg, t in bag.read_messages(topics=[topic]):
    timestamps.append(msg.Header.stamp.secs + msg.Header.stamp.nsecs*(1e-9))
    values1.append(msg.channel0)
    values2.append(msg.channel2)  
    values3.append(msg.channel3)
    # avg.append(msg.average)
bag.close()

values1 = [value - values1[0] for value in values1]
values2 = [value - values2[0] for value in values2]
values3 = [value - values3[0] for value in values3]

# Create Plots
plt.scatter(timestamps,values1, marker='o', s=10, label='Sensor 1')
plt.scatter(timestamps,values2, marker='o', s=10, label='Sensor 2')
plt.scatter(timestamps,values3, marker='o', s=10, label='Sensor 3')
plt.xlabel('Time (s)')
plt.ylabel('Value')
plt.title(f'Plot of ROS Bag Data from {selected_bag}')
plt.legend()
plt.grid(True)
plt.show()


# plt.hist(avg, bins=20)
# plt.legend()
# plt.grid(True)
# plt.show()
