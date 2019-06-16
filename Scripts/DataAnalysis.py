import pandas as pd
import matplotlib.pyplot as plt
from os import listdir
from os.path import isfile, join

telemetry_dir = "../Telemetry"
telemetry_files = [
    join(telemetry_dir, file) for file in listdir(telemetry_dir)
    if isfile(join(telemetry_dir, file))
]
telemetry_files = sorted(telemetry_files)

# Default to most recent file.
selected_file = telemetry_files[-1]
file_path = selected_file
df = pd.read_csv(file_path)

df[["y_pos", "y_pos_target"]].plot()
plt.show()

df[["z_pos", "z_pos_target"]].plot()
plt.show()

df[["y_vel", "y_vel_desired"]].plot()
plt.show()

df[["z_vel", "z_vel_desired"]].plot()
plt.show()

df[["x_vel", "vs_desired"]].plot()
plt.show()

df[["altitude", "target_alt"]].plot()
plt.show()

df[["roll_desired", "roll"]].plot()
plt.show()
df[["pitch_desired", "pitch"]].plot()
plt.show()
df[["yaw_desired", "yaw"]].plot()
plt.show()

