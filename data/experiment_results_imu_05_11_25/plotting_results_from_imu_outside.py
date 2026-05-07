import re
import matplotlib.pyplot as plt
import math
# === Change this to your log file name ===
log_file = "experiment_outside_05.11.25"

# Regex patterns
pattern_yaw = re.compile(r"Yaw:\s*([-+]?\d*\.\d+|\d+)")
pattern_yaw_prev = re.compile(r"Yaw previous:\s*([-+]?\d*\.\d+|\d+|None)")
pattern_ang_vel = re.compile(r"Ground angular velocity z:\s*([-+]?\d*\.\d+|\d+)")
pattern_slip = re.compile(r"slip:\s*([-+]?\d*\.\d+|\d+)")

# Lists to store values
yaw_list = []
yaw_prev_list = []
ang_vel_list = []
slip_list = []

with open(log_file, "r") as f:
    for line in f:
        if "Yaw:" in line and "Ground angular velocity z" in line:
            yaw_match = pattern_yaw.search(line)
            yaw_prev_match = pattern_yaw_prev.search(line)
            ang_vel_match = pattern_ang_vel.search(line)
            if yaw_match and yaw_prev_match and ang_vel_match:
                yaw = float(yaw_match.group(1))
                yaw_prev_str = yaw_prev_match.group(1)
                yaw_prev = float(yaw_prev_str) if yaw_prev_str != "None" else None
                ang_vel = float(ang_vel_match.group(1))
                yaw_list.append(yaw)
                yaw_prev_list.append(yaw_prev)
                ang_vel_list.append(ang_vel)

        if "slip:" in line:
            slip_match = pattern_slip.search(line)
            if slip_match:
                slip = float(slip_match.group(1))
                slip_list.append(slip)

# === Print extracted data summary ===
print(f"Extracted {len(yaw_list)} yaw values")
print(f"Extracted {len(slip_list)} slip values")

# === Plotting ===
plt.figure(figsize=(6, 4))
plt.plot(slip_list, label='Slip')
plt.xlabel('Time Step')
plt.ylabel('Slip (m)')
plt.title('Slip Over Time')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()



#plotting yaw values
plt.figure(figsize=(6, 4))
plt.plot(yaw_list, label='Yaw')
plt.xlabel('Time Step')
plt.ylabel('Yaw (radians)')
plt.title('Yaw Over Time')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

#plotting the difference between yaw and yaw previous
yaw_diff = []
for i in range(len(yaw_list)):
    if yaw_prev_list[i] is not None:
        unwrapped_diff = yaw_list[i] - yaw_prev_list[i]
        unwrapped_diff = (unwrapped_diff + math.pi) % (2 * math.pi) - math.pi
        yaw_diff.append(unwrapped_diff)
    else:
        yaw_diff.append(0.0)  # or some other default value
plt.figure(figsize=(6, 4))
plt.plot(yaw_diff, label='Yaw Difference')
plt.xlabel('Time Step')
plt.ylabel('Yaw Difference (radians)')
plt.title('Yaw Difference Over Time')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()