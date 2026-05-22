# import pandas as pd
# import matplotlib.pyplot as plt
# import os
# import numpy as np

# # ===== INPUT CSV =====
# file = "/tmp/arm_logs_20260401/log_141506_R.csv"

# df = pd.read_csv(file)

# # ===== OUTPUT FOLDER =====
# output_dir = os.path.join(os.path.dirname(file), "plots")
# os.makedirs(output_dir, exist_ok=True)

# # ===== FUNCTION: HUMAN vs ROBOT =====
# def save_plot(x, y1, y2, title, filename):
#     plt.figure()
#     plt.plot(x, y1, label="human")
#     plt.plot(x, y2, label="robot")
#     plt.legend()
#     plt.title(title)
#     plt.xlabel("time")
#     plt.ylabel("value")

#     path = os.path.join(output_dir, filename)
#     plt.savefig(path)
#     plt.close()

# # ===== POSITION =====
# save_plot(df["time"], df["human_elbow_x"], df["robot_elbow_x"], "Elbow X", "elbow_x.png")
# save_plot(df["time"], df["human_elbow_y"], df["robot_elbow_y"], "Elbow Y", "elbow_y.png")
# save_plot(df["time"], df["human_elbow_z"], df["robot_elbow_z"], "Elbow Z", "elbow_z.png")

# save_plot(df["time"], df["human_wrist_x"], df["robot_wrist_x"], "Wrist X", "wrist_x.png")
# save_plot(df["time"], df["human_wrist_y"], df["robot_wrist_y"], "Wrist Y", "wrist_y.png")
# save_plot(df["time"], df["human_wrist_z"], df["robot_wrist_z"], "Wrist Z", "wrist_z.png")

# # ===== ORIENTATION (QUATERNION) =====
# required_cols = [
#     "human_wrist_qx","human_wrist_qy","human_wrist_qz","human_wrist_qw",
#     "robot_wrist_qx","robot_wrist_qy","robot_wrist_qz","robot_wrist_qw"
# ]

# if all(col in df.columns for col in required_cols):

#     save_plot(df["time"], df["human_wrist_qx"], df["robot_wrist_qx"], "Wrist QX", "wrist_qx.png")
#     save_plot(df["time"], df["human_wrist_qy"], df["robot_wrist_qy"], "Wrist QY", "wrist_qy.png")
#     save_plot(df["time"], df["human_wrist_qz"], df["robot_wrist_qz"], "Wrist QZ", "wrist_qz.png")
#     save_plot(df["time"], df["human_wrist_qw"], df["robot_wrist_qw"], "Wrist QW", "wrist_qw.png")

#     # ===== ORIENTATION ERROR (VERY IMPORTANT) =====
#     error = np.sqrt(
#         (df["human_wrist_qx"] - df["robot_wrist_qx"])**2 +
#         (df["human_wrist_qy"] - df["robot_wrist_qy"])**2 +
#         (df["human_wrist_qz"] - df["robot_wrist_qz"])**2 +
#         (df["human_wrist_qw"] - df["robot_wrist_qw"])**2
#     )

#     plt.figure()
#     plt.plot(df["time"], error)
#     plt.title("Quaternion Error (Human vs Robot)")
#     plt.xlabel("time")
#     path = os.path.join(output_dir, "quat_error.png")
#     plt.savefig(path)
#     plt.close()

#     # ===== NORM CHECK =====
#     human_norm = np.sqrt(
#         df["human_wrist_qx"]**2 +
#         df["human_wrist_qy"]**2 +
#         df["human_wrist_qz"]**2 +
#         df["human_wrist_qw"]**2
#     )

#     robot_norm = np.sqrt(
#         df["robot_wrist_qx"]**2 +
#         df["robot_wrist_qy"]**2 +
#         df["robot_wrist_qz"]**2 +
#         df["robot_wrist_qw"]**2
#     )

#     save_plot(df["time"], human_norm, robot_norm, "Quaternion Norm", "quat_norm.png")

# else:
#     print("Quaternion data missing → fix C++ logging")

# print(f"Plots saved in: {output_dir}")