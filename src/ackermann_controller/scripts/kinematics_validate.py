#!/usr/bin/env python3
import math
import yaml
import matplotlib.pyplot as plt
#from ament_index_python.packages import get_package_share_directory

def get_closest_index(time_list, t):
    """Return the index of the entry in time_list closest to time t."""
    if not time_list:
        return None
    closest_index = 0
    min_diff = abs(time_list[0] - t)
    for i, time_val in enumerate(time_list):
        diff = abs(time_val - t)
        if diff < min_diff:
            min_diff = diff
            closest_index = i
    return closest_index

def compute_position_errors_sync(gt_time, gt_pos, model_time, model_pos):
    """For each ground truth timestamp, find the closest model data and compute Euclidean position error."""
    errors = []
    for i, t in enumerate(gt_time):
        idx = get_closest_index(model_time, t)
        if idx is not None:
            dx = gt_pos[i][0] - model_pos[idx][0]
            dy = gt_pos[i][1] - model_pos[idx][1]
            errors.append(math.sqrt(dx*dx + dy*dy))
    return errors

def compute_yaw_errors_sync(gt_time, gt_yaw, model_time, model_yaw):
    """For each ground truth timestamp, find the closest model data and compute absolute yaw error (wrapped)."""
    errors = []
    for i, t in enumerate(gt_time):
        idx = get_closest_index(model_time, t)
        if idx is not None:
            diff = gt_yaw[i] - model_yaw[idx]
            diff = math.atan2(math.sin(diff), math.cos(diff))
            errors.append(abs(diff))
    return errors

def trim_model_data(gt_time, model_time, model_pos, model_yaw):
    """Trim the model data (time, pos, yaw) so that their length matches ground truth.
       This removes extra starting values from the model data if necessary."""
    gt_len = len(gt_time)
    if len(model_time) > gt_len:
        return (model_time[-gt_len:], model_pos[-gt_len:], model_yaw[-gt_len:])
    return (model_time, model_pos, model_yaw)

def main():
    # Load the collected YAML data
    yaml_path = "/home/sunny/FRA532_LAB1/src/ackermann_controller/yaml/no-slip-kinematics.yaml"
    try:
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading YAML file from {yaml_path}: {e}")
        return

    # Load ground truth data
    gt_time = data["ground_truth"]["time"]
    gt_pos = data["ground_truth"]["pos"]
    gt_yaw = data["ground_truth"]["yaw"]

    # Trim the first 37 entries of ground truth
    trim_count = 37
    if len(gt_time) > trim_count:
        gt_time = gt_time[trim_count:]
        gt_pos = gt_pos[trim_count:]
        gt_yaw = gt_yaw[trim_count:]
    else:
        print("Warning: Ground truth data has less than 37 entries.")

    # Load model data
    st_time = data["single_track"]["time"]
    st_pos = data["single_track"]["pos"]
    st_yaw = data["single_track"]["yaw"]

    dt_time = data["double_track"]["time"]
    dt_pos = data["double_track"]["pos"]
    dt_yaw = data["double_track"]["yaw"]

    yr_time = data["yaw_rate"]["time"]
    yr_pos = data["yaw_rate"]["pos"]
    yr_yaw = data["yaw_rate"]["yaw"]

    # Debug prints to check lengths before trimming model data
    print(f"Ground Truth entries (after trim): {len(gt_time)}")
    print(f"Single Track entries (before trim): {len(st_time)}")
    print(f"Double Track entries (before trim): {len(dt_time)}")
    print(f"Yaw Rate entries (before trim): {len(yr_time)}")

    # Trim model data so that their lengths match the trimmed ground truth:
    st_time, st_pos, st_yaw = trim_model_data(gt_time, st_time, st_pos, st_yaw)
    dt_time, dt_pos, dt_yaw = trim_model_data(gt_time, dt_time, dt_pos, dt_yaw)
    yr_time, yr_pos, yr_yaw = trim_model_data(gt_time, yr_time, yr_pos, yr_yaw)

    print(f"Single Track entries (after trim): {len(st_time)}")
    print(f"Double Track entries (after trim): {len(dt_time)}")
    print(f"Yaw Rate entries (after trim): {len(yr_time)}")

    # --- Plot 1: Position Trajectories ---
    fig1, axs1 = plt.subplots(1, 3, figsize=(15, 5))
    gt_x = [p[0] for p in gt_pos]
    gt_y = [p[1] for p in gt_pos]
    st_x = [p[0] for p in st_pos]
    st_y = [p[1] for p in st_pos]
    dt_x = [p[0] for p in dt_pos]
    dt_y = [p[1] for p in dt_pos]
    yr_x = [p[0] for p in yr_pos]
    yr_y = [p[1] for p in yr_pos]

    axs1[0].plot(gt_x, gt_y, 'bo-', label='Ground Truth', markersize=1)
    axs1[0].plot(st_x, st_y, 'ro-', label='Single Track', markersize=1)
    axs1[0].set_title("Position: GT vs Single Track")
    axs1[0].set_xlabel("X")
    axs1[0].set_ylabel("Y")
    axs1[0].legend()

    axs1[1].plot(gt_x, gt_y, 'bo-', label='Ground Truth', markersize=1)
    axs1[1].plot(dt_x, dt_y, 'ro-', label='Double Track', markersize=1)
    axs1[1].set_title("Position: GT vs Double Track")
    axs1[1].set_xlabel("X")
    axs1[1].set_ylabel("Y")
    axs1[1].legend()

    axs1[2].plot(gt_x, gt_y, 'bo-', label='Ground Truth', markersize=1)
    axs1[2].plot(yr_x, yr_y, 'ro-', label='Yaw Rate', markersize=1)
    axs1[2].set_title("Position: GT vs Yaw Rate")
    axs1[2].set_xlabel("X")
    axs1[2].set_ylabel("Y")
    axs1[2].legend()

    # --- Plot 2: Yaw vs Time ---
    fig2, axs2 = plt.subplots(1, 3, figsize=(15, 5))
    axs2[0].plot(gt_time, gt_yaw, 'bo-', label='Ground Truth', markersize=1)
    axs2[0].plot(st_time, st_yaw, 'ro-', label='Single Track', markersize=1)
    axs2[0].set_title("Yaw: GT vs Single Track")
    axs2[0].set_xlabel("Time (s)")
    axs2[0].set_ylabel("Yaw (rad)")
    axs2[0].legend()

    axs2[1].plot(gt_time, gt_yaw, 'bo-', label='Ground Truth', markersize=1)
    axs2[1].plot(dt_time, dt_yaw, 'ro-', label='Double Track', markersize=1)
    axs2[1].set_title("Yaw: GT vs Double Track")
    axs2[1].set_xlabel("Time (s)")
    axs2[1].set_ylabel("Yaw (rad)")
    axs2[1].legend()

    axs2[2].plot(gt_time, gt_yaw, 'bo-', label='Ground Truth', markersize=1)
    axs2[2].plot(yr_time, yr_yaw, 'ro-', label='Yaw Rate', markersize=1)
    axs2[2].set_title("Yaw: GT vs Yaw Rate")
    axs2[2].set_xlabel("Time (s)")
    axs2[2].set_ylabel("Yaw (rad)")
    axs2[2].legend()

    # --- Plot 3: Position Error Distributions ---
    st_pos_err = compute_position_errors_sync(gt_time, gt_pos, st_time, st_pos)
    dt_pos_err = compute_position_errors_sync(gt_time, gt_pos, dt_time, dt_pos)
    yr_pos_err = compute_position_errors_sync(gt_time, gt_pos, yr_time, yr_pos)

    fig3, axs3 = plt.subplots(1, 3, figsize=(15, 5))
    axs3[0].hist(st_pos_err, bins=50, color='r', alpha=0.7, label='Single Track')
    axs3[0].set_title("Pos Error: GT vs Single Track")
    axs3[0].set_xlabel("Error (m)")
    axs3[0].set_ylabel("Frequency")
    axs3[0].legend()

    axs3[1].hist(dt_pos_err, bins=50, color='r', alpha=0.7, label='Double Track')
    axs3[1].set_title("Pos Error: GT vs Double Track")
    axs3[1].set_xlabel("Error (m)")
    axs3[1].set_ylabel("Frequency")
    axs3[1].legend()

    axs3[2].hist(yr_pos_err, bins=50, color='r', alpha=0.7, label='Yaw Rate')
    axs3[2].set_title("Pos Error: GT vs Yaw Rate")
    axs3[2].set_xlabel("Error (m)")
    axs3[2].set_ylabel("Frequency")
    axs3[2].legend()

    # --- Plot 4: Yaw Error Distributions ---
    st_yaw_err = compute_yaw_errors_sync(gt_time, gt_yaw, st_time, st_yaw)
    dt_yaw_err = compute_yaw_errors_sync(gt_time, gt_yaw, dt_time, dt_yaw)
    yr_yaw_err = compute_yaw_errors_sync(gt_time, gt_yaw, yr_time, yr_yaw)

    fig4, axs4 = plt.subplots(1, 3, figsize=(15, 5))
    axs4[0].hist(st_yaw_err, bins=50, color='r', alpha=0.7, label='Single Track')
    axs4[0].set_title("Yaw Error: GT vs Single Track")
    axs4[0].set_xlabel("Error (rad)")
    axs4[0].set_ylabel("Frequency")
    axs4[0].legend()

    axs4[1].hist(dt_yaw_err, bins=50, color='r', alpha=0.7, label='Double Track')
    axs4[1].set_title("Yaw Error: GT vs Double Track")
    axs4[1].set_xlabel("Error (rad)")
    axs4[1].set_ylabel("Frequency")
    axs4[1].legend()

    axs4[2].hist(yr_yaw_err, bins=50, color='r', alpha=0.7, label='Yaw Rate')
    axs4[2].set_title("Yaw Error: GT vs Yaw Rate")
    axs4[2].set_xlabel("Error (rad)")
    axs4[2].set_ylabel("Frequency")
    axs4[2].legend()

    # Display the plots. If you're in a non-GUI environment, consider saving the figures instead.
    plt.show()
    # To save figures instead, uncomment these lines:
    # fig1.savefig("position_trajectories.png")
    # fig2.savefig("yaw_vs_time.png")
    # fig3.savefig("position_error_distribution.png")
    # fig4.savefig("yaw_error_distribution.png")

if __name__ == '__main__':
    main()
