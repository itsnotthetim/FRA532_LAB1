import yaml
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import numpy as np

def calculate_mean_errors(yaml_path):
    # Load the YAML file
    with open(yaml_path, 'r') as file:
        data = yaml.safe_load(file)

    # Extract EKF positions and yaw data
    ekf_positions = np.array(data['ekf_positions'])
    ekf_yaw = ekf_positions[:, 1]  # Assuming the second column is yaw

    # Extract ground truth positions and yaw data
    ground_truth_positions = np.array(data['ground_truth_positions'])
    ground_truth_yaw = ground_truth_positions[:, 1]

    # Drop the first 180 indices
    ekf_positions = ekf_positions[200:]
    ground_truth_positions = ground_truth_positions[200:]
    ekf_yaw = ekf_yaw[200:]
    ground_truth_yaw = ground_truth_yaw[200:]

    # Ensure both arrays have the same length by trimming to the minimum length
    min_length = min(len(ekf_positions), len(ground_truth_positions))

    ekf_positions = ekf_positions[:min_length, :2]
    ground_truth_positions = ground_truth_positions[:min_length, :2]
    ekf_yaw = ekf_yaw[:min_length]
    ground_truth_yaw = ground_truth_yaw[:min_length]

    # Calculate position errors
    position_errors = np.linalg.norm(ekf_positions - ground_truth_positions, axis=1)
    mean_position_error = np.mean(position_errors)

    # Calculate yaw errors
    yaw_errors = np.abs(ekf_yaw - ground_truth_yaw)
    mean_yaw_error = np.mean(yaw_errors)

    return mean_position_error, mean_yaw_error

def main():
    package_name = "ackermann_controller"  # Your package name
    installed_package_path = Path(get_package_share_directory(package_name))
    workspace_path = installed_package_path.parents[3]  # Move up to workspace root
    package_path = workspace_path / "src" / package_name  # Construct the src path
    yaml_path = os.path.join(package_path, "yaml/ekf_inc22_double_track_validation.yaml")

    mean_position_error, mean_yaw_error = calculate_mean_errors(yaml_path)

    print(f"Mean Position Error: {mean_position_error}")
    print(f"Mean Yaw Error: {mean_yaw_error}")

if __name__ == "__main__":
    main()
