#!/usr/bin/env python3
"""
compute_all_odom_cov.py

Reads a CSV containing ground truth and three odometry estimates (each 12 states) from a fixed location,
computes the 12x12 covariance matrices for:
  - yaw_rate
  - single_track
  - double_track

Then exports all covariance matrices into a single YAML file for use by other nodes (e.g., an EKF node).

The script uses the workspace finder algorithm to locate the workspace directory (e.g. "MOBILE_ROBOT_WS")
and builds the input and output paths relative to it so that it works whether run via the src folder or install/lib folder.
"""

import os
import pandas as pd
import numpy as np
import yaml  # Install via: pip install pyyaml

def find_workspace_by_name(start_path, workspace_name="MOBILE_ROBOT_WS"):
    """
    Traverse upward from start_path until a directory named workspace_name is found.
    
    Args:
        start_path (str): The starting directory path.
        workspace_name (str): The name of the workspace directory to find.
        
    Returns:
        str or None: The absolute path of the workspace directory if found, else None.
    """
    current_dir = os.path.abspath(start_path)
    while True:
        if os.path.basename(current_dir) == workspace_name:
            return current_dir
        parent_dir = os.path.dirname(current_dir)
        if parent_dir == current_dir:  # reached the filesystem root
            return None
        current_dir = parent_dir

# --- Determine workspace and build directories ---
script_dir = os.path.dirname(os.path.abspath(__file__))
workspace_dir = find_workspace_by_name(script_dir, "MOBILE_ROBOT_WS")
if workspace_dir is None:
    print("Could not find workspace directory 'MOBILE_ROBOT_WS'. Using script directory as fallback.")
    workspace_dir = script_dir

# Build the data directory path relative to the workspace.
data_dir = os.path.join(workspace_dir, "src", "Mobile-Robot-LAB-1", "limo_localization", "data")
data_dir = os.path.abspath(data_dir)
os.makedirs(data_dir, exist_ok=True)

# Build the config directory path relative to the workspace.
config_dir = os.path.join(workspace_dir, "src", "Mobile-Robot-LAB-1", "limo_localization", "config")
config_dir = os.path.abspath(config_dir)
os.makedirs(config_dir, exist_ok=True)

# Fixed paths for input CSV and YAML output
INPUT_CSV = os.path.join(data_dir, "multi_odom_data.csv")
YAML_OUTPUT = os.path.join(config_dir, "covariances.yaml")

def compute_covariance(df, odom_prefix, gt_prefix="ground_truth"):
    """
    Computes the 12x12 covariance matrix for the odometry type specified by odom_prefix relative to ground truth.
    
    Returns:
        tuple: (mean_error, covariance matrix) or (None, None) if data is insufficient.
    """
    components = ["px", "py", "pz", "roll", "pitch", "yaw", "vx", "vy", "vz", "wx", "wy", "wz"]
    error_cols = []

    for comp in components:
        odom_col = f"{odom_prefix}_{comp}"
        gt_col = f"{gt_prefix}_{comp}"
        err_col = f"err_{odom_prefix}_{comp}"
        if odom_col not in df.columns or gt_col not in df.columns:
            print(f"Missing column: {odom_col} or {gt_col}")
            return None, None
        df[err_col] = df[odom_col] - df[gt_col]
        error_cols.append(err_col)

    # Remove any rows with missing error data
    df_clean = df.dropna(subset=error_cols)
    error_array = df_clean[error_cols].to_numpy()
    if len(error_array) < 2:
        print(f"Not enough rows for {odom_prefix} to compute covariance!")
        return None, None

    R_odom = np.cov(error_array, rowvar=False)
    mean_err = np.mean(error_array, axis=0)
    return mean_err, R_odom

def main():
    print(f"Looking for CSV file at: {INPUT_CSV}")
    if not os.path.exists(INPUT_CSV):
        print(f"Error: CSV file not found at {INPUT_CSV}")
        return

    df = pd.read_csv(INPUT_CSV)
    odom_types = ["yaw_rate", "single_track", "double_track"]
    cov_dict = {}

    for odom in odom_types:
        print(f"\n==== Processing covariance for '{odom}' ====")
        mean_err, cov = compute_covariance(df, odom)
        if cov is None:
            continue
        cov_dict[odom] = cov.tolist()  # Convert NumPy array to list for YAML

    export_dict = {"covariances": cov_dict}
    with open(YAML_OUTPUT, "w") as f:
        yaml.dump(export_dict, f)

    print(f"\nCovariance matrices exported to YAML file: {YAML_OUTPUT}")
    print(f"YAML file saved in directory: {config_dir}")

if __name__ == '__main__':
    main()
