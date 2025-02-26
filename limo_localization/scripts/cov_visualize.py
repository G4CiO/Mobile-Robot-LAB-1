import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

def plot_confidence_ellipse(cov, mean, ax, n_std=2.0, **kwargs):
    eigvals, eigvecs = np.linalg.eigh(cov)
    angle = np.degrees(np.arctan2(*eigvecs[:, 1][::-1]))
    width, height = 2 * n_std * np.sqrt(eigvals)
    ellipse = Ellipse(xy=mean, width=width, height=height, angle=angle, **kwargs)
    ax.add_patch(ellipse)
    return ellipse

# Load CSV file
csv_path = 'src/Mobile-Robot-LAB-1/limo_localization/data/multi_odom_data.csv'
df = pd.read_csv(csv_path)

# Extract ground truth positions (common for all plots)
gt_x = df['ground_truth_px'].values
gt_y = df['ground_truth_py'].values

# Define odometry topologies and colors for plotting
odom_types = {
    'yaw_rate': {'color': 'red'},
    'single_track': {'color': 'blue'},
    'double_track': {'color': 'magenta'}
}

# Create subplots in a row
fig, axes = plt.subplots(1, 3, figsize=(18, 6))
fig.suptitle("Comparison of Odometry Topologies vs Ground Truth", fontsize=16)

for ax, (topo, props) in zip(axes, odom_types.items()):
    # Extract odometry positions for the current topology
    odom_x = df[f'{topo}_px'].values
    odom_y = df[f'{topo}_py'].values

    # Plot odometry points and ground truth trajectory
    ax.plot(odom_x, odom_y, '.', color=props['color'], label=f'{topo} Odometry')
    ax.plot(gt_x, gt_y, 'g-', label='Ground Truth')
    
    # Compute covariance and mean for the odometry positions
    cov_pos = np.cov(np.vstack((odom_x, odom_y)))
    mean_pos = [np.mean(odom_x), np.mean(odom_y)]
    plot_confidence_ellipse(cov_pos, mean_pos, ax, edgecolor='blue', lw=2, facecolor='none')
    
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title(f'{topo} Topology')
    ax.legend()
    ax.grid(True)

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()
