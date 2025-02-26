# Mobile-Robot-LAB-1

# Install Package
1. Create workspace and src folder and go in src folder
    ```bash
    cd
    mkdir -p MOBILE_ROBOT_WS/src && cd MOBILE_ROBOT_WS/src
    ```
2. git clone all package.
    ```bash
    git clone https://github.com/G4CiO/Mobile-Robot-LAB-1.git
    ```
3. build package
    ```bash
    cd ..
    colcon build
    ```
4. add source this package in .bashrc file.
    ```bash
    echo "source ~/MOBILE_ROBOT_WS/install/setup.bash" >> ~/.bashrc
    ```
5. source this package.
    ```bash
    source ~/.bashrc
    ```

# LAB 1.1 Kinematics of Mobile Robot
## Implementation
1. Spawn robot in gazebo and publish three model of odometry.

- Set to **Ackermann** Mode (Default):
    ```bash
    ros2 launch limo_bringup limo_bringup.launch.py steering_mode:=ackermann
    ```
- Set to **Bicycle** Mode:
    ```bash
    ros2 launch limo_bringup limo_bringup.launch.py steering_mode:=bicycle
    ```
2. Control robot by teleop_twist_keyboard in another terminal.
    ```
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
## Varidation
### Result
### 1. No-slip
![all2_ackermann](./image/all2_ackermann.png)
### 2. Basic model
![all2_bicycle](./image/all2_bicycle.png)

### Comparison of results from graphs
**1.1 XY Trajectory**
- **No-Slip**: have path that similar to Ground Truth and less slip than the *Basic model* path.
- **Basic model**: More error-prone or slip, especially in path deviations, resulting in greater slip away from Ground Truth.
- **Yaw Rate Model**: is the most accurate because it takes into account the Ackermann steering principle and obtains the Yaw Rate value from the gyroscope sensor (IMU) for the rotation.
- **Single Track Model**: gives better results than Double Track Model but still has errors, especially in turning because the rotation is determined by the geometric orientation of the kinematic side-slip angles.
- **Double Track Model**: has the greatest error compared to Ground Truth because the rotation is calculated from the velocity-difference between the two rear wheels. Since the velocity of the two rear wheels is always the same, the change in rotation should always be 0. However, from the graph, there may be some spikes which may be from wheel slip causing the velocity of the two rear wheels to be different.

**1.2 Yaw**
- **No-Slip**: The yaw values ​​tend to be closer to Ground Truth, especially for *Yaw Rate Model*.
- **Basic model**: The yaw values has a greater slip from Ground Truth, as can be seen from the increased Yaw value compared to No-Slip. It because it have more slip when rotation.
- **Yaw Rate Model**: Most accurate and follows Ground Truth well.
- **Single Track Model**: Better performance but still has errors compared to Ground Truth.
- **Double Track Model**: has the most error, especially during fast rotations.

**1.3 Angular Velocity Z**
- **No-Slip**: It is closer to Ground Truth, especially the *Yaw Rate Model* and have some noise in *Single Track Model*
- **Basic model**: Same as *No-Slip* but in *Single Track Model* it has noise more than *No-Slip*.
- **Yaw Rate Model**: Highest accuracy, it can track along Angular Velocity Z of Ground Truth because it obtains the Yaw Rate value from the gyroscope sensor (IMU)
- **Single Track Model**:It's pretty close to Ground Truth, but it still has some errors.
- **Double Track Model**:Has the highest error and highest swing.

### Advantages and Disadvantages

#### 1. No-Slip
- **Advantages**
    - High accuracy, especially in trajectory tracking and yaw estimation.
    - Suitable for vehicles using Ackermann steering, such as cars.

- **Disadvantages**
    - More computationally intensive than the Basic Model.
    - Cannot model slip conditions.

#### 2. Basic Model
- **Advantages**
    - Faster and simpler to compute

- **Disadvantages**
    - Still has errors from slip, especially during sharp turns.
    - Cannot fully model Ackermann steering dynamics.

#### 3. Yaw Rate Model
- **Advantages**
    - Highest accuracy, especially in trajectory tracking and yaw from sensor IMU.
- **Disadvantages**
    - Requaied sensor IMU.
    - Still have error from wheel odometry.

#### 4. Single Track Model
- **Advantages**
    - Suitable for systems with Basic Model (Bicycle Model)
- **Disadvantages**
    - Still has errors, especially in sharp turns.

#### 5. Double Track  Model
- **Advantages**
    - Suitable for systems with Differential Velocity Model that rotation from differential velocity of two rear wheel.
- **Disadvantages**
    - Highest error, especially in XY trajectory for model Ackermann steering.

# LAB 1.2 Path Tracking Controller
First *Spawn robot* by command from LAB 1.1 then
1. Run controller server
- Set to **Pure Pursuit** Mode (Default)
    ```bash
    ros2 run limo_controller controller_server.py --ros-args -p control_mode:=pure_pursuit
    ```
- Set to **PID** Mode
    ```
    ros2 run limo_controller controller_server.py --ros-args -p control_mode:=pid
    ```
- Set to **Stanley** Mode
    ```
    ros2 run limo_controller controller_server.py --ros-args -p control_mode:=stanley
    ```
- Clear path of robot
    ```bash
    ros2 service call /clear_path std_srvs/srv/Empty
    ```
# LAB 1.3 Extended kalman filter && Tuning Q and R 
## Meaning of Q and R
1. Q (Process Noise Covariance)
- Represents uncertainty or inaccuracies in the system model.
- Too-small Q value makes the filter overly trust the model, resulting in a lagged response when unexpected events occur.
- Too-large Q makes the filter rely more heavily on sensor measurements, potentially causing noisy estimates.
2. R (Measurement Noise Covariance):
- Represents sensor or measurement uncertainties.
- Too-small R value causes the filter to overly trust measurements, becoming vulnerable to noise.
- Too-large R makes the filter rely excessively on the model, slowing its response to real measurements.


## Sampling Data for Covarian calculation

run sampling node 
```bash
ros2 run limo_localization plot_odom.py
```
for stop sampling data 
```bash
ros2 topic pub --once /stop_collection std_msgs/Empty "{}"
```
## Images

![Robot Image](image/multi_odom_plot.png)

stop dianosist
```bash
ros2 topic pub /stop_collection std_msgs/msg/Empty "{}" --once
```
