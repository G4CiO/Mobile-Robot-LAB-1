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
1. Spawn robot in gazebo and publish three model of odometry.

- Default (Ackermann):
    ```bash
    ros2 launch limo_bringup limo_bringup.launch.py
    ```
- Set to Bicycle Mode:
    ```bash
    ros2 launch limo_bringup limo_bringup.launch.py steering_mode:=bicycle
    ```
2. Control robot by teleop_twist_keyboard in another terminal.
    ```
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
# LAB 1.2 Path Tracking Controller
- Run controller server
    ```bash
    ros2 run limo_controller controller_server.py
    ```
- Clear path of robot
    ```bash
    ros2 service call /clear_path std_srvs/srv/Empty
    ```