## ROS-2 Programming Assignment 4

---
![Screenshot from 2024-11-24 17-39-18](https://github.com/user-attachments/assets/1a43f6b6-78ee-4b5d-9c16-77b24ae79cc2)
---
[Watch the video here](https://drive.google.com/file/d/1c8wnbFrWuQQHwjEAGf4uR8vH-duCqmn8/view?usp=sharing)
---

### Dependencies and Requirements

- Ubuntu 22.04 (If running locally)
- Git
- C++17
- ROS2 Humble


### Build instructions

```bash
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/kashifansaricodes/my_beginner_tutorials.git
cd ..
colcon_build
source . install/setup.bash
```

### Running the launch file

```bash
cd ros2_ws/
source . install/setup.bash
ros2 launch beginner_tutorials tutorial_launch.py
```
### To change the string use

```bash
ros2 service call /change_string example_interfaces/srv/SetBool "data: true"
```

### To change the frequency use

```bash
ros2 param set /talker publish_frequency 5.0"
```

### Coomands for TF frames
To check the actual transform values between frames:
```bash
ros2 run tf2_ros tf2_echo world talk
```
To visualize the TF tree (pdf)
```bash
ros2 run tf2_tools view_frames
```

### To run the catch2 test (with additional log details)

```bash
colcon test --packages-select beginner_tutorials --event-handlers console_direct+
```

### To record and play the ros bag

To record:
```bash
ros2 launch beginner_tutorials bag_record_launch.py record_bag:=true
```

To play:
```bash
ros2 bag play src/beginner_tutorials/results/rosbag2_2024_11_15_22_14_00
```

### To run the listener node

```bash
ros2 run beginner_tutorials listener
```



