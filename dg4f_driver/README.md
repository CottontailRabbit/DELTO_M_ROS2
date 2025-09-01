# dg4f\_driver ROS 2 Package 🚀

## 📌 Overview

The `dg4f_driver` ROS 2 package provides a hardware interface leveraging [ros2\_control](https://control.ros.org/) for the DG-4F grippers, enabling direct robotic control operations.

## 📦 Dependency Installation

### Navigate to Workspace

```bash
cd ~/your_ws
```

### Update rosdep

```bash
apt update
rosdep update
```

### Install Specific Dependencies

```bash
rosdep install --from-paths src/DELTO_ROS2/dg4f_driver --ignore-src -r -y
```

### Verify Installation by Building

```bash
colcon build --packages-select dg4f_driver
```

## ⚠️ Before You Control: Notes

The dg4f\_driver (ros2\_control) currently operates in Developer Mode, which uses a custom protocol over Ethernet.
If the gripper is set to Developer Mode, please make sure that switches ② and ④ are in the correct positions, as shown in the attached image.

<img src="./images/manual.png" width="400px"/> 

## 🎛️ Controlling Delto Gripper-4F-LEFT

### 1. Loading Delto-Gripper-4F-LEFT controller

Launch the Delto Gripper-4F controller with:

```bash
ros2 launch dg4f_driver dg4f_driver.launch.py
```


## 🎛️ Controlling Delto Gripper-4F

### 1. Loading Delto-Gripper-4F controller

Launch the Delto Gripper-4F controller with:

```bash
ros2 launch dg4f_driver dg4f_driver.launch.py
```


## 🤝 Contributing

Contributions are encouraged:

1. Fork repository
2. Create branch (`git checkout -b feature/my-feature`)
3. Commit changes (`git commit -am 'Add my feature'`)
4. Push (`git push origin feature/my-feature`)
5. Open pull request

## 📄 License

BSD-3-Clause

## 📧 Contact

[TESOLLO SUPPORT](mailto:support@tesollo.com)
