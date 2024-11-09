# IMU_ROS2_CMP10A
The nine-axis ARHS attitude sensor MEMS magnetometer bought from yahboom performs well in ROS1, but when it is used in ROS2, the data of a number of sensors will tend to zero indefinitely and the data will float in chaos. The warehouse has rewritten the driver code of the IMU CMP10A. So that it can read the data correctly in ROS2 HUMBLE.
<p align="center">
  <img src="readmefile/1.jpg" width="200" />
</p>

## Install dependencies
you must have serial, and change CMakeLists.txt

## 手动设置自己的serial库路径
```bash
set(SERIAL_INCLUDE_DIR /home/nano/ws/src/serial/include)
set(SERIAL_LIBRARY /home/nano/ws/src/serial/build/libserial.a)
```
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build
```
## Use IMU_Driver
```bash
. install/setup.bash
ros2 launch gnss_imu_sim imu_driver_launch.py
```
