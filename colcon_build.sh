LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zsj/WorkSpace/YLR1D_ROS2/src/drivers/robot_package/RobotConSys_SDK/lib

colcon build --cmake-args -DTARGET_PLATFORM=linux_x64 --packages-select example robot_package common_utils