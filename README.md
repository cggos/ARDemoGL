# AR Demo

-----

## Build

* build the ROS based app
  ```sh
  catkin_make -j4
  ```

## Run

* run ROS based vSLAM to get `current_pointcloud` and `camera_pose`

* run the app
  ```sh
  roslaunch ar_demo xxx.launch
  ```