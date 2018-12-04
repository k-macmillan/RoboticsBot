# RoboticsBot

Final project for CSC 515, Introduction to Robotics

---

## Helpful Links

* [Detecting white](https://stackoverflow.com/questions/22588146/tracking-white-color-using-python-opencv)
* [Detecting black](https://stackoverflow.com/questions/17883023/python-detect-black-squares)
* [Detecting green](https://stackoverflow.com/questions/31590499/opencv-android-green-color-detection)

## Ubuntu 18.04 Notes

Here’s a mostly complete list of things I had to do to get the Geekbot stuff set up on Ubuntu 18.04. I still don’t know for sure everything else will work, but I can at least get the camera feed now.

* Install ROS melodic (Kinetic is targeted specifically at 16.04)

  ```bash
    sudo apt install ros-melodic-desktop ros-melodic-rqt ros-melodic-rqt-common-plugins ros-melodic-rqt-graph
  ```

* You can’t share your wireless network connection over an ethernet cable from the network settings in 18.04. You can get to the same settings window as on 16.04 by running the command `nm-connection-editor`

* I had to modify `geekbot_connect.source` as follows (the output of `ifconfig` changed format, requiring different `grep` and `awk` commands, as well as sourcing ROS Melodic.)

  ```bash
  #!/usr/bin/env bash
  printf "Remember: source this file ('source geekbot_connect.source')\n\n"
  source /opt/ros/melodic/setup.bash

  ETH_GATEWAY=$(ifconfig | grep "inet 10" | awk '/inet/{print $2}')
  GEEKBOT_IP=$(nmap -n -sn $ETH_GATEWAY/24 -oG - | awk '/Up/ {if($2 !="10.42.0.1") {print $2};}')

  echo "This computer's IP: $ETH_GATEWAY"
  echo "Geekbot found at: $GEEKBOT_IP"
  echo "exporting ROS_IP=$ETH_GATEWAY"
  echo "exporting ROS_MASTER_URI=http://$GEEKBOT_IP:11311"

  export ROS_IP=$ETH_GATEWAY
  export ROS_MASTER_URI=http://$GEEKBOT_IP:11311

  printf "\nAvailable topics: \n"
  printf "===================\n"
  rostopic list
  ```

  The modified version of this file exists in *this* repository as [`geekbot.18.04.source`](geekbot.18.04.source).

* There’s a [nasty bug](https://github.com/ros-perception/image_pipeline/issues/201) related to OpenCV and GTK on Ubuntu 18.04 with ROS Melodic, so running

  ```bash
  rosrun image_view image_view image:=/geekbot/webcam/image_raw _image_transport:=compressed
  ```

  crashes. The solution is to use `rqt_image_viewer` with the command

  ```bash
  rosrun rqt_image_view rqt_image_view image:=/geekbot/webcam/image_raw _image_transport:=compressed
  ```

  You’ll need `PyQt5` and `opencv-python` installed for Python2 for this to work. The ROS dependencies are installed with ROS Melodic above.

Moral of the story: don’t try to be special, you’ll end up needing (additional) therapy.
