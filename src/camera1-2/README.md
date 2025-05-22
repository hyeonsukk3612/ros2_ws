

colcon build --symlink-install --packages-select camera1-2

source ~/ros2_ws/install/local_setup.bash

ros2 run camera1-2 sub

ros2 run camera1-2 sub_jetson

gst-launch-1.0 -v udpsrc port=9005 ! ‘application/x-rtp,encodingname=(string)H264,payload=(int)96’ ! rtph264depay ! queue ! avdec_h264 ! videoconvert ! autovideosink
