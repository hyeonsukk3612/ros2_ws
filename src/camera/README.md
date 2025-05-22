
설정 및 빌드

///////////////
colcon build --symlink-install --packages-select camera

source ~/ros2_ws/install/local_setup.bash

젝슨보드

ros2 run camera sub

ros2 run camera sub_jetson

윈도우

gst-launch-1.0 -v udpsrc port=9005 ! ‘application/x-rtp,encodingname=(string)H264,payload=(int)96’ ! rtph264depay ! queue ! avdec_h264 ! videoconvert ! autovideosink

카메라 실제 구동 영상입니다

https://youtube.com/shorts/9Q0vdCniHcU
