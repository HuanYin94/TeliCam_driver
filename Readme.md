### Publish ros sensor_msgs::Image

#### Compile

* Download code and install the basic linux drvier at [Toshiba](http://www.toshiba-teli.co.jp/en/products/industrial/usb/index.htm)

* OpenCV 2 is needed for transformation from Bayer Format to ROS message.

* Then

	mkdir build
    cd build
    cmake ..
    make

#### Run

Multiple cameras are allowed. And change different message or node names in ros.

    ./cam_driver [camera_index(0/1/...)] [message_name] [image_frame_name] [ros_node_name]

Launch and rviz are not used, as references.

![image](https://github.com/ZJUYH/TeliCam_driver/blob/master/camera.jpg)
