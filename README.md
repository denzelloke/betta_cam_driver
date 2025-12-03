# ROS_IMX_185
This driver is written for [Leopard Imaging Li-IMX185-MIPI-M12](https://www.leopardimaging.com/product/csi-2-mipi-modules-i-pex/csi-2-mipi-modules/rolling-shutter-mipi-cameras/2-36mp-imx185/li-imx185-mipi-m12/) camera for [Nvidia Jetson Xavier NX](https://www.nvidia.com/en-sg/autonomous-machines/embedded-systems/jetson-xavier-nx/) mounted on [Leopard Imaging Nano Carrier Board](https://www.leopardimaging.com/product/nvidia-jetson-cameras/nvidia_nano_mipi_camera_kits/li-nano-cb/li-nano-cb/).
It performs an image read, followed by JPEG compression using nvidia's hardware before publishing the data out as a ros [CompressedImage Message](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CompressedImage.html).

## Requirements
- Supports up to 2 Cameras
- Jetpack 4.4 R32.4.3
- Jetson Multi Media API R32.4.3
- OpenCV > 4.0

## Installation
Please follow [instructions](https://www.dropbox.com/sh/0t2kp7eqz5r2v66/AABanSUlnohk1HHNeIvSFstAa?dl=0) to flash the Jetson NX

## Improved Features

The enhanced ROS IMX477 camera has additional features to support better functionality:

- Initializers, Log directory setup, Loading of calibration file, setting of Publishers and services and Camera settings Initialization for capturing and streaming.

- Captures and process the Raw frame the camera

- Outputs Raw Image JPEG stream at ~4Hz using the Compressed Image message

- Outputs Clahe Image JPEG stream at ~4Hz using the Compressed Image message

- Integrated the main driver along with the Clahe Processing - Two thread process. Main thread runs ros bindings, capture image from camera sensor, and run image compression using Hardware acceleration. The main thread dumps to query image to a mutex-lock variable and waits for the clahe thread to be complete, to then compress and output this image. Second thread purely runs clahe.

- Clahe processing integrated with main driver which corrects the color of the image based on depth and contrast adjustment. The main algorith used is CLAHE histogram eqalization. It first balances the colorspace color based on depth by adding blue before publishing a [compressed image](https://docs.ros.org/en/api/sensor_msgs/html/msg/CompressedImage.html) and then converts to H264. It utilises the "/ikan/nav/world_ned" topic to make use of the depth value for clahe processing.

- Utilizes the following equation to balance blue = surface_blue/(1 + e<sup>(aggro*depth - shift)</sup>)

- Outputs Clahe Image H264 Stream at ~12Hz using theImageH264Feed message

- Outputs Raw Image H264 Stream at ~12Hz using theImageH264Feed message

- Both the Raw and Clahe H264 processing uses the Nvidia hardware acceleration wrapper provided by the FFMPEG which utilises the nvidia multi-media library.

- Outputs an uncompressed image stream 640x640 resolution of the Clahe feed at ~4Hz. It syncs with the Original Clahe compressed image stream.

- The previous uncompressed image stream resolution is configurable through the launch file.

- Option included in launch file to disable the clahe jpeg compression.

- The Clahe processing directly makes use of the YUV image thereby to avoid the colour space conversion.

- The driver is initialised after retrying with few frames to handle the initial frame drops properly.

- Enhanced H264 resource management through the RAII

- Image service callbacks are included to handle the snapshot capturing for both Raw jpeg and Clahe jpeg streams.


## Usage
Sample launch file to initalise the camera
```
<launch>
	<node pkg="h264_camera_driver" name="front_cam_node" type="imx_477_node" respawn="true" output="screen">
		<param name="img_width_"    value="2560"/>
		<param name="img_height_"   value="1440"/>
		<param name="jpeg_quality_" value="75"/>
		<param name="cam_port_"     value="0"/>
		<param name="fps_"          value="15"/>
		<param name="flip_image_"   value="true"/>
		<param name="frame_id_"     value="front_cam"/>
		<param name="topic_name_"   value="/ikan/front_cam"/>
		<param name="clahe_jpeg_compression_"   value="true"/>
		<param name="clahe_uncompressed_width_"   value="640"/>
		<param name="clahe_uncompressed_height_"   value="640"/>
	</node>
</launch>
```

## Dynamic Reconfigure - For Raw Image stream
- Exposure[-20.0 ~ 20.0]
- ISP Gain[-20.0 ~ 20.0]
- Auto White Balance[1~9]
- Red White Balance[Only when Auto White Balance is 9 0.1 ~ 10]
- Green Even White Balance[Only when Auto White Balance is 9 0.1 ~ 10]
- Green Odd White Balance[Only when Auto White Balance is 9 0.1 ~ 10]
- Blue White Balance [Only when Auto White Balance is 9 0.1 ~ 10]
- JPEG Compression Quality [0 ~ 100]

### Dynamic reconfigure - For Clahe Image stream
- Blue  (0 ~ 255) surface level blue
- Green (0 ~ 255) surface level green
- Red   (0 ~ 255) surface level red
- Shift (0.0 ~ 5.0) Linear shift to balance the blue
- aggro (1.0 ~ 3.0) How aggressive to use the depth to balance blue

## Topic Required
```
/ikan/nav/world_ned
```

## Topics output
```
/ikan/front_cam/image_color/compressed
/ikan/front_cam/image_color/clahe/compressed

/ikan/front_cam/image_color/clahe/uncompressed

/ikan/front_cam/h264_color
/ikan/front_cam/h264_color/clahe

```

---

# FFMPEG Hardware Acceleration - One Time Setup

## Minimal Setup for Jetson

* Use the repository [jocover/jetson-ffmpeg](https://github.com/jocover/jetson-ffmpeg) and follow the instructions in the README to build and install the library, create the FFmpeg patch, and build. This setup uses `nvmpi` for FFmpeg hardware acceleration.

* Use the following configuration flags instead of the ones given in the repository:

```bash
./configure --enable-nvmpi \
            --disable-everything \
            --enable-encoder=h264_nvmpi \
            --enable-decoder=h264_nvmpi \
            --enable-decoder=mjpeg \
            --enable-protocol=file \
            --enable-protocol=udp \
            --enable-protocol=rtp \
            --enable-muxer=mp4 \
            --enable-demuxer=h264 \
            --enable-demuxer=mov \
            --enable-demuxer=rtp \
            --enable-demuxer=rtsp \
            --enable-shared \
            --disable-static \
            --prefix=/home/aibx/jetson-ffmpeg/build/ffmpeg
```

* After configuring, run:

```bash
make
sudo make install
```

* Set up environment variables:

```bash
export PATH=/home/aibx/jetson-ffmpeg/build/ffmpeg:$PATH
export LD_LIBRARY_PATH=/home/aibx/jetson-ffmpeg/build/ffmpeg/lib:$LD_LIBRARY_PATH
```

* Add the "Above export lines" to your `.bashrc` for persistence

* Install Curl dev library

```bash
apt-get install libcurl4-openssl-dev
```

---
# betta_cam_driver
