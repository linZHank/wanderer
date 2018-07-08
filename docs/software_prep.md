# Jetpack Installation

Find out Jetpack download link and related materials [here](https://developer.nvidia.com/embedded/jetpack). Full-flash TX2 with Jetpack 3.2

# Post Jetpack
## Update and upgrade
```
$ sudo apt update
$ sudo apt upgrade
```
This may take a while...

## Install emacs
`$ sudo apt install emacs`<br/>
config emacs by going to home directory `$ cd ~` then `$ git clone https://github.com/linZHank/.emacs.d.git`. Fire up emacs `$ emacs` to start installing packages and setting things up.

# Install ROS-Kinetic
`$ cd ~` then `$ git clone https://github.com/linZHank/installROSTX2.git`<br/>
`$ cd installROSTX2` then `$ ./installROS.sh`<br/>
This will install ros-kinetic-base and neccessary tools including [catkin-command-line-tools](http://catkin-tools.readthedocs.io/en/latest/)

# Install libfreenect2
Refer to [this](https://github.com/OpenKinect/libfreenect2/blob/master/README.md#linux)<br/>

* Download libfreenect2 source
    ```
    git clone https://github.com/OpenKinect/libfreenect2.git
    cd libfreenect2
    ```
* Install build tools
    ```
    sudo apt-get install build-essential cmake pkg-config
    ```
* Install libusb. The version must be >= 1.0.20.<br/>
    `sudo apt-get install libusb-1.0-0-dev`
* Install TurboJPEG
    `sudo apt-get install libturbojpeg libjpeg-turbo8-dev`
* Install OpenGL
    `sudo apt-get install libglfw3-dev`
* Install OpenNI2 (optional)
    `sudo apt-get install libopenni2-dev`
* Build (if you have run `cd depends` previously, `cd ..` back to the libfreenect2 root directory first.)
    ```
    mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
    make
    make install
    ```
    You need to specify `cmake -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect2` for CMake based third-party application to find libfreenect2.
* Set up udev rules for device access: `sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/`, then replug the Kinect.
* Run the test program: `./bin/Protonect`
* Run OpenNI2 test (optional): `sudo apt-get install openni2-utils && sudo make install-openni2 && NiViewer2`. Environment variable `LIBFREENECT2_PIPELINE` can be set to `cl`, `cuda`, etc to specify the pipeline.
