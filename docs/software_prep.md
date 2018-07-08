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
`$ sudo apt install emacs`

config emacs by going to home directory `$ cd ~` then `$ git clone https://github.com/linZHank/.emacs.d.git$`. Fire up emacs `$ emacs` to start installing packages and setting things up.

# Install ROS-Kinetic
`$ cd ~` then `$ git clone https://github.com/linZHank/installROSTX2.git`

`$ cd installROSTX2` then `$ ./installROS.sh`

This will install ros-kinetic-base and neccessary tools including [catkin-command-line-tools](http://catkin-tools.readthedocs.io/en/latest/)
