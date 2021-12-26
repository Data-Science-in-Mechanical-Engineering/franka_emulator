# Welcome to `franka_emulator 1.0.0`!

Here you will find a `C++` library that replaces and behaves like [libfranka](https://frankaemika.github.io/docs/libfranka.html) with one exception - the robot is virtual! It has similar `C++` and `CMake` API and is created to give developers a possibility to write programs for Panda robot without having it.

### Contents
1. [Welcome to Franka Emulator](#welcome-to-franka-emulator)
2. [Contents](#contents)
3. [Usage](#usage)
4. [Requirements](#requirements)
5. [Building](#building)
6. [Installation](#installation)
7. [Documentation](#documentation)
8. [Contributors](#contributors)

### Usage
Usage `franka_emulator` is supposed to be very similar to usage of [libfranka](https://frankaemika.github.io/docs/libfranka.html). Simplest `C++` program reads:
```
#include <franka_emulator/robot.h>
int main()
{
	franka_emulator::Robot robot("192.168.0.1");
}
```
While typical `CMakeLists.txt` usage looks like:
```
find_package(FrankaEmulator 1.0.0 REQUIRED)
add_executable(example example.cpp)
target_link_libraries(example PRIVATE Franka::FrankaEmulator)
```
And that's it, further changes are needed. And is that is not enough, you can add `-DFRANKA_EMULATOR_IMITATE=true` to `cmake` options to eliminate even these differences!

The simulation itself can be launched with:
```
./franka_emulator.sh <IP>
```
Where `<IP>` is the same string that needs to be passed to `franka_emulator::Robot` to make it connected to the simulation. The string is arbitrary (but without slashes) as it is not an actual `IP` address. Actually, the emulator does not use sockets at all.

### Requirements
The library requires:
 - [Gazebo](http://gazebosim.org) (Standalone or as part of `ROS`)
 - [Eigen](https://eigen.tuxfamily.org)
 - [pinocchio](https://stack-of-tasks.github.io/pinocchio) (set with `-Dpinocchio_DIR=/absolute_path_to_pinocchio` or as part of `ROS`)
 - [libfranka](https://github.com/frankaemika/libfranka) (optionally) (set with `-DFranka_DIR=/absolute_path_to_libfranka/build` or as part of `ROS`)
 - [Google Test](https://github.com/google/googletest) (optionally)
 - [Doxygen](https://www.doxygen.nl/index.html) (optionally)
 - [CMake](https://cmake.org) >= `3.4.0`
 - C++11 compatible compiler
 - Fully preemptable Linux kernel (optionally)

###  Building
The library can be built with [CMake](https://cmake.org) using the following commands:
```
mkdir build
cd build
cmake .. #-DFRANKA_EMULATOR_IMITATE=true
cmake --build .
```

### Installation
**Optional step 1**: Getting real-time permission
This can be done in various ways. As an example, the following commands will allow everyone to execute any real-time commands:
```
sudo -s
echo * - rtprio 99       >> /etc/security/limits.conf
echo * - priority 99     >> /etc/security/limits.conf
echo * - memlock 102400  >> /etc/security/limits.conf
exit
```
Reboot for changes to take place. You also may read [this](https://linux.die.net/man/5/limits.conf) for more secure options.

**Optional step 2**: Installing `franka_emulator`
Although the emulator can be run without installation, it can be installed with the commands:
```
cd build
sudo cmake --install .
sudo ldconfig
```
Note that if `FRANKA_EMULATOR_IMITATE` is true, the library will conflict with [libfranka](https://github.com/frankaemika/libfranka). In that case, installing both the emulator and the library will lead to undefined behavior.

**Optional step 3**: Installing real-time kernel
It is said that the fully preemptive kernel can do priority boosting with semaphores, which might increase the emulator's performance (because the library uses semaphores a lot). Note that real-time kernel might cause system instability (and may not). You will alwaus be able to roll back to generic kernel.

For `Debian`: `sudo apt-get install linux-image-rt-amd64` (assuming you have a `x86_64` machine, proceed [here](https://packages.debian.org/search?keywords=linux-image-rt&searchon=names&suite=stable&section=all) if not)

For `Arch`: `yay -S linux-rt-lts` (assuming you use `yay`, proceed [here](https://wiki.archlinux.org/title/AUR_helpers) to see other options)

For `Ubuntu`: Try using `sudo ./install_rt.sh` - it should download, compile and install the kernel along with `systemd-boot`. Improvise if the script fails.

### Documentation
As the library has the same API as [libfranka](https://frankaemika.github.io/docs/index.html#), you may use [official wiki](https://frankaemika.github.io/libfranka/). However, you also can generate documentation with [Doxygen](https://www.doxygen.nl) using `doxygen` command. Generated documentation contains information of how the emulator's behavior differs from that of original [libfranka](https://frankaemika.github.io/docs/index.html#).

### Contributors
 - Kyrylo Sovailo
 - Original [libfranka](https://github.com/intelligent-soft-robots/o80) is owned by [Franka Emika GmbH](https://www.franka.de)
 - Models from `model` directory were initially generated with [Gazebo](http://gazebosim.org) from `ROS` package `franka_description`
