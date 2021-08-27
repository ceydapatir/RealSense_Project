# RealSense_Project
Recording high resolution image and IMU data using D435I and T265 cameras on NVIDIA JETSON NANO

# System Requirements
cat /etc/os-release
&nbsp /*
**Enter**NAME="Ubuntu"
**Enter**VERSION="18.04.5 LTS (Bionic Beaver)"
**Enter**ID=ubuntu
**Enter**ID_LIKE=debian
**Enter**PRETTY_NAME="Ubuntu 18.04.5 LTS"
**Enter**VERSION_ID="18.04"
**Enter**HOME_URL="https://www.ubuntu.com/"
**Enter**SUPPORT_URL="https://help.ubuntu.com/"
**Enter**BUG_REPORT_URL="https://bugs.launchpad.net/ubuntu/"
**Enter**PRIVACY_POLICY_URL="https://www.ubuntu.com/legal/terms-and-policies/privacy-policy"
**Enter****Enter**VERSION_CODENAME=bionic
**Enter**UBUNTU_CODENAME=bionic
**Enter***/

# Librealsense Setup
--> https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md
--> https://lieuzhenghong.com/how_to_install_librealsense_on_the_jetson_nx/
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
sudo apt-get install -y --no-install-recommends \
    python3 \
    python3-setuptools \
    python3-pip \
    python3-dev
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense/
sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at
./scripts/setup_udev_rules.sh
mkdir build
cd build
cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true
sudo make uninstall && sudo make clean && sudo make -j4 && sudo make install
export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.6/pyrealsense2

# Realsense Viewer Setup
--> https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils

# Import pyrealsense2
Apply the codes below
> import sys
> print(sys.path)
#Copy .so files of pyrealsense (librealsense/build/wrappers/python) to one of the displayed paths
In some cases;
> import pyrealsense2.pyrealsense2
statement can be used
