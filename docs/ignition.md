Note, ROS2 Foxy on Ubuntu 20.04 with Ignition Dome is the only supported setup.

1. Start by setting up your system dependencies:

    ```
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    sudo apt-get update
    
    sudo apt install git python3-pip
    
    pip3 install vcstool
    pip3 install -U colcon-common-extensions
    ```

# Binary installation

The easiest way to setup ignition is to install from binaries:

    ```
    sudo apt install ignition-dome
    ```

Alternatively you can install it from source.

# Source installation

1. Install `ignition-dome` from source. The branches in the official guide can be used

    ```
    mkdir ws_dome/src -p
    cd ws_dome
    ```

1. Use the provided `ign-dome.yaml` file and initialise the sources:

    ```
    wget https://raw.githubusercontent.com/ignition-tooling/gazebodistro/master/collection-dome.yaml
    vcs import src < collection-dome.yaml

    ```

1. Build from source and source in your .bashrc (or remember to source manually before running any demo):

    ```
    sudo apt-get install cmake freeglut3-dev libavcodec-dev libavdevice-dev libavformat-dev libavutil-dev libdart6-collision-ode-dev libdart6-dev libdart6-utils-urdf-dev libfreeimage-dev libgflags-dev libglew-dev libgts-dev libogre-1.9-dev libogre-2.1-dev libprotobuf-dev libprotobuf-dev libprotoc-dev libqt5core5a libswscale-dev libtinyxml2-dev libtinyxml-dev pkg-config protobuf-compiler python qml-module-qt-labs-folderlistmodel qml-module-qt-labs-settings qml-module-qtquick2 qml-module-qtquick-controls qml-module-qtquick-controls2 qml-module-qtquick-dialogs qml-module-qtquick-layouts qml-module-qtqml-models2 qtbase5-dev qtdeclarative5-dev qtquickcontrols2-5-dev ruby ruby-ronn uuid-dev libzip-dev libjsoncpp-dev libcurl4-openssl-dev libyaml-dev libzmq3-dev libsqlite3-dev libwebsockets-dev swig ruby-dev -y
    
    colcon build --merge-install --symlink-install --cmake-args -DBUILD_TESTING=false
    
    echo "source ~/ws_dome/install/setup.bash" >> ~/.bashrc
    ```
    
# Launch

To launch a demo in ignition set the `use_ignition` parameter to 1, i.e. to launch the office demo:

  ```
  ros2 launch demos office.launch.xml use_ignition:=1
  ```
