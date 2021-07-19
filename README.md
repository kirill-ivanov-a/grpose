# Generalized Relative Pose Estimation

This repository contains utilities for testing generalized relative pose estimation in the context of multi-camera SLAM initialization. The goal of the project is to bring real-world and synthetic datasets together with relative pose solvers to provide a benchmarking framework for the latter.

## Setup (A recent Ubuntu)
### Prerequisites
  ```bash
sudo apt install git g++ cmake libgflags-dev libgoogle-glog-dev libfmt-dev
  ```

### OpenCV
The OpenGV solver initialization pipeline requires a version of OpenCV from 4.4. Please install it from [here](https://opencv.org/releases/) and provide the installation directory to CMake by setting the additional flag `-DOPENCV_INSTALL_DIR=<your OpenCV install dir>`

### Building All Demos and the Libarary
  ```bash
  mkdir build
  cd build
  cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
  cmake --build . -- -j6
  ```
  To build a specific demo you can run
  ```bash
cmake --build . --target demo -- -j6
  ```
  where `demo` stands for the required binary. Both commands will also build OpenGV, which may take substantial time and may even freeze the machine if built in Debug mode.


## License
This project is licensed under the terms of the MIT license. For more information, please check the [`LICENSE.txt`](LICENSE.txt) file.
