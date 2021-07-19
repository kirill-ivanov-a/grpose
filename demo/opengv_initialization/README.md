# OpenGV ReadMe

This ReadMe assumes that you have the necessary data files in order to run OpenGV Initialization.

## RobotCar
After building the code, you can run the following command to see OpenGV try to initialize on two frames in the RobotCar dataset.
```
./build/demo/opengv_initialization/OpenGVInitialization pathToRobotCar pathToRtk 6125 6135 1
```
The first argument calls the executable, the second is the path to the RobotCar dataset. The third argument is the path to the RobotCar rtk if it exists. The fourth and fifth arguments are the first and second indices of the camera frames we want to calculate the relative pose. And the last argument is the number of times we want to run RANSAC. For example, to run on the downloaded data samples set `pathToRobotCar` to `./demo/data-samples/robotcar/2015-10-30-11-56-36` and `pathToRtk` to `./demo/data-samples/robotcar/rtk`

## MultiCam 
Similarly for the synthetic data, you can run the following command
```
./build/demo/opengv_initialization/OpenGVInitialization pathToMCAM 1560 1565 1
```
You just need to provide the path to the MCAM (synthetic data) repository. The third and fourth argument are the frames we want to compare. Note that the frames must correspond to the names of the images in the dataset themselves. And the last argument is the number of times to run RANSAC. For example, to run on the downloaded data samples set `pathToMCAM` to `./demo/data-samples/mcam`


Note, the paths should not have a slash at the end.
