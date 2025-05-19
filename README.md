# IHBFMTstart（beta version）
An asymptotically optimal sampling-based method for high-dimensional complex motion planning.
# Dependencies
The Open Motion Planning Library (OMPL) 
* Install dependencies according to OMPLapp 1.3.1 requirements.
* opencv-2.4.13.6
# Installation
Pull this repository to /ompl-1.3.1-IHBFMTstart-2d-beta.
```
mkdir -p build/Release
cd build/Release
cmake ../..
# next step is optional
make -j 4 # replace "4" with the number of cores on your machine
```
# Environments
The library contains a 2D environment. However, you can make your own environments.
![Snipaste_2025-05-19_12-32-11](https://github.com/user-attachments/assets/07b7f216-46b4-4953-8f6a-b3c15e035910)

# Usage
Please note: before using our Testplanning, please modify the map path in **IHBFMT.h**.
```
*you should set your own map path*/
Mat envImage_ = cv::imread("~/ompl-1.3.1-2d-IHBFMTstart-beta/tests/resources/ppm/floor.ppm", 1);
```
After the compilation of ompl-1.3.1-IHBFMTstart-2d-beta is successful, you can run the dynamic search demo of IHBFMT* (``` ompl-1.3.1-2d-IHBFMTstart-beta/demos/TestPlanning.cpp```).
```
ompl-1.3.1-2d-IHBFMTstart-beta/build/Release/bin$ ./demo_TestPlanning
```
The path of the source files IHBFMT*.h and IHBFMT*.cpp for IHBFMT* is:
```ompl-1.3.1-2d-IHBFMTstart-beta/src/ompl/geometric/planners/fmt```

# Reference paper: 
J. Xu, C. Huang, L. Li, B. Zhang, J. Chen, Y. He, & Z. Wei. (2025). An asymptotically optimal sampling-based method for high-dimensional complex motion planning using novel efficient heuristics. Robotics and Autonomous Systems. (Under Review)
