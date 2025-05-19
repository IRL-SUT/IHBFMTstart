# IHBFMTstart（beta version）
An asymptotically optimal sampling-based method for high-dimensional complex motion planning.
# Dependencies
The Open Motion Planning Library (OMPL) 
Install dependencies according to OMPLapp 1.4.0 requirements.
# Installation
Pull this repository to /omplapp-1.4.0-Source-IHBFMTstart-beta.
```
mkdir -p build/Release
cd build/Release
cmake ../..
# next step is optional
make -j 4 update_bindings # if you want to use the GUI or Python bindings
make -j 4 # replace "4" with the number of cores on your machine
```
# Environments
The library contains a selection of 2D and 3D environments to choose from. 




# Usage



# Reference paper: 
J. Xu, C. Huang, L. Li, B. Zhang, J. Chen, Y. He, & Z. Wei. (2025). An asymptotically optimal sampling-based method for high-dimensional complex motion planning using novel efficient heuristics. Robotics and Autonomous Systems. (Under Review)
