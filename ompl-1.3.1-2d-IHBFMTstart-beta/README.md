The Open Motion Planning Library (OMPL)
=======================================

Linux / OS X [![Build Status](https://travis-ci.org/ompl/ompl.svg?branch=master)](https://travis-ci.org/ompl/ompl)
Windows [![Build status](https://ci.appveyor.com/api/projects/status/valuv9sabye1y35n/branch/master?svg=true)](https://ci.appveyor.com/project/mamoll/ompl/branch/master)

Visit the [OMPL installation page](http://ompl.kavrakilab.org/core/installation.html) for
detailed installation instructions.

OMPL has the following required dependencies:

* [Boost](http://www.boost.org) (version 1.54 or higher)
* [CMake](http://www.cmake.org) (version 2.8.7 or higher)

The following dependencies are optional:

* [ODE](http://ode.org) (needed to compile support for planning using the Open Dynamics Engine)
* [Py++](https://bitbucket.org/ompl/ompl/src/tip/doc/markdown/installPyPlusPlus.md) (needed to generate Python bindings)
* [Doxygen](http://www.doxygen.org) (needed to create a local copy of the documentation at
  http://ompl.kavrakilab.org/core)
* [Eigen](http://eigen.tuxfamily.org) (needed for an informed sampling technique to improve the optimization of path length and for the Vector Field RRT planner)

Once dependencies are installed, you can build OMPL on Linux, OS X,
and MS Windows. Go to the top-level directory of OMPL and type the
following commands:

    mkdir -p build/Release
    cd build/Release
    cmake ../..
    # next step is optional
    make -j 4 update_bindings # if you want Python bindings
    make -j 4 # replace "4" with the number of cores on your machine

