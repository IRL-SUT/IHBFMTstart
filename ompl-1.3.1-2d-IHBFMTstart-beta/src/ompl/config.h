/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef OMPL_CONFIG_
#define OMPL_CONFIG_

/** \brief The ompl version */
#define OMPL_VERSION "1.3.1"

#define OMPL_MAJOR_VERSION 1
#define OMPL_MINOR_VERSION 3
#define OMPL_PATCH_VERSION 1

#define OMPL_VERSION_VALUE ( OMPL_MAJOR_VERSION * 1000000       \
                             + OMPL_MINOR_VERSION * 1000        \
                             + OMPL_PATCH_VERSION)

/** \brief Specify whether the MORSE extension is built */
#define OMPL_EXTENSION_MORSE 1

/** \brief Specify whether the OpenDE extension is built */
#define OMPL_EXTENSION_OPENDE 1

/** \brief Specify whether Eigen is installed */
#define OMPL_HAVE_EIGEN3 1

/** \brief Whether FLANN is installed */
#define OMPL_HAVE_FLANN 1

#endif
