\mainpage The Open Motion Planning Library

\htmlonly
<div id="fb-root"></div>
<script>(function(d, s, id) {
  var js, fjs = d.getElementsByTagName(s)[0];
  if (d.getElementById(id)) return;
  js = d.createElement(s); js.id = id;
  js.src = "//connect.facebook.net/en_US/all.js#xfbml=1&status=0";
  fjs.parentNode.insertBefore(js, fjs);
}(document, 'script', 'facebook-jssdk'));</script>

<div class="row"><div class="col-md-6 col-sm-12">\endhtmlonly
\if OMPLAPP
__OMPL__, the Open Motion Planning Library, consists of many state-of-the-art sampling-based motion planning algorithms. OMPL itself does not contain any code related to, e.g., collision checking or visualization. This is a deliberate design choice, so that OMPL is not tied to a particular collision checker or visualization front end. The library is designed so it can be easily integrated into [systems that provide the additional needed components](integration.html).

__OMPL.app__, the front-end for [OMPL](/core), contains a lightweight wrapper for the  [FCL](http://gamma.cs.unc.edu/FCL) and [PQP](http://gamma.cs.unc.edu/SSV) collision checkers and a simple GUI based on [PyQt](http://www.riverbankcomputing.co.uk/software/pyqt/intro) / [PySide](http://www.pyside.org). The graphical front-end can be used for planning motions for rigid bodies and a few vehicle types (first-order and second-order cars, a blimp, and a quadrotor). It relies on the [Assimp](http://assimp.sf.net) library to import a large variety of mesh formats that can be used to represent the robot and its environment.

\else
The Open Motion Planning Library (__OMPL__) consists of a set of sampling-based motion planning algorithms. The content of the library is limited to these algorithms, which means there is no environment specification, no collision detection or visualization. The library is designed so it can be easily integrated into [systems that provide the additional needed components](integration.html). For examples of complete systems using OMPL, see [OMPL.app](http://ompl.kavrakilab.org) and [MoveIt!](http://moveit.ros.org). We have also developed a [educational module on motion planning](education.html) that is centered around OMPL.app. We are looking for educational partners to use and further develop the material. Please contact us for more information.

OMPL is intended to be efficient, thread safe, easy to use, easily extensible and freely available (visit this project on [Bitbucket](https://bitbucket.org/ompl/ompl)).

\endif
\htmlonly

<a href="download.html" class="btn btn-primary">Current version: 1.3.1
<br>
<small>Released: May 18, 2025</small></a>

<a href="citations.html" class="btn btn-primary">Click for citation,<br><small>if you use OMPL in your work</small></a>
</p>

<div class="fb-like" data-href="http://www.facebook.com/pages/OMPL/320018418039567" data-send="true" data-layout="button_count" data-width="450" data-show-faces="false"></div>

</div><div class="col-md-6 col-sm-6">
  <div id="omplCarousel" class="carousel slide" data-ride="carousel">
    <!-- Indicators -->
    <ol class="carousel-indicators">
      <li data-target="#omplCarousel" data-slide-to="0" class="active"></li>
      <li data-target="#omplCarousel" data-slide-to="1"></li>
      <li data-target="#omplCarousel" data-slide-to="2"></li>
      <li data-target="#omplCarousel" data-slide-to="3"></li>
    </ol>
    <!-- Wrapper for slides -->
    <div class="carousel-inner">
      <div class="item active">
        <img src="images/mp.jpg">
      </div>
      <!--<div class="item">
        <img src="images/mp.jpg" class="hidden" style="margin-top: 1px">
        <div class="carousel-caption carousel-caption-inset"><h4>New in 0.11!</h4>
        <ul>
           <li>PlannerData now uses the Boost Graph Library; each planner can store arbitrary metadata in a graph.  All this PlannerData can easily be (de)serialized for messaging or storing/loading of planner data.
           <li>Implementation of PRM is now threaded (one thread for growing the roadmap, one thread for monitoring whether the problem is solved).
        </ul>
        <p>See <a href="releaseNotes.html">release notes</a> for details</p>
      </div>-->
      <div class="item">
        <img src="images/T-RRT.jpg" style="padding-bottom: 2px">
        <div class="carousel-caption">
          <h4>Planning with costs</h4>
          <p>Visualization of the cost map explored by the T-RRT planner.</p>
        </div>
      </div>
      <div class="item">
        <a href="http://ompl.kavrakilab.org/gui.html"><img src="images/gui_path-small.jpg"></a>
        <div class="carousel-caption">
          <h4>OMPL.app GUI</h4>
        </div>
      </div>
      <div class="item">
        <a href="http://www.ros.org/wiki/ompl"><img src="images/pr2.jpg"></a>
        <div class="carousel-caption">
          <h4>OMPL Inside</h4>
          <p>OMPL is used inside ROS to plan motions for the PR2 and many other robots.</p>
        </div>
      </div>
    </div>
    <!-- Controls -->
    <a class="carousel-control left" href="#omplCarousel" role="button" data-slide="prev">
      <span class="glyphicon glyphicon-chevron-left"></span>
    </a>
    <a class="carousel-control right" href="#omplCarousel" role="button" data-slide="next">
      <span class="glyphicon glyphicon-chevron-right"></span>
    </a>
  </div>
</div></div><div class="row"><div class="col-md-4 col-sm-6">\endhtmlonly
## Contents of This Library

- OMPL contains implementations of many sampling-based algorithms such as PRM, RRT, EST, SBL, KPIECE, SyCLOP, and several variants of these planners. See [available planners](planners.html) for a complete list.
- All these planners operate on very abstractly defined state spaces. Many commonly used [state spaces](spaces.html) are already implemented (e.g., SE(2), SE(3), R<sup>n</sup>, etc.).
- For any state space, different [state samplers](samplers.html) can be used (e.g., uniform, Gaussian, obstacle based, etc.).
- [API overview](api_overview.html)
\if OMPLAPP
- [Documentation for just the OMPL core library (i.e., without the “app” layer)](/core).
\endif

\htmlonly</div><div class="col-md-4 col-sm-6">\endhtmlonly
## Getting Started

- The [OMPL primer](http://ompl.kavrakilab.org/OMPL_Primer.pdf) provides a brief background on sampling-based motion planning, and an overview of OMPL.
- [Download](download.html) and [install](installation.html) OMPL.
\if OMPLAPP
- Learn how to use the [OMPL.app GUI](gui.html).
\endif
- [Demos](group__demos.html) and [tutorials](tutorials.html).
- [Frequently Asked Questions.](FAQ.html)
- Learn how to integrate your own code with [OMPL's build system](buildSystem.html).
- [Learn more about how OMPL is integrated within other systems](integration.html) (such as [MoveIt!](http://moveit.ros.org), [OpenRAVE](http://openrave.org), [V-REP](http://coppeliarobotics.com), and [MORSE](https://www.openrobots.org/wiki/morse)).
- If interested in using Python, make sure to read [the documentation for the Python bindings](python.html).

\htmlonly</div><div class="col-md-4 col-sm-6">\endhtmlonly
## Other Resources

- [OMPL for education.](education.html)
- [Gallery of example uses of OMPL.](gallery.html)
- If you use [ROS](http://www.ros.org), the recommended way to use OMPL is through [MoveIt!](http://moveit.ros.org).
- [Third-party contributions.](thirdparty.html) ([Contribute your own extensions!](contrib.html))

\htmlonly</div><div class="col-md-12">\endhtmlonly


## News & Events

- [At ROSCON 2013, Sachin Chitta gave a presentation about MoveIt!](https://vimeo.com/66567049), the new  software framework for motion planning in ROS. It provides a common interface to motion planning libraries (including OMPL). The old ROS arm_navigation stack is now deprecated and all ROS users are encouraged to switch to MoveIt!.
- [ICRA 2013 Tutorial on Motion Planning for Mobile Manipulation: State-of-the-art Methods and Tools](http://moveit.ros.org/wiki/Tutorials/ICRA2013). Both OMPL and [MoveIt!](http://moveit.ros.org) were heavily featured in this tutorial.
- [OMPL has won the 2012 Open Source Software World Grand Challenge!](http://ompl.kavrakilab.org/blog/?p=178)
- [An article about OMPL](ieee-ram-2012-ompl.pdf) has been accepted for publication in IEEE's Robotics & Automation Magazine! It will appear in the December 2012 issue.
- [At ROSCON 2012, Sachin Chitta and Ioan Șucan gave a talk about MoveIt!](http://www.youtube.com/watch?v=r1zbuLc8RhI), the new motion planning stack in ROS. It provides a common interface to motion planning libraries in ROS (including OMPL). It will eventually replace the arm navigation stack.
- [IROS 2011 Tutorial on Motion Planning for Real Robots](http://kavrakilab.org/OMPLtutorial). This hands-on tutorial described how to use the ROS and OMPL, but it also provided some background on sampling-based motion planning.

\htmlonly</div></div></div>\endhtmlonly
