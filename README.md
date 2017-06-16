orgpmp2
===================================================
orgpmp2 is a OpenRAVE plugin for GPMP2 (Gaussian Process Motion Planner 2) algorithm described in [Motion Planning as Probabilistic Inference using Gaussian Processes and Factor Graphs](http://www.cc.gatech.edu/~bboots3/files/GPMP2.pdf) (RSS 2016). Examples provided for WAM arm and PR2 use python scripts.

orgpmp2 is being developed by [Mustafa Mukadam](mailto:mmukadam3@gatech.edu) and [Jing Dong](mailto:thu.dongjing@gmail.com) as part of their work at Georgia Tech Robot Learning Lab and is a modified version of [CHOMP openrave plugin](https://github.com/personalrobotics/or_cdchomp).

Compilation & Installation
------

- Install [ROS](http://wiki.ros.org/indigo/Installation/Ubuntu). We have tested ROS indigo.
- Install [OpenRAVE](http://openrave.org/). If you have trouble compiling the original version, please check out and install [our fork](https://github.com/gtrll/openrave) which has fixed few minor bugs.
- Install [GPMP2](https://github.com/gtrll/gpmp2) core C++ library.
- Install a few additional dependencies

  ```bash
  sudo apt-get install gfortran libgsl0-dev python-enum
  ```

- Initialize a catkin workspace (if you use a existing catkin workspace this step is not needed)

  ```bash
  mkdir -p ~/gpmp2_catkin_ws/src
  cd gpmp2_catkin_ws/src
  catkin_init_workspace
  ```

- Add orgpmp2, [PrPy](https://github.com/personalrobotics/prpy) (python library for OpenRAVE), [openrave_catkin](https://github.com/personalrobotics/openrave_catkin) (utility package for OpenRAVE) to ```gpmp2_catkin_ws/src```

  ```bash
  git clone https://github.com/gtrll/orgpmp2.git
  git clone https://github.com/personalrobotics/openrave_catkin.git
  git clone https://github.com/personalrobotics/prpy.git
  ```

- Compile the catkin workspace

  ```bash
  catkin_make -DCMAKE_BUILD_TYPE=Release
  ```

- Before running the examples, the last step is setup the environment variables

  ```bash
  source ~/gpmp2_catkin_ws/devel/setup.bash
  ```

Questions & Bug reporting
-----

Please use Github issue tracker to report bugs. For other questions please contact [Mustafa Mukadam](mailto:mmukadam3@gatech.edu) or [Jing Dong](mailto:thu.dongjing@gmail.com).


Citing
-----

If you use orgpmp2 or GPMP2 in an academic context, please cite following publications:

```
@inproceedings{Dong-RSS-16,
  Author = "Jing Dong and Mustafa Mukadam and Frank Dellaert and Byron Boots",
  booktitle = {Proceedings of Robotics: Science and Systems (RSS-2016)},
  Title = "Motion Planning as Probabilistic Inference using Gaussian Processes and Factor Graphs",
  year = {2016}
}
```

License
-----

orgpmp2 is released under the GPL license, reproduced in the file license-gpl.txt in this directory.
