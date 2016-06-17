Parameters
=========================================
Here we describe parameters associated with orgpmp2 and explain how to set them.


Signed Distance Field Parameters
-----
- **centroid**: This is the centroid of the 3D SDF cube. Set this as the base of the robot or arm being planned for.
- **extents**: This is the extent of the 3D SDF cube from the centroid in +/- x, y, and z directions. 
- **res**: This is the resolution parameter and defines the cell size of the SDF cube. For environments with small obstacles pick a finer resolution.

Note that SDF computation takes longer with smaller cell size and larger extents. All dimensions are in meters.


GPMP2 parameters
-----
More information on parameters associated with the GPMP2 algorithm can be found in [GPMP2's doc](https://github.com/gtrll/gpmp2/doc/Parameters.md).

- **robot**: openrave robot object.
- **end_conf**: goal in configuration space.
- **base_pose**: pose for base of the robot or arm.
- **dh_a**, **dh_alpha**, **dh_d**, **dh_theta**: DH parameters of the robot.
- **robot_idx** and **link_idx**: when planning for only part of the robot, for example, PR2's arm, spheres need to be matched correctly to the GPMP2 robot model. A mapping between the true index of the link on the robot (**robot_idx**) and its index in the GPMP2 model (**link_idx**) has to be provided.  
- **starttraj** (optional): initialization of the trajectory. Straight line used if not passed.
- **total_step**: total states to be optimized on the trajectory.
- **obs_check_inter**: number of states between any two optimized states where obstacle cost is evaluated.
- **output_inter**: number of states between any two optimized states for the output trajectory.
- **total_time**: Total runtime of the trajectory in seconds.
- **fix_pose_sigma** and **fix_vel_sigma**: pose/velocity prior model covariance.
- **cost_sigma**: \sigma_obs for obstacle cost that controls the balance between 'smoothness' of the trajectory and 'obstacle avoidance'. Smaller \sigma_obs refers to less smooth trajectory, with lower probability of colliding with obstacles and vice versa.
- **hinge_loss_eps**: \epsilon for hinge loss function, the 'safety distance' from obstacles.
- **Qc**: GP hyperparameter.