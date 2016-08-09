import atexit
from openravepy import *
import orgpmp2.orgpmp2
import time
import types
import prpy.kin
import prpy.rave
import sys

# Initialize openrave logging
from openravepy.misc import InitOpenRAVELogging
InitOpenRAVELogging()

# Start and end joint angles
start_joints = numpy.array([1.57,1.2,0,1.7,0,-1.24,1.57])
end_joints = numpy.array([-0.26,0.6,0,1.3,0,-0.25,1.57])

# Set up openrave and load env
RaveInitialize(True, level=DebugLevel.Info)
atexit.register(RaveDestroy)
e = Environment()
atexit.register(e.Destroy)
e.Load('data/envs/lab.env.xml')
e.Load('data/robots/barrettwam_gpmp2spheres.robot.xml')
e.SetViewer('qtcoin')

# Get the robot
r = e.GetRobots()[0]
raveLogInfo("Robot "+r.GetName()+" (#links: " + str(len(r.GetLinks())) + ") has "+repr(r.GetDOF())+" joints with values:\n"+repr(r.GetDOFValues()))

# Set robot joint active
r.SetActiveManipulator('arm')
m = r.GetActiveManipulator()
r.SetActiveDOFs(m.GetArmIndices())
r.SetActiveDOFValues(start_joints)

# Load gpmp2
m_gpmp2 = RaveCreateModule(e,'orgpmp2')
orgpmp2.orgpmp2.bind(m_gpmp2)

# Compute distance field for the env
# first disable robot
r.Enable(False)
# get a distance field for env
m_gpmp2.computedistancefield(cache_filename='sdf_env_lab.dat',
  centroid=numpy.array([0,0,0]),extents=numpy.array([1.2,1.2,1.2]),res=0.02)
# enable robot
r.Enable(True)

# DH parameters of robot
alpha = numpy.array([-1.5708, 1.5708, -1.5708, 1.5708, -1.5708, 1.5708, 0])
a = numpy.array([0, 0, 0.045, -0.045, 0, 0, 0])
d = numpy.array([0, 0, 0.55, 0, 0.3, 0, 0.06])
theta = numpy.array([0, 0, 0, 0, 0, 0, 0])

# Initialize Trajectory
total_step = 10
st = RaveCreateTrajectory(e,'')
st.Init(r.GetActiveConfigurationSpecification())
zero = numpy.zeros(len(start_joints))
init_p = []
init_v = []
for i in range(total_step):
  mid = (1-i/((total_step-1)*1.0))*start_joints + i/((total_step-1)*1.0)*end_joints
  init_p = numpy.append(init_p,mid)
  init_v = numpy.append(init_v,zero)
init_traj = numpy.append(init_p,init_v)
st.Insert(0,init_traj)

# Run gpmp2
try:
  t = m_gpmp2.rungpmp2(
    robot = r,
    end_conf = end_joints,
    dh_a = a,
    dh_alpha = alpha,
    dh_d = d,
    dh_theta = theta,
    starttraj=st,
    total_step = total_step,
    obs_check_inter = 9,
    output_inter = 9,
    total_time = 1.0,
    fix_pose_sigma = 1e-3,
    fix_vel_sigma = 1e-3,
    cost_sigma = 0.02,
    hinge_loss_eps = 0.2,
    Qc = 1,
    no_report_cost = False,
    no_collision_exception = True)
except RuntimeError as ex:
  print ex
  t = None

# Run optimized trajectory
try:
  while t is not None:
    raw_input('Press [Enter] to run the trajectory ...\n')
    with e:
      r.GetController().SetPath(t)
except KeyboardInterrupt:
  print