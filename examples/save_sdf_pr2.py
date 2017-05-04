import atexit
from openravepy import *
import orgpmp2.orgpmp2
import time
import types
import prpy.kin
import prpy.rave
import sys
sys.path.append('data')
import problemsets

# Initialize openrave logging
from openravepy.misc import InitOpenRAVELogging
InitOpenRAVELogging()

# Problemset
problemset = 'industrial2'

# Start and end joint angles
n_states, states = problemsets.states(problemset)
start_joints = numpy.array(states[3])
end_joints = numpy.array(states[1])

# Set up openrave and load env
RaveInitialize(True, level=DebugLevel.Info)
atexit.register(RaveDestroy)
e = Environment()
atexit.register(e.Destroy)
e.Load(problemsets.env_file(problemset))
e.Load('data/robots/pr2_gpmp2spheres.robot.xml')
e.SetViewer('qtcoin')

# Set up robot
r = e.GetRobots()[0]
r.SetTransform(matrixFromPose(problemsets.default_base_pose(problemset)))
rave_joint_names = [joint.GetName() for joint in r.GetJoints()]
rave_inds, rave_values = [],[]
for (name,val) in zip(problemsets.joint_names(problemset), problemsets.default_joint_values(problemset)):
  if name in rave_joint_names:
    i = rave_joint_names.index(name)
    rave_inds.append(i)
    rave_values.append(val)
r.SetDOFValues(rave_values, rave_inds)
active_joint_inds = [rave_joint_names.index(name) for name in problemsets.active_joints(problemset)]
r.SetActiveDOFs(active_joint_inds, problemsets.active_affine(problemset))
r.SetActiveDOFValues(start_joints)

# Calculate arm_pose
l = r.GetLinks()
for i in l:  
  if (i.GetName() == "torso_lift_link"):
    lp1 = poseFromMatrix(i.GetTransform())
  if (i.GetName() == "r_shoulder_pan_link"):
    lp2 = poseFromMatrix(i.GetTransform())
arm_pose = numpy.array([lp1[0], lp1[1], lp1[2], lp1[3], lp2[4], lp2[5], lp2[6]])
arm_origin = numpy.array([lp2[4], lp2[5], lp2[6]])

# Load gpmp2
m_gpmp2 = RaveCreateModule(e,'orgpmp2')
orgpmp2.orgpmp2.bind(m_gpmp2)

# SDF
# remove right arm links to calculate sdf
l = r.GetLinks()
right_arm_links = ['r_shoulder_pan_link', 'r_shoulder_lift_link', 
'r_upper_arm_roll_link', 'r_upper_arm_link', 'r_elbow_flex_link', 'r_forearm_roll_link', 
'r_forearm_cam_frame', 'r_forearm_cam_optical_frame', 'r_forearm_link', 'r_wrist_flex_link', 
'r_wrist_roll_link', 'r_gripper_palm_link', 'r_gripper_l_finger_link', 'r_gripper_l_finger_tip_link', 
'r_gripper_motor_slider_link', 'r_gripper_motor_screw_link', 'r_gripper_led_frame', 
'r_gripper_motor_accelerometer_link', 'r_gripper_r_finger_link', 'r_gripper_r_finger_tip_link', 
'r_gripper_l_finger_tip_frame', 'r_gripper_tool_frame']
for i in l:
  if i.GetName() in right_arm_links:
    i.Enable(False)
# Compute distance field for the env and remaining robot
m_gpmp2.computedistancefield(cache_filename='sdf_env_'+problemset+'.dat',
  centroid=arm_origin,extents=numpy.array([1.2,1.2,1.2]),res=0.02,save_sdf=1)

raw_input()
sys.exit()
