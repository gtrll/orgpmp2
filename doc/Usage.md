Usage
=============
Before using the python script make sure all the dependencies and the plugin are installed correctly. The example python scripts are located in the [examples](../examples) folder and the robot and environment files are located in the [data](../examples/data) folder. WAM is setup for the lab environment and PR2 works with several environments. For convenience, settings based on environments for PR2 are accessed through [problemsets.py](../examples/data/problemsets.py).


Initialize module
-----
Once openrave is initialized with appropriate environment and robot xml files set the active DOF of the robot and their **start_conf**. Then create an object of orgpmp2

```python
m_gpmp2 = RaveCreateModule(e,'orgpmp2')
orgpmp2.orgpmp2.bind(m_gpmp2)
```


Signed Distance Field
-----
Signed distance field is calculated by the ```computedistancefield``` function. First disable the part of the robot being planned for and then compute the SDF. For example, [WAM](../examples/gpmp2_wam.py) is disabled completely but [PR2 example](../examples/gpmp2_pr2.py) plans for it's right arm so only that is disabled.

```python
# Compute distance field for the env
# first disable robot
r.Enable(False)
# get a distance field for env
m_gpmp2.computedistancefield(cache_filename='sdf_env_lab.dat',
  centroid=numpy.array([0,0,0]),extents=numpy.array([1.2,1.2,1.2]),res=0.02)
# enable robot
r.Enable(True)
```
See [Parameters](Parameters.md) for their description and how to set them.


Initializing trajectory
-----
**starttraj** is used to pass an initialization of the trajectory if available. If nothing is passed a straight line initialization is used by default. The trajectory can be initialized as follows

```python
st = RaveCreateTrajectory(e,'')
st.Init(r.GetActiveConfigurationSpecification())
init_traj = numpy.append(init_p,init_v) # (position, velocity)
st.Insert(0,init_traj)
```
See initializing with trajectory [example](../examples/gpmp2_wam_withtraj.py) on WAM for more details.


Running GPMP2
-----
After setting up the environment and calculating the SDF the GPMP2 algorithm can be run

```python
# Run gpmp2
try:
  t = m_gpmp2.rungpmp2(...parameters go here...)
except RuntimeError as ex:
  print ex
  t = None
```
See [Parameters](Parameters.md) for their description and how to set them.


Displaying optimized trajectory
-----
Once a solution is found it can be displayed as follows

```python
# Run optimized trajectory
try:
  while t is not None:
    raw_input('Press [Enter] to run the trajectory ...\n')
    with e:
      r.GetController().SetPath(t)
except KeyboardInterrupt:
  print
```