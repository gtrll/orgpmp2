# \file orgpmp2.py
# \brief Python interface to orgpmp2.

import types
import openravepy

def bind(mod):
   mod.viewspheres = types.MethodType(viewspheres,mod)
   mod.computedistancefield = types.MethodType(computedistancefield,mod)
   mod.create = types.MethodType(create,mod)
   mod.gettraj = types.MethodType(gettraj,mod)
   mod.destroy = types.MethodType(destroy,mod)
   mod.rungpmp2 = types.MethodType(rungpmp2,mod)

def shquot(s):
   return "'" + s.replace("'","'\\''") + "'"
   
def viewspheres(mod, robot=None, releasegil=False):
   cmd = 'viewspheres'
   if robot is not None:
      if hasattr(robot,'GetName'):
         cmd += ' robot %s' % shquot(robot.GetName())
      else:
         cmd += ' robot %s' % shquot(robot)
   print 'cmd:', cmd
   return mod.SendCommand(cmd, releasegil)

def computedistancefield(mod, res=None, centroid=None, extents=None,
   cache_filename=None, save_sdf=None, releasegil=False):
   cmd = 'computedistancefield'
   if res is not None:
      cmd += ' res %f' % res
   if centroid is not None:
      cmd += ' centroid %s' % shquot(' '.join([str(v) for v in centroid]))
   if extents is not None:
      cmd += ' extents %s' % shquot(' '.join([str(v) for v in extents]))
   if cache_filename is not None:
      cmd += ' cache_filename %s' % shquot(cache_filename)
   if save_sdf is not None:
      cmd += ' save_sdf %d' % save_sdf
   print 'cmd:', cmd
   return mod.SendCommand(cmd, releasegil)

def create(mod, robot=None, end_conf=None, base_pose=None,
   dh_a=None, dh_alpha=None, dh_d=None, dh_theta=None,
   robot_idx=None, link_idx=None,
   starttraj=None, total_step=None, obs_check_inter=None, output_inter=None,
   total_time=None, fix_pose_sigma=None, fix_vel_sigma=None, 
   cost_sigma=None, hinge_loss_eps=None, Qc=None,
   save_info=None, releasegil=False, **kwargs):
   cmd = 'create'
   # robot params
   if robot is not None:
      if hasattr(robot,'GetName'):
         cmd += ' robot %s' % shquot(robot.GetName())
      else:
         cmd += ' robot %s' % shquot(robot)
   if end_conf is not None:
      cmd += ' end_conf %s' % shquot(' '.join([str(v) for v in end_conf]))
   if base_pose is not None:
      cmd += ' base_pose %s' % shquot(' '.join([str(v) for v in base_pose]))
   if dh_a is not None:
      cmd += ' dh_a %s' % shquot(' '.join([str(v) for v in dh_a]))
   if dh_alpha is not None:
      cmd += ' dh_alpha %s' % shquot(' '.join([str(v) for v in dh_alpha]))
   if dh_d is not None:
      cmd += ' dh_d %s' % shquot(' '.join([str(v) for v in dh_d]))
   if dh_theta is not None:
      cmd += ' dh_theta %s' % shquot(' '.join([str(v) for v in dh_theta]))
   if robot_idx is not None:
      cmd += ' robot_idx %s' % shquot(' '.join([str(v) for v in robot_idx]))
   if link_idx is not None:
      cmd += ' link_idx %s' % shquot(' '.join([str(v) for v in link_idx]))
   # traj params
   if starttraj is not None:
      in_traj_data = starttraj.serialize(0)
      cmd += ' starttraj %s' % shquot(in_traj_data)
   if total_step is not None:
      cmd += ' total_step %d' % total_step
   if obs_check_inter is not None:
      cmd += ' obs_check_inter %d' % obs_check_inter
   if output_inter is not None:
      cmd += ' output_inter %d' % output_inter
   if total_time is not None:
      cmd += ' total_time %f' % total_time
   # graph params
   if fix_pose_sigma is not None:
      cmd += ' fix_pose_sigma %f' % fix_pose_sigma
   if fix_vel_sigma is not None:
      cmd += ' fix_vel_sigma %f' % fix_vel_sigma
   if cost_sigma is not None:
      cmd += ' cost_sigma %f' % cost_sigma
   if hinge_loss_eps is not None:
      cmd += ' hinge_loss_eps %f' % hinge_loss_eps
   if Qc is not None:
      cmd += ' Qc %f' % Qc
   # misc
   if save_info is not None:
      cmd += ' save_info %d' % save_info
   print 'cmd:', cmd
   return mod.SendCommand(cmd, releasegil)

def gettraj(mod, run=None, no_collision_check=None, no_collision_exception=None,
      no_collision_details=None, releasegil=False):
   cmd = 'gettraj'
   if run is not None:
      cmd += ' run %s' % run
   if no_collision_check is not None and no_collision_check:
      cmd += ' no_collision_check'
   if no_collision_exception is not None and no_collision_exception:
      cmd += ' no_collision_exception'
   if no_collision_details is not None and no_collision_details:
      cmd += ' no_collision_details'
   out_traj_data = mod.SendCommand(cmd, releasegil)
   return openravepy.RaveCreateTrajectory(mod.GetEnv(),'').deserialize(out_traj_data)
   
def destroy(mod, run=None, releasegil=False):
   cmd = 'destroy'
   if run is not None:
      cmd += ' run %s' % run
   return mod.SendCommand(cmd, releasegil)

def rungpmp2(mod,
      # gettraj args
      no_collision_check=None, no_collision_exception=None, no_collision_details=None,
      releasegil=False, **kwargs):
   # pass unknown args to create
   run = create(mod, releasegil=releasegil, **kwargs)
   traj = gettraj(mod, run=run,
      no_collision_check=no_collision_check,
      no_collision_exception=no_collision_exception,
      no_collision_details=no_collision_details,
      releasegil=releasegil)
   destroy(mod, run=run, releasegil=releasegil)
   return traj
