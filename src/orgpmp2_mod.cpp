/** 
 * \file orgpmp2_mod.cpp
 * \brief Implementation of the orgpmp2 module for gpmp2
 */

#include <time.h>
#include <cblas.h>

extern "C" {
#include "utils/grid.h"
#include "utils/grid_flood.h"
#include "utils/kin.h"
#include "utils/mat.h"
#include "utils/util_shparse.h"
}

#include <openrave/openrave.h>
#include <openrave/planningutils.h>

#include <gtsam/base/timing.h>

#include <gpmp2/kinematics/Arm.h>
#include <gpmp2/obstacle/SignedDistanceField.h>
#include <gpmp2/planner/BatchTrajOptimizer.h>
#include <gpmp2/planner/TrajUtils.h>
#include <gpmp2/utils/Timer.h>
#include <gpmp2/utils/OpenRAVEutils.h>

#include "orgpmp2_kdata.h"
#include "orgpmp2_mod.h"

#define SAVE_TRAJ_CSV 0

namespace orgpmp2
{

struct run_rsdf
{
  double pose_world_gsdf[7];
  double pose_gsdf_world[7];
  struct cd_grid * grid;
};

struct run_sphere
{
  struct run_sphere * next;
  double radius;
  OpenRAVE::KinBody::Link * robot_link;
  int robot_linkindex;
  double pos_wrt_link[3];
};

struct sdf
{
  char kinbody_name[256];
  double pose[7]; /* pose of the grid w.r.t. the kinbody frame */
  cd_grid * grid;
};

/* this encodes a run of gpmp2 */
struct run
{
  /* the trajectory */
  double * traj;
  int n_points; /* points, including endpoints */

  /* for interfacing to openrave */
  OpenRAVE::RobotBase * robot;
  int n_adof;
  int * adofindices;

  /* gtsam parameters */
  int total_step;
  int check_inter;
  int output_inter;
  double delta_t;
  double fix_pose_sigma;
  double fix_vel_sigma;
  double cost_sigma;
  double hinge_loss_eps;
  int save_info;

  /* all spheres (first n_spheres_active are active) */
  struct run_sphere * spheres;
  int n_spheres;
  int n_spheres_active; 

  /* rooted sdfs */
  int n_rsdfs;
  struct run_rsdf * rsdfs;
};

/**
 * utilities
 */
int replace_1_to_0(double * val, void * rptr)
{
  if (*val == 1.0)
  {
    *val = 0.0;
    return 1;
  }
  return 0;
}

/**
 * view spheres on the robot
 */
int mod::viewspheres(int argc, char * argv[], std::ostream& sout)
{
  int i;
  int si;
  OpenRAVE::EnvironmentMutex::scoped_lock lockenv(this->e->GetMutex());
  OpenRAVE::RobotBasePtr r;
  char buf[1024];
  struct orgpmp2::sphereelem * el;
  struct orgpmp2::sphere * s;
  std::vector<OpenRAVE::KinBodyPtr> vgrabbed;
  OpenRAVE::KinBodyPtr k;
  #if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0, 9, 0)
    std::vector<OpenRAVE::KinBody::LinkPtr> klinks;
    std::vector<OpenRAVE::KinBody::GeometryInfoPtr> vgeometries;
    OpenRAVE::KinBody::GeometryInfoPtr g;
    int klinkindex;
    int geomindex;
    OpenRAVE::KinBody::Link * klink;
  #endif

  /* parse command line arguments */
  for (i=1; i<argc; i++)
  {
    if (strcmp(argv[i],"robot")==0 && i+1<argc)
    {
    if (r.get()) throw OpenRAVE::openrave_exception("Only one robot can be passed!");
    RAVELOG_INFO("Getting robot named |%s|.\n", argv[i+1]);
    r = this->e->GetRobot(argv[++i]);
    if (!r.get()) throw OpenRAVE::openrave_exception("Could not find robot with that name!");
    RAVELOG_INFO("Using robot %s.\n", r->GetName().c_str());
    }
    else break;
  }
  if (i<argc)
  {
    for (; i<argc; i++) RAVELOG_ERROR("argument %s not known!\n", argv[i]);
    throw OpenRAVE::openrave_exception("Bad arguments!");
  }

  /* check that we have everything */
  if (!r.get()) { throw OpenRAVE::openrave_exception("Did not pass all required args!"); }

  /* view each sphere */
  r->GetGrabbed(vgrabbed);
  /*vgrabbed.push_front(r);*/
  vgrabbed.insert(vgrabbed.begin(), r);
  si = 0;
  for (i=0; i<(int)(vgrabbed.size()); i++)
  {
    k = vgrabbed[i];
    /* get kinbody spheres */
    boost::shared_ptr<orgpmp2::kdata> d = boost::dynamic_pointer_cast<orgpmp2::kdata>(k->GetReadableInterface("orgpmp2"));
    if (d) for (el=d->sphereelems; el; el=el->next)
    {
      s = el->s;
      /* make some sweet spheres! */
      OpenRAVE::KinBodyPtr sbody = OpenRAVE::RaveCreateKinBody(this->e);
      sprintf(buf, "orgpmp2_sphere_%d", si);
      sbody->SetName(buf);
      /* set its dimensions */
      {
        std::vector< OpenRAVE::Vector > svec;
        OpenRAVE::Transform t = k->GetLink(s->linkname)->GetTransform();
        OpenRAVE::Vector v = t * OpenRAVE::Vector(s->pos); /* copies 3 values */
        v.w = s->radius; /* radius */
        //printf("%.5f, %.5f, %.5f, %.5f\n",v.x,v.y,v.z,v.w);
        svec.push_back(v);
        sbody->InitFromSpheres(svec, true);
      }
      /* add the sphere */
      #if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0,8,0)
        this->e->Add(sbody);
      #else
        this->e->AddKinBody(sbody);
      #endif
      si++;
    }

    #if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0, 9, 0)
      /* load any spheres from the "spheres" geometry group */
      klinks = k->GetLinks();
      for (klinkindex=0; klinkindex<(int)(klinks.size()); klinkindex++)
      {
        klink = klinks[klinkindex].get();
        if (klink->GetGroupNumGeometries("spheres") == -1)
          continue; /* there is no "spheres" group */
        vgeometries = klink->GetGeometriesFromGroup("spheres");
        for (geomindex = 0; geomindex < (int)(vgeometries.size()); geomindex++)
        {
          g = vgeometries[geomindex];
          if (g->_type != OpenRAVE::GT_Sphere)
            throw OPENRAVE_EXCEPTION_FORMAT("link %s contains non-spherical geometry in the 'spheres' geometry group", klink->GetName().c_str(), OpenRAVE::ORE_Failed);
          /* make some sweet spheres! */
          OpenRAVE::KinBodyPtr sbody = OpenRAVE::RaveCreateKinBody(this->e);
          sprintf(buf, "orgpmp2_sphere_%d", si);
          sbody->SetName(buf);
          /* set its dimensions */
          {
            std::vector< OpenRAVE::Vector > svec;
            OpenRAVE::Transform t = klink->GetTransform();
            OpenRAVE::Vector v = t * g->_t.trans;
            v.w = g->_vGeomData[0]; /* radius */
            svec.push_back(v);
            sbody->InitFromSpheres(svec, true);
          }
          /* add the sphere */
          this->e->Add(sbody);
          si++;
        }
      }
    #endif
  }

  return 0;
}


/**
 * computes a distance field of the environment
 */
int mod::computedistancefield(int argc, char * argv[], std::ostream& sout)
{
  // variables
  int i, j, k;
  int err;
  OpenRAVE::EnvironmentMutex::scoped_lock lockenv;
  // parameters
  int world_dim = 3;
  double res; // resolution of each cell in m
  double centroid[world_dim]; // centroid of the grid in world frame
  double extents[world_dim]; // extents of the grid
  double origin[world_dim]; // origin of grid
  char * cache_filename;
  // other
  double temp;
  OpenRAVE::KinBodyPtr cube;
  size_t idx;
  struct cd_grid * g_obs;
  int gsdf_sizearray[world_dim];
  OpenRAVE::geometry::aabb< OpenRAVE::dReal > aabb;
  double pose_world_gsdf[7];
  double pose_cube[7];
  struct sdf sdf_new;
  struct timespec ticks_tic;
  struct timespec ticks_toc;
  int sdf_data_loaded;

  // lock environment
  lockenv = OpenRAVE::EnvironmentMutex::scoped_lock(this->e->GetMutex());

  // default setting
  res = 0.02; // 2 cm
  centroid[0] = 0; // x
  centroid[1] = 0; // y
  centroid[2] = 0; // z
  extents[0] = 1; // x
  extents[1] = 1; // y
  extents[2] = 1; // z
  cache_filename = 0;

  // parse command line arguments
  for (i=1; i<argc; i++)
  {
    if (strcmp(argv[i],"centroid")==0 && i+1<argc)
    {
      char ** centroid_argv = 0;
      cd_util_shparse(argv[++i], &world_dim, &centroid_argv);
      for (j=0; j<world_dim; j++)
        centroid[j] = atof(centroid_argv[j]);
      free(centroid_argv);
    }
    else if (strcmp(argv[i],"extents")==0 && i+1<argc)
    {
      char ** extents_argv = 0;
      cd_util_shparse(argv[++i], &world_dim, &extents_argv);
      for (j=0; j<world_dim; j++)
        extents[j] = atof(extents_argv[j]);
      free(extents_argv);
    }
    else if (strcmp(argv[i],"res")==0 && i+1<argc)
      res = atof(argv[++i]);
    else if (strcmp(argv[i],"cache_filename")==0 && i+1<argc)
      cache_filename = argv[++i];
    else {
      RAVELOG_ERROR("argument %s not known!\n", argv[i]);
      throw OpenRAVE::openrave_exception("Bad arguments!");
    }
  }

  // display passed arguments
  RAVELOG_INFO("Using centroid |%f,%f,%f|.\n", centroid[0], centroid[1], centroid[2]);
  RAVELOG_INFO("Using extents |%f,%f,%f|.\n", extents[0], extents[1], extents[2]);
  RAVELOG_INFO("Using res |%f|.\n", res);
  RAVELOG_INFO("Using cache_filename |%s|.\n", cache_filename ? cache_filename : "none passed");

  // name the sdf
  strcpy(sdf_new.kinbody_name, std::string("env").c_str());

  // calculate dimension sizes (number of cells) and origin
  for (i=0; i<world_dim; i++)
  {
    if (fabs(extents[i]) <= res/2)
      gsdf_sizearray[i] = 1;
    else
      gsdf_sizearray[i] = (int) ceil(fabs(2*extents[i])/res);
    RAVELOG_INFO("gsdf_sizearray[%d]: %d\n", i, gsdf_sizearray[i]);
    origin[i] = centroid[i] - fabs(extents[i]);
  }

  // Creating grid
  temp = 1.0; // free space is 1.0 and is changed to 0.0 in floodfill
  err = cd_grid_create_sizearray(&sdf_new.grid, &temp, sizeof(double), world_dim, gsdf_sizearray);
  if (err) throw OpenRAVE::openrave_exception("Not enough memory for distance field!");

  // side length of the grid in meters
  for (i=0; i<world_dim; i++)
    sdf_new.grid->lengths[i] = gsdf_sizearray[i] * res;
  cd_mat_vec_print("sdf_new.grid->lengths: ", sdf_new.grid->lengths, world_dim);

  // set pose of grid w.r.t. world
  cd_kin_pose_identity(sdf_new.pose);
  for (i=0; i<world_dim; i++)
    sdf_new.pose[i] = origin[i];
  cd_mat_vec_print("pose_gsdf: ", sdf_new.pose, 7);

  // no sdf grid data available yet
  sdf_data_loaded = 0;

  // attempt to load the sdf from file
  if (cache_filename) do
  {
    FILE * fp;
    RAVELOG_INFO("reading sdf data from file %s ...\n", cache_filename);
    fp = fopen(cache_filename, "rb");
    if (!fp) { RAVELOG_ERROR("could not read from file!\n"); break; }

    // check file size
    fseek(fp, 0L, SEEK_END); 
    // Case where need to recompute the sdf even though the name of the file is correct
    if (ftell(fp) != sdf_new.grid->cell_size * sdf_new.grid->ncells) // cell_size x ncells
    {
      RAVELOG_ERROR("cached file size %lu bytes doesn't match expected size %lu! recomputing ...\n",
      ftell(fp), sdf_new.grid->cell_size * sdf_new.grid->ncells);
      fclose(fp);
      break;
    }

    fseek(fp, 0L, SEEK_SET);
    // read grid data
    i = fread(sdf_new.grid->data, sdf_new.grid->cell_size, sdf_new.grid->ncells, fp);
    if (i != sdf_new.grid->ncells)
    {
      RAVELOG_ERROR("error, couldn't read all the sdf data from the file!\n");
      fclose(fp);
      break;
    }
    fclose(fp);
    sdf_data_loaded = 1;
  } while (0);

  // if no file or incorrect calculate and save a new sdf
  if (!sdf_data_loaded)
  {
    // create the obstacle grid (same size as sdf_new.grid)
    err = cd_grid_create_copy(&g_obs, sdf_new.grid);
    if (err)
    {
      cd_grid_destroy(sdf_new.grid);
      throw OpenRAVE::openrave_exception("Not enough memory for distance field!");
    }

    // use a res size cube to compute occupancy in the grid
    cube = OpenRAVE::RaveCreateKinBody(this->e);
    cube->SetName("cube");
    // set its dimensions
    {
      std::vector<OpenRAVE::AABB> vaabbs(1);
      vaabbs[0].extents = OpenRAVE::Vector(res/2, res/2, res/2); // extents = half side lengths
      cube->InitFromBoxes(vaabbs, 1);
    }
    // add the cube
    #if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0,8,0)
      this->e->Add(cube);
    #else
      this->e->AddKinBody(cube);
    #endif
    // get the pose_world_gsdf = sdf_new.pose
    {
      cd_kin_pose_identity(pose_world_gsdf);
      cd_kin_pose_compose(pose_world_gsdf, sdf_new.pose, pose_world_gsdf);
    }
    int collisions = 0;
    // go through the grid, testing for collision: collisions are HUGE_VAL, free are 1.0
    RAVELOG_INFO("computing occupancy grid ...\n");
    // Go through all the cells in the grid
    OpenRAVE::Transform t;
    for (idx=0; idx<g_obs->ncells; idx++)
    {
      // Print out progress info
      if (idx % 100000 == 0)
      RAVELOG_INFO("idx=%d (%5.1f%%)...\n", (int)idx, (100.0*((double)idx)/((double)g_obs->ncells)));
      // set cube location
      t.identity();
      cd_kin_pose_identity(pose_cube);
      // Get the center of the cell
      cd_grid_center_index(g_obs, idx, pose_cube);
      cd_kin_pose_compose(pose_world_gsdf, pose_cube, pose_cube);
      t.trans.x = pose_cube[0];
      t.trans.y = pose_cube[1];
      t.trans.z = pose_cube[2];
      t.rot.y = pose_cube[3];
      t.rot.z = pose_cube[4];
      t.rot.w = pose_cube[5];
      t.rot.x = pose_cube[6];
      // Move cube to the cell
      cube->SetTransform(t);
      // collision check
      if (this->e->CheckCollision(cube))
      {
        *(double *)cd_grid_get_index(g_obs, idx) = HUGE_VAL;
        collisions++;
      }
    }
    RAVELOG_INFO("found %d/%d collisions!\n", collisions, (int)(g_obs->ncells));
    // remove cube
    this->e->Remove(cube);

    // assume the point in the very corner x=y=z is free
    RAVELOG_INFO("performing flood fill ...\n");
    idx = 0;
    // Flood fill spanning from the first cell
    cd_grid_flood_fill(g_obs, idx, 0, (int (*)(void *, void *))replace_1_to_0, 0);
    // change any remaining 1.0 cells to HUGE_VAL (assumed inside of obstacles)
    for (idx=0; idx<g_obs->ncells; idx++)
      if (*(double *)cd_grid_get_index(g_obs, idx) == 1.0)
        *(double *)cd_grid_get_index(g_obs, idx) = HUGE_VAL;

    // compute the signed distance field (in the module instance)
    RAVELOG_INFO("computing signed distance field ...\n");
    cd_grid_double_bin_sdf(&sdf_new.grid, g_obs);

    // obstacle grid no longer needed
    cd_grid_destroy(g_obs);

    // if passed a cache_filename, save the computed sdf
    if (cache_filename)
    {
      FILE * fp;
      RAVELOG_INFO("saving sdf data to file %s ...\n", cache_filename);
      fp = fopen(cache_filename, "wb");
      i = fwrite(sdf_new.grid->data, sdf_new.grid->cell_size, sdf_new.grid->ncells, fp);
      fclose(fp);
      if (i != sdf_new.grid->ncells)
      RAVELOG_ERROR("error, couldn't write the sdf data to the file!\n");
    }
  }

  // allocate a new sdf struct, and copy the new one there
  this->sdfs = (struct sdf *) realloc(this->sdfs, (this->n_sdfs+1)*sizeof(struct sdf));
  this->sdfs[this->n_sdfs] = sdf_new;
  this->n_sdfs++;

  // assign sdf to gtsam variable
  RAVELOG_INFO("Saving sdf to gtsam variable.\n");

  {
    using namespace std;
    using namespace gtsam;

    // sdf to Eigen style, grid[i][j][k] = grid[i*gsize_y*gsize_z + j*gsize_z + k]
    int gsize_x = this->sdfs->grid->sizes[0];
    int gsize_y = this->sdfs->grid->sizes[1];
    int gsize_z = this->sdfs->grid->sizes[2];
    vector<Matrix> env_sdf;
    Matrix temp_mat(gsize_y, gsize_x);
    for (k=0; k<gsize_z; k++) {
      temp_mat.setZero();
      for (i=0; i<gsize_x; i++)
        for (j=0; j<gsize_y; j++)
          temp_mat(j,i) = *(double *)cd_grid_get_index(this->sdfs[0].grid,i*gsize_y*gsize_z + j*gsize_z + k);
      env_sdf.push_back(temp_mat);
    }
    // origin of grid in world frame
    Point3 gorigin(origin[0],origin[1],origin[2]);

    this->gtsam_sdf = gpmp2::SignedDistanceField(gorigin, res, env_sdf);
/*
    // save to text file
    std::ofstream myFile;
    myFile.open((std::string(getenv("HOME"))+"/Desktop/sdf.txt").c_str(), std::ios::app);
    for (k=0; k<gsize_z; k++) {
      for (i=0; i<gsize_y; i++) {
        for (j=0; j<gsize_x; j++) {
          myFile<<env_sdf[k](i,j)<<"\t";
        }
        myFile<<std::endl;
      }
      myFile<<std::endl;
      myFile<<std::endl;
    }
    myFile.close();
*/
  }
  return 0;
}

/**
 * create gpmp2 and use gtsam internally for optimization
 */
int mod::create(int argc, char * argv[], std::ostream& sout)
{
  int i, j, k;
  int err;
  struct orgpmp2::sphere * s;
  const char * exc = 0;
  struct run * r = 0;
  int n_adofgoal = 0, n_base_pose = 0, n_dh_a = 0, n_dh_alpha = 0, n_dh_d = 0, n_dh_theta = 0, n_robot_idx = 0, n_link_idx = 0;
  double *adofgoal = 0, *base_pose = 0, *dh_a = 0, *dh_alpha = 0, *dh_d = 0, *dh_theta = 0, *robot_idx = 0, *link_idx = 0;
  double Qc = 1;
  std::map<int,int> robot_link_idx;

  /* lock environment; other temporaries */
  OpenRAVE::EnvironmentMutex::scoped_lock lockenv(this->e->GetMutex());
  std::vector< OpenRAVE::dReal > vec_jlimit_lower;
  std::vector< OpenRAVE::dReal > vec_jlimit_upper;
  struct run_sphere * s_inactive_head;
  OpenRAVE::TrajectoryBasePtr starttraj;

  r = (struct run *) malloc(sizeof(struct run));
  if (!r) throw OpenRAVE::openrave_exception("no memory!");

  /* initialize */
  r->traj = 0;
  r->n_points = 5; // 3 internal points
  r->robot = 0;
  r->n_adof = 0;
  r->adofindices = 0;
  r->spheres = 0;
  r->n_spheres = 0;
  r->n_spheres_active = 0;
  r->n_rsdfs = 0;
  r->rsdfs = 0;
  r->save_info = 0;
  
  /* temp cache var*/
  double total_time;

  /* parse command line arguments */
  for (i=1; i<argc; i++)
  {
    if (strcmp(argv[i],"robot")==0 && i+1<argc)
    {
      if (r->robot) { exc = "Only one robot can be passed!"; goto error; }
      OpenRAVE::RobotBasePtr rob = this->e->GetRobot(argv[++i]);
      r->robot = rob.get();
      if (!r->robot) { exc = "Could not find robot with that name!"; goto error; }
    }
    else if (strcmp(argv[i],"end_conf")==0 && i+1<argc)
    {
      if (adofgoal) { exc = "Only one adofgoal can be passed!"; goto error; }
      {
        char ** adofgoal_argv = 0;
        cd_util_shparse(argv[++i], &n_adofgoal, &adofgoal_argv);
        adofgoal = (double *) malloc(n_adofgoal * sizeof(double));
        for (j=0; j<n_adofgoal; j++)
          adofgoal[j] = atof(adofgoal_argv[j]);
        free(adofgoal_argv);
      }
    }
    else if (strcmp(argv[i],"base_pose")==0 && i+1<argc)
    {
      if (base_pose) { exc = "Only one base_pose can be passed!"; goto error; }
      {
        char ** base_pose_argv = 0;
        cd_util_shparse(argv[++i], &n_base_pose, &base_pose_argv);
        if (n_base_pose != 7) { exc = "Incorrect pose; pose = [1x4 quaternion, 1x3 position]"; goto error; }
        base_pose = (double *) malloc(n_base_pose * sizeof(double));
        for (j=0; j<n_base_pose; j++)
          base_pose[j] = atof(base_pose_argv[j]);
        free(base_pose_argv);
      }
    }
    else if (strcmp(argv[i],"dh_a")==0 && i+1<argc)
    {
      if (dh_a) { exc = "Only one dh_a can be passed!"; goto error; }
      {
        char ** dh_a_argv = 0;
        cd_util_shparse(argv[++i], &n_dh_a, &dh_a_argv);
        dh_a = (double *) malloc(n_dh_a * sizeof(double));
        for (j=0; j<n_dh_a; j++)
          dh_a[j] = atof(dh_a_argv[j]);
        free(dh_a_argv);
      }
    }
    else if (strcmp(argv[i],"dh_alpha")==0 && i+1<argc)
    {
      if (dh_alpha) { exc = "Only one dh_alpha can be passed!"; goto error; }
      {
        char ** dh_alpha_argv = 0;
        cd_util_shparse(argv[++i], &n_dh_alpha, &dh_alpha_argv);
        dh_alpha = (double *) malloc(n_dh_alpha * sizeof(double));
        for (j=0; j<n_dh_alpha; j++)
          dh_alpha[j] = atof(dh_alpha_argv[j]);
        free(dh_alpha_argv);
      }
    }
    else if (strcmp(argv[i],"dh_d")==0 && i+1<argc)
    {
      if (dh_d) { exc = "Only one dh_d can be passed!"; goto error; }
      {
        char ** dh_d_argv = 0;
        cd_util_shparse(argv[++i], &n_dh_d, &dh_d_argv);
        dh_d = (double *) malloc(n_dh_d * sizeof(double));
        for (j=0; j<n_dh_d; j++)
          dh_d[j] = atof(dh_d_argv[j]);
        free(dh_d_argv);
      }
    }
    else if (strcmp(argv[i],"dh_theta")==0 && i+1<argc)
    {
      if (dh_theta) { exc = "Only one dh_theta can be passed!"; goto error; }
      {
        char ** dh_theta_argv = 0;
        cd_util_shparse(argv[++i], &n_dh_theta, &dh_theta_argv);
        dh_theta = (double *) malloc(n_dh_theta * sizeof(double));
        for (j=0; j<n_dh_theta; j++)
          dh_theta[j] = atof(dh_theta_argv[j]);
        free(dh_theta_argv);
      }
    }
    else if (strcmp(argv[i],"robot_idx")==0 && i+1<argc)
    {
      if (robot_idx) { exc = "Only one robot_idx can be passed!"; goto error; }
      {
        char ** robot_idx_argv = 0;
        cd_util_shparse(argv[++i], &n_robot_idx, &robot_idx_argv);
        robot_idx = (double *) malloc(n_robot_idx * sizeof(double));
        for (j=0; j<n_robot_idx; j++)
          robot_idx[j] = atof(robot_idx_argv[j]);
        free(robot_idx_argv);
      }
    }
    else if (strcmp(argv[i],"link_idx")==0 && i+1<argc)
    {
      if (link_idx) { exc = "Only one link_idx can be passed!"; goto error; }
      {
        char ** link_idx_argv = 0;
        cd_util_shparse(argv[++i], &n_link_idx, &link_idx_argv);
        link_idx = (double *) malloc(n_link_idx * sizeof(double));
        for (j=0; j<n_link_idx; j++)
          link_idx[j] = atof(link_idx_argv[j]);
        free(link_idx_argv);
      }
    }
    else if (strcmp(argv[i],"starttraj")==0 && i+1<argc)
    {
      if (starttraj.get()) { exc = "Only one starttraj can be passed!"; goto error; }
      starttraj = RaveCreateTrajectory(this->e);
      std::string my_string(argv[++i]);
      std::istringstream ser_iss(my_string);
      starttraj->deserialize(ser_iss);
    }
    else if (strcmp(argv[i],"total_step")==0 && i+1<argc)
      r->total_step = atoi(argv[++i]);
    else if (strcmp(argv[i],"obs_check_inter")==0 && i+1<argc)
      r->check_inter = atoi(argv[++i]);
    else if (strcmp(argv[i],"output_inter")==0 && i+1<argc)
      r->output_inter = atoi(argv[++i]);
    else if (strcmp(argv[i],"total_time")==0 && i+1<argc)
      total_time = atof(argv[++i]);
    else if (strcmp(argv[i],"fix_pose_sigma")==0 && i+1<argc)
      r->fix_pose_sigma = atof(argv[++i]);
    else if (strcmp(argv[i],"fix_vel_sigma")==0 && i+1<argc)
      r->fix_vel_sigma = atof(argv[++i]);
    else if (strcmp(argv[i],"cost_sigma")==0 && i+1<argc)
      r->cost_sigma = atof(argv[++i]);
    else if (strcmp(argv[i],"hinge_loss_eps")==0 && i+1<argc)
      r->hinge_loss_eps = atof(argv[++i]);
    else if (strcmp(argv[i],"Qc")==0 && i+1<argc)
      Qc = atof(argv[++i]);
    else if (strcmp(argv[i],"save_info")==0 && i+1<argc)
      r->save_info = atoi(argv[++i]);
    else break;
  }
  
  // parse read tmp vars, get n_points, step_interval, delta_t
  r->n_points = r->total_step * (r->output_inter + 1) + 1;
  r->delta_t = total_time / r->total_step;
  
  if (i<argc)
  {
    for (; i<argc; i++) RAVELOG_ERROR("argument %s not known!\n", argv[i]);
    throw OpenRAVE::openrave_exception("Bad arguments!");
  }

  RAVELOG_INFO("Using robot %s.\n", r->robot->GetName().c_str());

  /* check validity of input arguments ... */
  if (!r->robot) { exc = "Did not pass a robot!"; goto error; }
  if (!adofgoal) { exc = "Did not pass end_conf (goal in configuration space)!"; goto error; }
  if (!this->n_sdfs) { exc = "No signed distance fields have yet been computed!"; goto error; }
  if (r->n_points < 3) { exc = "n_points must be >=3!"; goto error; }
  if (!dh_a || !dh_alpha || !dh_d || !dh_theta) { exc = "Did not pass DH parameters!"; goto error; }

  /* get optimizer degrees of freedom */
  r->n_adof = r->robot->GetActiveDOF();
  /* check that n_adofgoal matches active dof */
  if (adofgoal && (n_adofgoal != r->n_adof))
  {
    RAVELOG_INFO("n_adof: %d; n_adofgoal: %d\n", r->n_adof, n_adofgoal);
    exc = "size of adofgoal does not match active dofs!";
    goto error;
  }
  /* check that n_dh matches active dof */
  if (n_dh_a != r->n_adof || n_dh_alpha != r->n_adof || n_dh_d != r->n_adof || n_dh_theta != r->n_adof)
  {
    exc = "size of DH parameters does not match active dofs!";
    goto error;
  }
  /* allocate adofindices */
  r->adofindices = (int *) malloc(r->n_adof * sizeof(int));
  {
    std::vector<int> vec = r->robot->GetActiveDOFIndices();
    for (j=0; j<r->n_adof; j++)
      r->adofindices[j] = vec[j];
  }
  /* check if pose passed */
  if (!base_pose)
  {
    RAVELOG_INFO("No base_pose passed, assuming arm at world origin\n");
    base_pose = (double *) malloc(7 * sizeof(double));
    base_pose[0] = 1.0;
    for (j=1; j<7; j++)
      base_pose[j] = 0.0;
  }
  /* create map of robot_idx and link_idx if passed */
  if (robot_idx && link_idx)
  {
    if (n_robot_idx != n_link_idx) { exc = "Robot and link index sizes do not match!"; goto error; }
    for (j=0; j<n_robot_idx; j++)
      robot_link_idx[robot_idx[j]] = link_idx[j];
  }
  else
    RAVELOG_INFO("No mapping between robot and link idx passed. Default set as 0 to dof.\n");

  /* allocate sphere stuff */
  {
    struct run_sphere * s_new;
    struct run_sphere * s_active_tail;
    struct sphereelem * sel;
    OpenRAVE::KinBody::Link * link;
    int linkindex;
    std::vector<OpenRAVE::KinBodyPtr> vgrabbed;
    OpenRAVE::KinBodyPtr k;
    boost::shared_ptr<orgpmp2::kdata> d;
    #if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0, 9, 0)
      std::vector<OpenRAVE::KinBody::LinkPtr> klinks;
      int klinkindex;
      std::vector<OpenRAVE::KinBody::GeometryInfoPtr> vgeometries;
      int geomindex;
      OpenRAVE::KinBody::GeometryInfoPtr g;
      OpenRAVE::KinBody::Link * klink;
    #endif

    s_active_tail = 0; /* keep track of first active sphere inserted (will be tail) */

    /* consider the robot kinbody, as well as all grabbed bodies */
    r->robot->GetGrabbed(vgrabbed);
    vgrabbed.insert(vgrabbed.begin(), this->e->GetRobot(r->robot->GetName()));

    for (i=0; i<(int)(vgrabbed.size()); i++)
    {
      std::vector<struct run_sphere *> k_spheres;

      k = vgrabbed[i];

      /* get kinbody spheres */
      d = boost::dynamic_pointer_cast<orgpmp2::kdata>(k->GetReadableInterface("orgpmp2"));
      if (d) for (sel=d->sphereelems; sel; sel=sel->next)
      {
        /* what robot link is this sphere attached to? */
        if (k.get() == r->robot)
          link = r->robot->GetLink(sel->s->linkname).get();
        else
          link = r->robot->IsGrabbing(k).get();
        if(!link){ throw OPENRAVE_EXCEPTION_FORMAT("link %s in <orgpmp2> does not exist.", sel->s->linkname, OpenRAVE::ORE_Failed); }

        linkindex = link->GetIndex();

        /* make a new sphere */
        s_new = (struct run_sphere *) malloc(sizeof(struct run_sphere));
        s_new->radius = sel->s->radius;
        s_new->robot_link = link;
        s_new->robot_linkindex = linkindex;
        if (k.get()==r->robot)
        {
          cd_mat_memcpy(s_new->pos_wrt_link, sel->s->pos, 3, 1);
        }
        else
        {
          OpenRAVE::Transform T_w_klink = k->GetLink(sel->s->linkname)->GetTransform();
          OpenRAVE::Transform T_w_rlink = link->GetTransform();
          OpenRAVE::Vector v = T_w_rlink.inverse() * T_w_klink * OpenRAVE::Vector(sel->s->pos);
          s_new->pos_wrt_link[0] = v.x;
          s_new->pos_wrt_link[1] = v.y;
          s_new->pos_wrt_link[2] = v.z;
        }
        k_spheres.push_back(s_new);
      }

      #if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0, 9, 0)
        /* load any spheres from the "spheres" geometry group */
        klinks = k->GetLinks();
        for (klinkindex=0; klinkindex<(int)(klinks.size()); klinkindex++)
        {
          klink = klinks[klinkindex].get();
          if (klink->GetGroupNumGeometries("spheres") == -1)
            continue; /* there is no "spheres" group */
          vgeometries = klink->GetGeometriesFromGroup("spheres");
          for (geomindex = 0; geomindex < (int)(vgeometries.size()); geomindex++)
          {
            g = vgeometries[geomindex];
            if (g->_type != OpenRAVE::GT_Sphere)
              throw OPENRAVE_EXCEPTION_FORMAT("link %s contains non-spherical geometry in the 'spheres' geometry group", klink->GetName().c_str(), OpenRAVE::ORE_Failed);

            /* what robot link is this sphere attached to? */
            if (k.get() == r->robot)
              link = klink;
            else
              link = r->robot->IsGrabbing(k).get();
            if(!link){ throw OpenRAVE::openrave_exception("link does not exist!"); }

            linkindex = link->GetIndex();

            /* make a new sphere */
            s_new = (struct run_sphere *) malloc(sizeof(struct run_sphere));
            s_new->radius = g->_vGeomData[0];
            s_new->robot_link = link;
            s_new->robot_linkindex = linkindex;
            if (k.get()==r->robot)
            {
              s_new->pos_wrt_link[0] = g->_t.trans.x;
              s_new->pos_wrt_link[1] = g->_t.trans.y;
              s_new->pos_wrt_link[2] = g->_t.trans.z;
            }
            else
            {
              OpenRAVE::Transform T_w_klink = klink->GetTransform();
              OpenRAVE::Transform T_w_rlink = link->GetTransform();
              OpenRAVE::Vector v = T_w_rlink.inverse() * T_w_klink
              * OpenRAVE::Vector(g->_t.trans.x, g->_t.trans.y, g->_t.trans.z);
              s_new->pos_wrt_link[0] = v.x;
              s_new->pos_wrt_link[1] = v.y;
              s_new->pos_wrt_link[2] = v.z;
            }
            k_spheres.push_back(s_new);
          }
        } 
      #endif

      if (!k_spheres.size())
        throw OpenRAVE::openrave_exception("no spheres! kinbody does not have a <orgpmp2> tag defined?");

      /* sort spheres between active and inactive */
      for (unsigned int si=0; si<k_spheres.size(); si++)
      {
        s_new = k_spheres[si];
        /* is this link affected by the robot's active dofs? */
        for (j=0; j<r->n_adof; j++)
          if (r->robot->DoesAffect(r->adofindices[j], s_new->robot_linkindex))
            break;
        if (j<r->n_adof) /* if we're floating, call all spheres active (for now!) */
        {
          /* active; insert at head of r->spheres */
          s_new->next = r->spheres;
          r->spheres = s_new;
          r->n_spheres_active++;
          r->n_spheres++;
          if (!s_active_tail)
            s_active_tail = s_new;
        }
        else
        {
          /* inactive; insert into s_inactive_head */
          s_new->next = s_inactive_head;
          s_inactive_head = s_new;
          r->n_spheres++;
        }
      }
    }

    if (!r->n_spheres_active) { exc = "robot active dofs must have at least one sphere!"; goto error; }
    /* append the inactive spheres at the end of the active list */
    s_active_tail->next = s_inactive_head;
  }

  /* Allocate space for trajectory */
  r->traj = (double *) malloc(2*(r->n_points)*r->n_adof*sizeof(double));
  cd_mat_set_zero(r->traj, 2*r->n_points, r->n_adof);

  /* GPMP2 using gtsam */
  {
    using namespace std;
    using namespace gtsam;
    using namespace gpmp2;

    RAVELOG_INFO("Starting GPMP2\n");

    int dof = r->n_adof;
    
    // start and goal configuration
    vector<OpenRAVE::dReal> start_or;
    Vector start(dof), goal(dof);
    r->robot->GetActiveDOFValues(start_or);
    for (i=0; i<dof; i++) {
      start(i) = start_or[i];
      goal(i) = adofgoal[i];
    }

    // spheres data: vector of spheres -> pair(link_number,pair(radius,position_wrt_link))
    BodySphereVector spheres_data;
    run_sphere * currsp = r->spheres;
    for (i=0; i<r->n_spheres; i++) {
      size_t link;
      if (robot_idx && link_idx)
        if (robot_link_idx.count(currsp->robot_linkindex))
          link = robot_link_idx.at(currsp->robot_linkindex);
        else
          throw runtime_error("some robot_idx mapping is missing!");
      else
        link = currsp->robot_linkindex - 1;
      spheres_data.push_back(BodySphere(link, currsp->radius, 
          Point3(currsp->pos_wrt_link[0],currsp->pos_wrt_link[1],currsp->pos_wrt_link[2])));
      currsp = currsp->next;
    }

    // Arm settings
    Vector a(dof), alpha(dof), d(dof), theta(dof);
    Vector joint_upper_limit(dof), joint_lower_limit(dof);
    ArmModel arm;
    // DH parameters
    for (i=0; i<dof; i++) {
      a(i) = dh_a[i];
      alpha(i) = dh_alpha[i];
      d(i) = dh_d[i];
      theta(i) = dh_theta[i];
    }
    arm = ArmModel(Arm(dof, a, alpha, d, Pose3(Rot3::Quaternion(base_pose[0], base_pose[1], base_pose[2], base_pose[3]),
        Point3(base_pose[4], base_pose[5], base_pose[6])), theta), spheres_data);
    // joint limits
    r->robot->GetDOFLimits(vec_jlimit_lower, vec_jlimit_upper);
    for (i = 0; i < dof; i++) {
      joint_lower_limit(i) = vec_jlimit_lower[r->adofindices[i]];
      joint_upper_limit(i) = vec_jlimit_upper[r->adofindices[i]];
    }
      
    // planner settings
    size_t total_step = r->n_points - 1;
    size_t opt_step = r->total_step;
    
    SharedNoiseModel Qc_model = noiseModel::Isotropic::Sigma(dof, Qc);
    
    TrajOptimizerSetting opt_setting(dof);
    opt_setting.total_step      = opt_step ;
    opt_setting.total_time      = r->delta_t * opt_step;   // TODO: actual run time??
    opt_setting.epsilon         = r->hinge_loss_eps;
    opt_setting.cost_sigma      = r->cost_sigma;
    opt_setting.obs_check_inter = r->check_inter;
    opt_setting.Qc_model        = Qc_model;
 
    // timing starts from here
    Timer total_timer("total");
    total_timer.tic();
    
    // Initialize trajectory
    Values init_values, results;
    if (starttraj.get()) {
      RAVELOG_INFO("Initializing from a passed trajectory ...\n");
      for (i=0; i<2*r->total_step; i++)
      {
        std::vector<OpenRAVE::dReal> vec;
        starttraj->GetWaypoint(i, vec, r->robot->GetActiveConfigurationSpecification());
        for (j=0; j<r->n_adof; j++)
          r->traj[i*r->n_adof+j] = vec[j];
      }
      // convert r->traj to init_values
      convertOpenRavePointerValues(dof, init_values, r->traj, opt_step);
    } else {
      RAVELOG_INFO("Initializing from a straight-line trajectory ...\n");
      init_values = initArmTrajStraightLine(start, goal, opt_step);
    }
    results = init_values;

    // Planner
    RAVELOG_INFO("Optimizing ...\n");
    results = BatchTrajOptimize3DArm(arm, this->gtsam_sdf, start, Vector::Zero(dof),
        goal, Vector::Zero(dof), init_values, opt_setting);

    // traj interpolation to total step to output
    Values output_traj_values = interpolateArmTraj(results, Qc_model, r->delta_t, r->output_inter); 

    double totalTime = static_cast<double>(total_timer.toc()) / 1e6;
    // total time in sec
    cout << "total time (s): " << totalTime <<endl;
    // timing ends from here

    // save timing to text file
    if (r->save_info) {
      std::ofstream myFile;
      myFile.open((std::string(getenv("HOME"))+"/Desktop/benchmark_data/time_gpmp2.txt").c_str(), std::ios::app);
      myFile<<totalTime;
      myFile<<std::endl;
      myFile.close();
    }

    // convert to r->traj
    convertValuesOpenRavePointer(dof, output_traj_values, r->traj, total_step, 
        joint_lower_limit, joint_upper_limit);

    /* save trajectory to csv file */
    #if SAVE_TRAJ_CSV
      RAVELOG_INFO("Saving trajectory to csv file.\n");
      std::ofstream myFile;
      myFile.open((std::string(getenv("HOME"))+"/Desktop/traj_gpmp2.csv").c_str());
      myFile<<"jp_type"<<std::endl;
      myFile<<std::setprecision(3)<<std::fixed;
      double dt = 5.0/(r->n_points-1);
      for (i=0; i<r->n_points; i++) {
        myFile<<dt*i<<",";
        for (j=0; j<DOF; j++) {
          if (j<DOF-1)
            myFile<<*(r->traj+i*DOF+j)<<",";
          else
            myFile<<*(r->traj+i*DOF+j)<<std::endl;
        }
      }
      myFile.close();
    #endif
  } // end GPMP2

  /* return pointer to run struct as string */
  {
    char buf[128];
    sprintf(buf, "%p", r);
    sout.write(buf, strlen(buf));
  }

  error:
    free(adofgoal);
    free(base_pose);
    free(dh_a);
    free(dh_alpha);
    free(dh_d);
    free(dh_theta);
    free(robot_idx);
    free(link_idx);

  if (exc)
  {
    run_destroy(r);
    throw OpenRAVE::openrave_exception(exc);
  }

  RAVELOG_INFO("Done! returning ...\n");
  return 0;
}

/**
 * get result trajectory and check for collision 
 */
int mod::gettraj(int argc, char * argv[], std::ostream& sout)
{
  int i;
  int nscan;
  /* args */
  struct run * r = 0;
  int no_collision_check = 0;
  int no_collision_exception = 0;
  int no_collision_details = 0;
  /* other */
  OpenRAVE::EnvironmentMutex::scoped_lock lockenv;
  OpenRAVE::TrajectoryBasePtr t;
  OpenRAVE::RobotBasePtr boostrobot;

  /* parse arguments */
  for (i=1; i<argc; i++)
  {
    if (strcmp(argv[i],"run")==0 && i+1<argc)
    {
      if (r) throw OpenRAVE::openrave_exception("Only one r can be passed!");
      nscan = sscanf(argv[++i], "%p", &r);
      if (nscan != 1) throw OpenRAVE::openrave_exception("Could not parse r!");
    }
    else if (strcmp(argv[i],"no_collision_check")==0)
      no_collision_check = 1;
    else if (strcmp(argv[i],"no_collision_exception")==0)
      no_collision_exception = 1;
    else if (strcmp(argv[i],"no_collision_details")==0)
      no_collision_details = 1;
    else break;
  }
  if (i<argc)
  {
    for (; i<argc; i++) RAVELOG_ERROR("argument %s not known!\n", argv[i]);
    throw OpenRAVE::openrave_exception("Bad arguments!");
  }

  if (!r) throw OpenRAVE::openrave_exception("you must pass a created run!");
  boostrobot = this->e->GetRobot(r->robot->GetName());

  lockenv = OpenRAVE::EnvironmentMutex::scoped_lock(this->e->GetMutex());

  /* create an openrave trajectory from the result, and send to sout */
  t = OpenRAVE::RaveCreateTrajectory(this->e);
  t->Init(r->robot->GetActiveConfigurationSpecification());
  for (i=0; i<r->n_points; i++)
  {
    std::vector<OpenRAVE::dReal> vec(&r->traj[i*r->n_adof], &r->traj[(i+1)*r->n_adof]);
    t->Insert(i, vec);
  }

  RAVELOG_INFO("timing trajectory ...\n");
  #if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0,7,0)
    /* new openrave added a fmaxaccelmult parameter (number 5) */
    OpenRAVE::planningutils::RetimeActiveDOFTrajectory(t,boostrobot,false,1.0,1.0,"LinearTrajectoryRetimer","");
  #else
    OpenRAVE::planningutils::RetimeActiveDOFTrajectory(t,boostrobot,false,1.0,"LinearTrajectoryRetimer");
  #endif

  if (!no_collision_check)
  {
    RAVELOG_INFO("checking trajectory for collision ...\n");
    int collides = 0;
    double time;
    OpenRAVE::CollisionReportPtr report(new OpenRAVE::CollisionReport());

    /* get trajectory length */
    int NN = t->GetNumWaypoints();
    int ii = 0;
    double total_dist = 0.0;
    for (ii=0; ii<NN-1; ii++) 
    {
      std::vector< OpenRAVE::dReal > point1;
      t->GetWaypoint(ii, point1);
      std::vector< OpenRAVE::dReal > point2;
      t->GetWaypoint(ii+1, point2);
      double dist = 0.0;
      int total_dof = boostrobot->GetActiveDOF();
      for (int jj=0; jj<total_dof; jj++)
      {
        dist += pow(point1[jj]-point2[jj],2);
      }
      total_dist += sqrt(dist);
    }

    double step_dist = 0.04;
    double step_time = t->GetDuration()*step_dist/total_dist;

    for (time=0.0; time<t->GetDuration(); time+=step_time)
    {
      std::vector< OpenRAVE::dReal > point;
      t->Sample(point, time);
      r->robot->SetActiveDOFValues(point);
      /* EnvironmentBase::CheckCollision should also check robot's grabbed bodies against the environment;
      * RobotBase::CheckSelfCollision should check robot's grabbed bodies as well
      * (as opposed to EnvironmentBase::CheckSelfCollision, see
      * http://openrave-users-list.185357.n3.nabble.com/Self-Collision-functions-td4026234.html) */
      if (this->e->CheckCollision(boostrobot,report)
      || boostrobot->CheckSelfCollision(report))
      {
        collides = 1;
        if (!no_collision_details) RAVELOG_ERROR("Collision: %s\n", report->__str__().c_str());
        if (!no_collision_exception) throw OpenRAVE::openrave_exception("Resulting trajectory is in collision!");
      }
    }
    if (collides)
      RAVELOG_ERROR("   trajectory collides!\n");
    if (r->save_info) {
      std::ofstream myFile;
      myFile.open((std::string(getenv("HOME"))+"/Desktop/benchmark_data/collision_gpmp2.txt").c_str(), std::ios::app);
      myFile<<collides;
      myFile<<std::endl;
      myFile.close();
    }
  }

  t->serialize(sout);
  return 0;
}

/**
 * destroy run
 */
int mod::destroy(int argc, char * argv[], std::ostream& sout)
{
  int i;
  int nscan;
  struct run * r = 0;
  /* parse arguments */
  for (i=1; i<argc; i++)
  {
    if (strcmp(argv[i],"run")==0 && i+1<argc)
    {
      if (r) throw OpenRAVE::openrave_exception("Only one run can be passed!");
      nscan = sscanf(argv[++i], "%p", &r);
      if (nscan != 1) throw OpenRAVE::openrave_exception("Could not parse r!");
    }
    else break;
  }
  if (i<argc)
  {
    for (; i<argc; i++) RAVELOG_ERROR("argument %s not known!\n", argv[i]);
    throw OpenRAVE::openrave_exception("Bad arguments!");
  }
  if (!r) throw OpenRAVE::openrave_exception("you must pass a created run!");
  run_destroy(r);
  return 0;
}

void run_destroy(struct run * r)
{
  free(r->traj);
  //free(r->rsdfs); 
  free(r->adofindices);
  free(r->spheres);
  free(r);
}

} /* namespace orgpmp2 */
