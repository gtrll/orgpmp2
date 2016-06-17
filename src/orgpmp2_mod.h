/** \file orgpmp2_mod.h
 * \brief Interface to the orgpmp2 module, an implementation of gpmp2
 *        with gtsam.
 */

/* requires:
 *  - openrave/openrave.h
 * */

#include <gpmp2/obstacle/SignedDistanceField.h>

#include "orcwrap.h"

using namespace gtsam;

namespace orgpmp2
{

struct sdf;

/* the module itself */
class mod : public OpenRAVE::ModuleBase
{
public:
   OpenRAVE::EnvironmentBasePtr e; /* filled on module creation */
   int n_sdfs;
   struct sdf * sdfs;
   gpmp2::SignedDistanceField gtsam_sdf;
   
   int viewspheres(int argc, char * argv[], std::ostream& sout);
   int computedistancefield(int argc, char * argv[], std::ostream& sout);
   int create(int argc, char * argv[], std::ostream& sout);
   int gettraj(int argc, char * argv[], std::ostream& sout);
   int destroy(int argc, char * argv[], std::ostream& sout);

   mod(OpenRAVE::EnvironmentBasePtr penv) : OpenRAVE::ModuleBase(penv)
   {
      __description = "orgpmp2: openrave wrapper for gpmp2";
      RegisterCommand("viewspheres",orcwrap(boost::bind(&mod::viewspheres,this,_1,_2,_3)),"view spheres");
      RegisterCommand("computedistancefield",orcwrap(boost::bind(&mod::computedistancefield,this,_1,_2,_3)),"compute distance field");
      RegisterCommand("create",orcwrap(boost::bind(&mod::create,this,_1,_2,_3)),"create a gpmp2 run");
      RegisterCommand("gettraj",orcwrap(boost::bind(&mod::gettraj,this,_1,_2,_3)),"get trajectory of gpmp2 run");
      RegisterCommand("destroy",orcwrap(boost::bind(&mod::destroy,this,_1,_2,_3)),"destroy gpmp2 run");
      
      this->e = penv;
      this->n_sdfs = 0;
      this->sdfs = 0;
   }
   virtual ~mod() {}
   void Destroy() { RAVELOG_INFO("module unloaded from environment\n"); }
   /* This is called on e.LoadProblem(m, 'command') */
   int main(const std::string& cmd) { RAVELOG_INFO("module init cmd: %s\n", cmd.c_str()); return 0; }
};

void run_destroy(struct run * r);

} /* namespace orgpmp2 */