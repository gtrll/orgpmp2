/** 
 * \file orgpmp2.cpp
 * \brief orgpmp2 entry point from OpenRAVE.
 */

#include <cstdio>
#include <cstring>
#include <boost/bind.hpp>

#include <openrave/config.h>
#include <openrave/openrave.h>
#include <openrave/plugin.h>

#include "orgpmp2_kdata.h"
#include "orgpmp2_mod.h"

/* globals: we maintain a single kdata xml reader instance */
namespace
{
   
static boost::shared_ptr<void> reg_reader;

static OpenRAVE::BaseXMLReaderPtr rdata_parser_maker(OpenRAVE::InterfaceBasePtr ptr, const OpenRAVE::AttributesList& atts)
{
   return OpenRAVE::BaseXMLReaderPtr(new orgpmp2::kdata_parser(boost::shared_ptr<orgpmp2::kdata>(),atts));
}

} /* anonymous namespace */

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO& info)
{ 
   /* create the kdata xml parser interface if it does noet yet exit */
   if(!reg_reader)
      reg_reader = OpenRAVE::RaveRegisterXMLReader(OpenRAVE::PT_KinBody,"orgpmp2",rdata_parser_maker);
   
   info.interfacenames[OpenRAVE::PT_Module].push_back("orgpmp2");
   
   return;
}

OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
{
   if((type == OpenRAVE::PT_Module)&&(interfacename == "orgpmp2"))
      return OpenRAVE::InterfaceBasePtr(new orgpmp2::mod(penv));
      
   return OpenRAVE::InterfaceBasePtr();
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
   RAVELOG_INFO("destroying plugin\n");
   return;
}