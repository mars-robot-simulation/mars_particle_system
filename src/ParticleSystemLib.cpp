
#include "ParticleSystemLib.hpp"
//#include "Octree.hpp"

#include <mars_interfaces/graphics/GraphicsManagerInterface.h>
#include <mars_interfaces/sim/NodeManagerInterface.h>
#include <mars_utils/misc.h>

#include <osg/Texture2D>
#include <osgDB/WriteFile>

namespace mars
{
    namespace plugin
    {
        namespace particle_system
        {

            using namespace mars::utils;
            using namespace mars::interfaces;

            ParticleSystemLib::ParticleSystemLib(lib_manager::LibManager *theManager)
                : MarsPluginTemplate(theManager, "particle_system"),
                  materialManager(NULL), IdToFollow(0), pIndex(0)
            {
            }

            void ParticleSystemLib::init()
            {

                //if(!control->graphics) return;
                // scene = static_cast<osg::Group*>(control->graphics->getScene2());

                materialManager = libManager->getLibraryAs<osg_material_manager::OsgMaterialManager>("osg_material_manager", true);
                // terrain = new osg_terrain::Terrain(materialManager);
                // scene->addChild(terrain.get());
                //control->sim->switchPluginUpdateMode(PLUGIN_GUI_MODE, this);

                // 1. load instances: is done by loading scene
                std::string configFile;
                configmaps::ConfigMap map;
                try
                {
                    // get config path
                    std::string configPath = control->cfg->getOrCreateProperty("Config", "config_path", ".").sValue;
                    std::string terrainPath = "";
                    control->cfg->getPropertyValue("Scene", "terrain_path", "value",
                                                   &terrainPath);
                    if(terrainPath != "")
                    {
                        char *envText = getenv("AUTOPROJ_CURRENT_ROOT");
                        terrainPath = replaceString(terrainPath, "$AUTOPROJ_CURRENT_ROOT", envText);
                        configPath = terrainPath;
                    }

                    configFile = utils::pathJoin(configPath, "particle_system.yml");
                    LOG_INFO("load %s", configFile.c_str());
                    map = configmaps::ConfigMap::fromYamlFile(configFile);
                } catch (...)
                {
                    LOG_ERROR("no particle_system.yml found but particle_system plugin loaded! (%s)", configFile.c_str());
                }
                // 2. load positions into octree
                if(map.hasKey("particles"))
                {
                    for(auto& it: map["particles"])
                    {
                        ParticleSystem *ps = new ParticleSystem(materialManager, control, pIndex++);
                        ps->init(it);
                        systems.push_back(ps);
                    }
                }
                NodeToFollow = "";
                if (map.hasKey("followNode"))
                    NodeToFollow << map["followNode"];
                ParticleSystemLib::reset();
            }

            void ParticleSystemLib::reset()
            {
                for(auto it: systems)
                {
                    it->reset();
                }
            }

            ParticleSystemLib::~ParticleSystemLib()
            {
                for(auto it: systems)
                {
                    delete it;
                }
                if(materialManager) libManager->releaseLibrary("osg_material_manager");
            }


            void ParticleSystemLib::update(sReal time_ms)
            {
                double camPose[7] = {0, 0, 0, 0, 0, 0, 0};
                GraphicsWindowInterface* gw = control->graphics->get3DWindow(1);
                NodeManagerInterface* nm = control->nodes;

                if(gw)
                {
                    gw->getCameraInterface()->getViewportQuat(camPose, camPose+1, camPose+2, camPose+3, camPose+4, camPose+5, camPose+6);
                }
                for(auto &it: systems)
                {
                    it->update(camPose);
                }
            }

        } // end of namespace particle_system
    } // end of namespace plugin
} // end of namespace mars

DESTROY_LIB(mars::plugin::particle_system::ParticleSystemLib);
CREATE_LIB(mars::plugin::particle_system::ParticleSystemLib);
