#pragma once

// set define if you want to extend the gui
#include <mars_interfaces/sim/MarsPluginTemplate.h>
#include <mars_interfaces/MARSDefs.h>
#include <osg_material_manager/OsgMaterialManager.hpp>
//#include <osg_terrain/Terrain.h>

//#include <osg/Group>
#include <string>
#include "ParticleSystem.hpp"

namespace mars
{

    namespace plugin
    {
        namespace particle_system
        {

            class ParticleSystemLib: public mars::interfaces::MarsPluginTemplate
            {
            public:
                ParticleSystemLib(lib_manager::LibManager *theManager);
                ~ParticleSystemLib();

                // LibInterface methods
                int getLibVersion() const
                    { return 1; }
                const std::string getLibName() const
                    { return std::string("mars_particle_system"); }
                CREATE_MODULE_INFO();

                // MarsPlugin methods
                void init();
                void reset();
                void update(mars::interfaces::sReal time_ms);

                // ParticleSystem methods

            private:
                std::vector<ParticleSystem*> systems;
                osg_material_manager::OsgMaterialManager *materialManager;
                unsigned long IdToFollow;
                std::string NodeToFollow;
                int pIndex;

            }; // end of class definition ParticleSystemLib

        } // end of namespace particle_system
    } // end of namespace plugin
} // end of namespace mars
