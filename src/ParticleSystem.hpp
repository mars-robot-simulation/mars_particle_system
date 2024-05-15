#pragma once

// set define if you want to extend the gui
#include <mars_interfaces/MARSDefs.h>
#include <mars_interfaces/NodeData.h>
#include <mars_interfaces/sim/ControlCenter.h>
#include <osg_material_manager/OsgMaterialManager.hpp>
//#include <osg_terrain/Terrain.h>

//#include <osg/Group>
#include <string>

#include "Octree.hpp"
#include "Node.hpp"
#include <mars_ode_collision/objects/Object.hpp>

namespace mars
{
    namespace plugin
    {
        namespace particle_system
        {

            struct NodeInfo
            {
                interfaces::NodeData nodeData;
                utils::Vector originalExt;
                unsigned long index;
                bool skip;
                ode_collision::Object *collision;
            };

            class ParticleSystem
            {
            public:
                ParticleSystem(osg_material_manager::OsgMaterialManager *materialManager, interfaces::ControlCenter *control, int pIndex);
                ~ParticleSystem();

                void init(configmaps::ConfigMap &map);
                void reset();
                void update(double *pose);

                // ParticleSystem methods

            private:
                interfaces::ControlCenter *control;
                Octree<Node*> octree;
                osg_material_manager::OsgMaterialManager *materialManager;
                osg::ref_ptr<osg_material_manager::OsgMaterial> material;
                osg::ref_ptr<osg::Texture2D> particleTexture;
                std::vector<NodeInfo> nodes;
                utils::Vector pos;
                int numInstances;
                double targetSize, targetSizeCamScale;
                bool double_precision;
                int imageMaxValue;
                double divImageMaxValue;
                bool firstUpdate;
                int pIndex;

                interfaces::sReal getHeightFromScene(interfaces::sReal x,
                                                     interfaces::sReal y);
                double toShaderValue(double v);
            }; // end of class definition ParticleSystem

        } // end of namespace particle_system
    } // end of namespace plugin
} // end of namespace mars
