#pragma once

#include "Octree.hpp"
#include <mars_utils/Vector.h>

namespace mars
{
    namespace plugin
    {
        namespace particle_system
        {

            class Node;

            class BaseContent {};

            class Content
            {
            public:
                Content(Octree<Node*> *octree, BaseContent *data);
                ~Content();

                void moveToNode(Node *n);
                void setPosition(const utils::Vector &p);
                utils::Vector getPosition() const {return pos;}
                void setRotation(double alpha, double beta, double gamma);
                void getRotation(double *alpha, double *beta, double *gamma);
                void setScale(double scale);
                double getScale() {return scale;}

                BaseContent *getData() {return data;}

                int id;
                unsigned long index;

            private:
                Octree<Node*> *octree;
                BaseContent *data;
                Node* node;
                utils::Vector pos;
                double alpha, beta, gamma, scale;
                unsigned x, y, z;
                unsigned bound(int v);
            };

        } // end of namespace particle_system
    } // end of namespace plugin
} // end of namespace mars
