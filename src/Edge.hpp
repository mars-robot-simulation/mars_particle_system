#pragma once
#include "Edge.hpp"

namespace mars
{
    namespace plugin
    {
        namespace particle_system
        {

            class Node;
      
            class Edge
            {
            public:
                Edge(Node *fromNode, Node *toNode);
                ~Edge();

                void clearToNode();
                void clearFromNode();

            private:
                Node *fromNode, *toNode;

            }; // end of class definition Edge

        } // end of namespace particle_system
    } // end of namespace plugin
} // end of namespace mars
