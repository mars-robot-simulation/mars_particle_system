#include "Node.hpp"
#include "Edge.hpp"

namespace mars
{
    namespace plugin
    {
        namespace particle_system
        {
      
            Edge::Edge(Node *fromNode, Node *toNode) : fromNode(fromNode), toNode(toNode)
            {
                fromNode->addOutgoingEdge(this);
                toNode->addIncomingEdge(this);
            }

            Edge::~Edge()
            {
                fromNode->removeOutgoingEdge(this);
                toNode->removeIncomingEdge(this);
            }

            void Edge::clearToNode()
            {
                toNode = NULL;
            }

            void Edge::clearFromNode()
            {
                fromNode = NULL;
            }

        } // end of namespace particle_system
    } // end of namespace plugin
} // end of namespace mars
