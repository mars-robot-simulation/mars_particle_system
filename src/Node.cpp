#include "Node.hpp"
#include "Edge.hpp"

namespace mars
{
    namespace plugin
    {
        namespace particle_system
        {
      
            Node::Node()
            {
            }

            Node::~Node()
            {
                // todo: here we have a problem if there are still eges connected
                //   - maybe change node in Edge to NULL
                for(auto e: incoming)
                {
                    e->clearToNode();
                }
                for(auto e: outgoing)
                {
                    e->clearFromNode();
                }
            }

            void Node::addOutgoingEdge(Edge* e)
            {
                outgoing.push_back(e);
            }

            void Node::addIncomingEdge(Edge* e)
            {
                incoming.push_back(e);
            }

            void Node::removeOutgoingEdge(Edge* e)
            {
                for(auto it=outgoing.begin(); it!=outgoing.end(); ++it)
                {
                    if(*it == e)
                    {
                        outgoing.erase(it);
                        break;
                    }
                }
            }

            void Node::removeIncomingEdge(Edge* e)
            {
                for(auto it=incoming.begin(); it!=incoming.end(); ++it)
                {
                    if(*it == e)
                    {
                        incoming.erase(it);
                        break;
                    }
                }
            }

            void Node::addContent(Content *c)
            {
                for(auto it: content)
                {
                    if(it == c)
                    {
                        return;
                    }
                }
                content.push_back(c);
            }

            void Node::removeContent(Content *c)
            {
                for(auto it=content.begin(); it!=content.end(); ++it)
                {
                    if(*it == c)
                    {
                        content.erase(it);
                        break;
                    }
                }
            }

            int Node::numContents()
            {
                return (int)content.size();
            }

        } // end of namespace particle_system
    } // end of namespace plugin
} // end of namespace mars
