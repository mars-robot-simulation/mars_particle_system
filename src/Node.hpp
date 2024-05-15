#pragma once

#include <mars_utils/Vector.h>
#include <vector>

namespace mars
{
    namespace plugin
    {
        namespace particle_system
        {

            class Edge;
            class Content;

            class Node
            {
            public:
                Node();
                ~Node();

                void addOutgoingEdge(Edge* e);
                void addIncomingEdge(Edge* e);
                void removeOutgoingEdge(Edge* e);
                void removeIncomingEdge(Edge* e);

                void addContent(Content *c);
                void removeContent(Content *c);
                int numContents();

                bool isEmpty() {return (content.size()==0);}
                Content* getContent(size_t i) {return content[i];}

            private:
                std::vector<Edge*> outgoing, incoming;
                std::vector<Content*> content;

            }; // end of class definition Node

        } // end of namespace particle_system
    } // end of namespace plugin
} // end of namespace mars
