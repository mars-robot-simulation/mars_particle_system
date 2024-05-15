#include "Octree.hpp"
#include "Content.hpp"
#include "Node.hpp"

namespace mars
{
    namespace plugin
    {
        namespace particle_system
        {

            Content::Content(Octree<Node*> *octree, BaseContent *data) : octree(octree),
                                                                         data(data),
                                                                         node(NULL),
                                                                         alpha(0.0),
                                                                         beta(0.0),
                                                                         gamma(0.0),
                                                                         scale(1.0)
            {
            }

            Content::~Content()
            {
                if(node)
                {
                    node->removeContent(this);
                    // todo: add remove node to octree
                }
            }

            void Content::moveToNode(Node* n)
            {
                if(n == node)
                {
                    return;
                }
                if(node)
                {
                    node->removeContent(this);
                    // todo: add remove node to octree
                }
                node = n;
                node->addContent(this);
            }

            unsigned Content::bound(int v)
            {
                if(v<0) return 0;
                if(v >= octree->width()) return octree->width()-1;
                return v;
            }

            void Content::setPosition(const utils::Vector &p)
            {
                pos = p;
                x = bound(floor(pos.x()) + octree->halfWidth());
                y = bound(floor(pos.y()) + octree->halfWidth());
                z = bound(floor(pos.z()) + octree->halfWidth());
                Octree<Node*>::Leaf *leaf = octree->at(x, y, z);
                if(!leaf)
                {
                    if(node)
                    {
                        node->removeContent(this);
                        // todo: add remove node to octree
                        // if(node->isEmpty()) {}
                        delete node;
                    }
                    node = new Node();
                    node->addContent(this);
                    octree->insert(x, y, z, node);
                } else
                {
                    moveToNode(leaf->value());
                }
            }

            void Content::setRotation(double alpha, double beta, double gamma)
            {
                this->alpha = alpha;
                this->beta = beta;
                this->gamma = gamma;
            }

            void Content::getRotation(double *alpha, double *beta, double *gamma)
            {
                *alpha = this->alpha;
                *beta = this->beta;
                *gamma = this->gamma;
            }

            void Content::setScale(double scale)
            {
                this->scale = scale;
            }

        } // end of namespace particle_system
    } // end of namespace plugin
} // end of namespace mars
