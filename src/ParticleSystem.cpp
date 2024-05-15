#include "ParticleSystem.hpp"
//#include "Octree.hpp"

#include <mars_interfaces/sim/NodeManagerInterface.h>
#include <mars_interfaces/sim/SimulatorInterface.h>
#include <mars_interfaces/sim/CollisionInterface.hpp>
#include <mars_interfaces/sim/LoadCenter.h>
#include <mars_ode_collision/objects/Mesh.hpp>
#include <random>
#include "Content.hpp"

#include <osg/Texture2D>
#include <osgDB/WriteFile>
#include <mars_utils/misc.h>
#include <mars_utils/mathUtils.h>

namespace mars
{
    namespace plugin
    {
        namespace particle_system
        {

            using namespace mars::utils;
            using namespace mars::interfaces;

            // input have to be between 0 and 1
            void writeFloat(float value, unsigned char *buffer)
            {
                value *= 65535;
                double s256 = 1./256;
                int v = floor(value*s256);
                if(v < 0) v = 0;
                if(v>255) v = 255;
                buffer[0] = (unsigned char)v;
                v = (int)value % 256;
                if(v < 0) v = 0;
                if(v>255) v = 255;
                buffer[1] = (char)v;
            }

            // input can between 0 - and 65535
            void writeInt(int value, unsigned char *buffer)
            {
                double s256 = 1./256;
                int v = floor(value*s256);
                if(v < 0) v = 0;
                if(v>255) v = 255;
                buffer[0] = (unsigned char)v;
                v = (int)value % 256;
                if(v < 0) v = 0;
                if(v>255) v = 255;
                buffer[1] = (char)v;
            }

            double ParticleSystem::toShaderValue(double v)
            {
                double d = (double)(int)((0.5+(v/200.0))*imageMaxValue);
                //fprintf(stderr, "d: %g\n", d);
                return d*divImageMaxValue;
            }

            ParticleSystem::ParticleSystem(osg_material_manager::OsgMaterialManager *materialManager, interfaces::ControlCenter *control, int pIndex)
                : control(control), octree(256), pIndex(pIndex),
                  materialManager(materialManager), firstUpdate(true)
            {
            }

            ParticleSystem::~ParticleSystem()
            {
                for(auto& nodeInfo : nodes)
                {
                    auto& mesh = nodeInfo.nodeData.mesh;
                    constexpr bool free_memory = true;
                    mesh.setZero(free_memory);
                }
            }

            void ParticleSystem::init(configmaps::ConfigMap &map)
            {
                // load information from material
                material = materialManager->getOsgMaterial(map["materialName"]);
                if(material.valid())
                {
                    configmaps::ConfigMap info = material->getMaterialData();
                    std::string loadPath = ".";
                    if(info.hasKey("loadPath"))
                    {
                        loadPath << info["loadPath"];
                    }
                    if(info.hasKey("filePrefix"))
                    {
                        loadPath << info["filePrefix"];
                    }
                    double_precision = false;
                    if(info.hasKey("particle_16bit"))
                    {
                        double_precision = info["particle_16bit"];
                    }
                    particleTexture = material->getTexture(map["textureName"]);
                    numInstances = material->getNumInstances();
                    targetSize = material->getInstancesWidth();
                    targetSizeCamScale = 200 / targetSize;
                    // get texture resolution *Note* image->r() should be in bytes
                    osg::ref_ptr<osg::Image> i = particleTexture->getImage();
                    fprintf(stderr, "image resolution: %d\n", i->r());
                    if(i->r() == 1)
                    {
                        imageMaxValue = 255;
                    } else
                    {
                        imageMaxValue = 65535;
                    }
                    if(double_precision)
                    {
                        // overwrite imageMaxValue
                        imageMaxValue = 65535;
                    }
                    divImageMaxValue = 1.0/imageMaxValue;
                    particleTexture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
                    particleTexture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
                    long index = 0;
                    // 2. load positions into octree
                    double la = 0.0, lb = 0.0, lg = 0.0, ls = 0.0;
                    int seed = 0;
                    std::default_random_engine gen;
                    std::uniform_real_distribution<double> random;

                    if(map.hasKey("seed"))
                    {
                        seed = map["seed"];
                        Vector p;
                        bool first = 1;
                        double x, y;
                        la = lg = lg = ls = 1.0;
                        if(map.hasKey("limits"))
                        {
                            if(map["limits"].hasKey("alpha"))
                            {
                                la = map["limits"]["alpha"];
                            }
                            if(map["limits"].hasKey("beta"))
                            {
                                lb = map["limits"]["beta"];
                            }
                            if(map["limits"].hasKey("gamma"))
                            {
                                lg = map["limits"]["gamma"];
                            }
                            if(map["limits"].hasKey("scale"))
                            {
                                ls = map["limits"]["scale"];
                            }
                        }
                    }
                    gen.seed(seed);

                    if(map.hasKey("positionFile"))
                    {
                        std::string filename = map["positionFile"];
                        filename = pathJoin(loadPath, filename);
                        LOG_INFO("posFile: %s", filename.c_str());
                        if(pathExists(filename))
                        {
                            Vector p(0, 0, 0);
                            double x, y;
                            FILE *file = fopen(filename.c_str(), "r");
                            int read = fscanf(file, "%lf,%lf", &x, &y);
                            int i = 0;
                            while(read == 2)
                            {
                                Content *c = new Content(&octree, new BaseContent());
                                p.x() = x*targetSizeCamScale;
                                p.y() = y*targetSizeCamScale;
                                x = (((double)(int)((0.5+(p.x()/200.0))*imageMaxValue))*divImageMaxValue-0.5)*targetSize;
                                y = (((double)(int)((0.5+(p.y()/200.0))*imageMaxValue))*divImageMaxValue-0.5)*targetSize;
                                p.z() = getHeightFromScene(x, y);
                                c->setPosition(p);
                                c->id = i++%256;
                                c->index = ++index;

                                c->setRotation(random(gen)*la, random(gen)*lb,
                                               random(gen)*lg);
                                c->setScale(1-0.5*ls);
                                read = fscanf(file, "%lf,%lf", &x, &y);
                            }
                            fclose(file);
                        }
                    } else
                    {
                        int numRandomParticles = 1000;
                        if(map.hasKey("numRandomParticles"))
                        {
                            numRandomParticles = map["numRandomParticles"];
                        }
                        Vector p;
                        bool first = 0;
                        double x, y;
                        //FILE *f = fopen("p.txt", "w");
                        for(int i=0; i<numRandomParticles; ++i)
                        {
                            Content *c = new Content(&octree, new BaseContent());
                            if(first)
                            {
                                p.x() = p.y() = p.z() = 0.0;
                                first = false;
                                c->setRotation(random(gen)*la, random(gen)*lb,
                                               random(gen)*lg);
                                c->setScale(1-0.5*ls);
                            } else
                            {
                                x = random(gen)*200-100;
                                y = random(gen)*200-100;
                                p.x() = x;
                                p.y() = y;
                                x = (((double)(int)((0.5+(p.x()/200.0))*imageMaxValue))*divImageMaxValue-0.5)*targetSize;
                                y = (((double)(int)((0.5+(p.y()/200.0))*imageMaxValue))*divImageMaxValue-0.5)*targetSize;

                                p.z() = getHeightFromScene(x, y);
                                //p.z() = 0.0;//random(gen)*200-100;
                                //c->setRotation(random(gen), 0.0, 0.0);
                                c->setRotation(random(gen)*la, random(gen)*lb,
                                               random(gen)*lg);
                                c->setScale(1-0.5*ls+random(gen)*ls);
                            }
                            c->setPosition(p);
                            c->id = i%256;
                            c->index = ++index;
                            //fprintf(f, "%lf,%lf\n", x, y);
                            //fprintf(stderr, "fin: %g %g %g\n", p.x(), p.y(), p.z());
                        }
                        //fclose(f);
                    }
                    if(map.hasKey("physics"))
                    {
                        std::string type = "box";
                        if(map["physics"].hasKey("type"))
                        {
                            type << map["physics"]["type"];
                        }
                        interfaces::NodeData node;
                        node.map = map;
                        node.physicMode = interfaces::NodeData::typeFromString(type);
                        node.ext = Vector(1, 1, 1);
                        if(map["physics"].hasKey("size"))
                        {
                            node.ext.x() = map["physics"]["size"]["x"];
                            node.ext.y() = map["physics"]["size"]["y"];
                            node.ext.z() = map["physics"]["size"]["z"];
                        }
                        node.movable = false;
                        node.filename = "PRIMITIVE";
                        node.origName = "empty";
                        MaterialData coll_material;
                        coll_material.exists = false;
                        coll_material.transparency = 1;
                        node.material = coll_material;

                        if(type == "mesh" && map["physics"].hasKey("path") && map["physics"].hasKey("name"))
                        {
                            std::string path = map["physics"]["path"];
                            path = pathJoin(loadPath, path);
                            if(pathExists(path))
                            {
                                node.filename = path;
                                node.origName = map["physics"]["name"].toString();
                            } else
                            {
                                node.physicMode = NODE_TYPE_BOX;
                                std::cout << "Path '" << path << "' does not exist, cannot use physics type ’mesh’." << std::endl;
                            }
                        } else if(type == "mesh")
                        {
                            node.physicMode = NODE_TYPE_BOX;
                            std::cout << "Node 'physics' has no attribute 'path' and/or attribute 'name', cannot use physics type 'mesh'." << std::endl;
                        }

                        char name[55];
                        for(int i=0; i<numInstances; ++i)
                        {
                            sprintf(name, "p_%05d_%05d", pIndex, i);
                            node.name = name;
                            ConfigMap config;
                            node.toConfigMap(&config);
                            config["type"] = config["physicmode"];
                            //fprintf(stderr, "%s\n", config.toYamlString().c_str());
                            config["movable"] = false;
                            ode_collision::Object* collision = control->collision->createObject(config, NULL);
                            if(collision && type == "mesh")
                            {
                                control->loadCenter->loadMesh->getPhysicsFromMesh(&node);
                                ((ode_collision::Mesh*)collision)->setMeshData(node.mesh);
                                collision->createGeom();
                            }

                            NodeInfo nf;
                            nf.nodeData = node;
                            nf.originalExt = node.ext;
                            nf.index = 0;
                            nf.collision = collision;
                            nodes.push_back(nf);
                        }
                    }
                } else
                {
                    fprintf(stderr, "cannot find material: %s\n%s\n", map["materialName"].getString().c_str(), map.toYamlString().c_str());
                }
            }

            void ParticleSystem::reset()
            {
                firstUpdate = true;
            }

            // todo: handle the hight correctly by scene properties
            sReal ParticleSystem::getHeightFromScene(sReal x, sReal y)
            {
                CollisionInterface* collisionSpace = control->collision.get();
                const utils::Vector ray_origin(x, y, -100.0);
                const utils::Vector ray_vector(0.0, 0.0, 200);
                sReal value = -100.0 + collisionSpace->getVectorCollision(ray_origin, ray_vector);
                if (value>=99.9)
                    value = 0.0;

                return value;
            }

            void ParticleSystem::update(double *pose_)
            {
                size_t t=0;
                Vector new_pos(pose_[0], pose_[1], 0.0);
                Vector p = new_pos-pos;
                if(p.norm() < 6.0 and not firstUpdate) return;
                firstUpdate = false;
                fprintf(stderr, ".");
                pos = new_pos;
                double pose[2] = {pose_[0], pose_[1]};

                if(material.valid() && particleTexture.valid())
                {
                    Vector p;
                    unsigned x, y, z;

                    x = y = z = octree.halfWidth();
                    pose[0] *= targetSizeCamScale;
                    pose[1] *= targetSizeCamScale;
                    if(fabs(pose[0]) > x)
                    {
                        if(pose[0] < 0)
                        {
                            x = 0;
                        } else
                        {
                            x = octree.width();
                        }
                    } else
                    {
                        x += pose[0];
                    }
                    if(fabs(pose[1]) > y)
                    {
                        if(pose[1] < 0)
                        {
                            y = 0;
                        } else
                        {
                            y = octree.width();
                        }
                    } else
                    {
                        y += pose[1];
                    }

                    //fprintf(stderr, "update for %u %u %u %d\n", x, y, z, numFile);
                    std::vector<Octree<Node*>::Leaf*> leafs;
                    octree.findNearestNeighbours(x, y, z, numInstances, leafs);

                    osg::ref_ptr<osg::Image> i = particleTexture->getImage();
                    // todo: changing the pixel format does not always work have to check that
                    i->setPixelFormat(GL_RGBA);
                    unsigned char *data = i->data();
                    int dim = sqrt(numInstances);
                    int texDim = i->s();
                    dim = texDim;
                    float vx, vy, vz;
                    int pixel_offset = 0;
                    int pixel_pos;
                    sRotation r, r2;
                    double s;
                    //fprintf(stderr, "leafs size: %lu\n", leafs.size());
                    std::vector<Content*> toAdd;
                    for(size_t i=0; i<nodes.size(); ++i)
                    {
                        nodes[i].skip = false;
                    }
                    for(size_t i=0; i<leafs.size(); ++i)
                    {
                        Node *n = leafs[i]->value();
                        for(int l=0; l<n->numContents(); ++l)
                        {
                            Content *c = n->getContent(l);
                            p = c->getPosition();
                            c->getRotation(&r.alpha, &r.beta, &r.gamma);
                            r2 = r;
                            r2.alpha *= 360;
                            r2.beta *= 360;
                            r2.gamma *= 360;
                            s = c->getScale();
                            if(nodes.size() > t)
                            {
                                bool found = false;
                                for(size_t i=0; i<nodes.size(); ++i)
                                {
                                    if(nodes[i].index == c->index)
                                    {
                                        nodes[i].skip = true;
                                        //fprintf(stderr, "found nf %lu\n", c->index);
                                        found = true;
                                        break;
                                    }
                                }
                                if(!found)
                                {
                                    toAdd.push_back(c);
                                }
                            }
                            // todo: change resolution to 16 bit
                            //fprintf(stderr, "pos: %g %g\n", p.x(), p.y());
                            // invert y coordinate for osg texture handling
                            // convert to a value between 0-1
                            vx = (0.5+(p.x()/200.0));
                            vy = (0.5+(p.y()/200.0));
                            vz = (0.5+(p.z()/200.0));
                            pixel_pos = (pixel_offset/dim)*texDim*4+pixel_offset%dim*4;
                            if(double_precision)
                            {
                                writeFloat(vx, data+pixel_pos);
                                writeFloat(vy, data+pixel_pos+2);
                                writeInt(c->id, data+pixel_pos+4);
                                writeFloat(vz, data+pixel_pos+6);

                                // fprintf(stderr, "buffer: %d %d %d %d\n",
                                //         (unsigned char)data[pixel_pos],
                                //         (unsigned char)data[pixel_pos+1],
                                //         (unsigned char)data[pixel_pos+2],
                                //         (unsigned char)data[pixel_pos+3]);

                                writeFloat(r.alpha, data+pixel_pos+8);
                                writeFloat(r.beta, data+pixel_pos+10);
                                writeFloat(r.gamma, data+pixel_pos+12);
                                writeFloat(s*0.1, data+pixel_pos+14);

                                // fprintf(stderr, "buffer2: %d %d %d %d\n",
                                //         (unsigned char)data[pixel_pos+6],
                                //         (unsigned char)data[pixel_pos+7],
                                //         (unsigned char)data[pixel_pos+14],
                                //         (unsigned char)data[pixel_pos+15]);

                            } else
                            {
                                data[pixel_pos+0] = (unsigned char)(vx*imageMaxValue);
                                data[pixel_pos+1] = (unsigned char)(vy*imageMaxValue);
                                // we have a problem if c->id is bigger then imageMaxValue
                                data[pixel_pos+2] = (unsigned char)c->id;
                                data[pixel_pos+3] = imageMaxValue;
                            }
                            ++t;
                            if(double_precision)
                            {
                                pixel_offset += 4;
                            } else
                            {
                                pixel_offset += 1;
                            }
                            if(t==numInstances) break;
                        }
                        if(t==numInstances) break;
                    }
                    for(int i=t; i<numInstances; ++i)
                    {
                        pixel_pos = (pixel_offset/dim)*texDim*4+pixel_offset%dim*4;
                        if(double_precision)
                        {
                            for(int k=0; k<7; ++k)
                            {
                                data[pixel_pos+k] = 0;
                            }
                            data[pixel_pos+3] = 255;
                            data[pixel_pos+7] = 255;
                            pixel_offset += 4;
                        } else
                        {
                            for(int k=0; k<3; ++k)
                            {
                                data[pixel_pos+k] = 0;
                            }
                            data[pixel_pos+3] = 255;
                            pixel_offset += 1;
                        }
                        if(nodes.size() > i)
                        {
                            for(size_t k=0; k<nodes.size(); ++k)
                            {
                                if(nodes[i].skip == false)
                                {
                                    // todo: fix this (orientation and scale)
                                    LOG_ERROR("Skip particle may not work correctly");
                                    nodes[i].skip = true;
                                    nodes[i].nodeData.pos.x() = 0.0;
                                    nodes[i].nodeData.pos.y() = 0.0;
                                    nodes[i].index = 0;
                                    nodes[i].collision->setPosition(p);
                                    // todo: updateTransform is currently automatically called
                                    //nodes[i].collision->updateTransform();
                                    break;
                                }
                            }
                        }
                    }
                    for(auto c: toAdd)
                    {
                        p = c->getPosition();
                        c->getRotation(&r.alpha, &r.beta, &r.gamma);
                        r.alpha *= 360;
                        r.beta *= 360;
                        r.gamma *= 360;
                        s = c->getScale();
                        for(size_t i=0; i<nodes.size(); ++i)
                        {
                            NodeInfo &nf = nodes[i];
                            if(nf.skip == false)
                            {
                                // either nf_ptr is NULL or points to the next free
                                nf.nodeData.pos.x() = (toShaderValue(p.x())-0.5)*targetSize;
                                nf.nodeData.pos.y() = (toShaderValue(p.y())-0.5)*targetSize;
                                nf.nodeData.pos.z() = (toShaderValue(p.z())-0.5)*200.0;
                                nf.nodeData.rot = eulerToQuaternion(r);
                                s = toShaderValue((s*0.1-0.5)*200)*10;
                                nf.nodeData.ext = nf.originalExt*s;
                                //fprintf(stderr, "%lu %g %g\n", c->index, nf.nodeData.pos.x(), nf.nodeData.pos.y());
                                nf.collision->setSize(nf.nodeData.ext);
                                nf.collision->setPosition(nf.nodeData.pos);
                                nf.collision->setRotation(nf.nodeData.rot);
                                // todo: updateTransform is currently automatically called
                                //nf.collision->updateTransform();
                                nf.skip = true;
                                nf.index = c->index;
                                //simNode = control->nodes->getSimNode(nodes[t].index);
                                //((sim::NodeManager*)control->nodes)->changeNode(simNode, &nodes[t]);
                                break;
                            }
                        }
                    }
                    //fprintf(stderr, "numInstances: %zu/%d\n", t, numInstances);
                    i->dirty();

                    // char filename[55];
                    // sprintf(filename, "test_%05d.png", ++numFile);
                    // osgDB::writeImageFile(*(i.get()), filename);

                }
            }

        } // end of namespace particle_system
    } // end of namespace plugin
} // end of namespace mars
