/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <string>
#include <cmath>
#include <algorithm>
#include <functional>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <cer_mobile_kinematics/mobile_arm.h>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include <R1Model.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace cer::kinematics;
using namespace octomap;

#define CTRL_DEG2RAD (M_PI/180.0)
#define CTRL_RAD2DEG (180.0/M_PI)

//typedef OcTreeDataNode<Vector> ManipOcTreeDataNode;
typedef ColorOcTreeNode ManipOcTreeDataNode;
typedef ColorOcTree ManipulabilityOctTree;

typedef ColorOcTreeNode RotationOcTreeDataNode;
typedef ColorOcTree RotationOctTree;

// forward declaration for "friend"
class VectorOcTree;

#define ROT_MAP_RES (0.1)

/****************************************************************/
void heatMap(double min, double max, double value, uint8_t &r, uint8_t &g, uint8_t &b)
{
    value = std::max(min, std::min(max, value));

    double a = 2 * (value - min) / (max - min);
    r = std::max(0.0, 255*(a - 1));
    b = std::max(0.0, 255*(1 - a));
    g = 255 - b - r;
}

// Vector OcTree node definition
class VectorOcTreeNode : public OcTreeNode
{
public:
    friend class VectorOcTree; // needs access to node children (inherited)

public:

    VectorOcTreeNode() : OcTreeNode() {}

    VectorOcTreeNode(const VectorOcTreeNode& rhs) : OcTreeNode(rhs), vec(rhs.vec) {}

    bool operator==(const VectorOcTreeNode& rhs) const
    {
        return (rhs.value == value && rhs.vec == vec);
    }

    void copyData(const VectorOcTreeNode& from)
    {
        OcTreeNode::copyData(from);
        this->vec =  from.getVector();
    }

    inline Vector getVector() const
    {
        return vec;
    }
    inline void  setVector(const Vector &v)
    {
        this->vec = v;
    }

    Vector& getVector() { return vec; }

    inline bool isVectorSet() const
    {
        return (vec.size() != 0);
    }

    void updateVectorChildren()
    {
        vec = getAverageChildVector();
    }

    Vector getAverageChildVector() const
    {
        Vector m(vec.size());
        int c = 0;

        if (children != NULL)
        {
            for (int i=0; i<8; i++)
            {
                VectorOcTreeNode* child = static_cast<VectorOcTreeNode*>(children[i]);

                if (child != nullptr && child->isVectorSet())
                {
                    m += child->getVector();
                    c++;
                }
            }
        }

        if (c > 0)
        {
            m /= c;

            return m;
        }
        else
        { // no child had a color other than white
          return Vector();
        }
    }

    // file I/O
    std::istream& readData(std::istream &s)
    {
        s.read((char*) &value, sizeof(value)); // occupancy

        size_t size;
        s.read((char*) &size, sizeof(size_t)); // vector
        vec.resize(size);
        for(size_t i=0; i<size; i++)
        {
            s.read((char*) &(vec[i]), sizeof(double));
        }

        return s;
    }
    std::ostream& writeData(std::ostream &s) const
    {
       s.write((const char*) &value, sizeof(value)); // occupancy

       size_t size = vec.size();
       s.write((const char*) &size, sizeof(size_t)); // vector
       for(size_t i=0; i<vec.size(); i++)
       {
           s.write((const char*) &(vec[i]), sizeof(double));
       }
       return s;
    }

protected:
    Vector vec;
};

class VectorOcTree: public OccupancyOcTreeBase<VectorOcTreeNode>
{
public:
  /// Default constructor, sets resolution of leafs
  VectorOcTree(double resolution): OccupancyOcTreeBase<VectorOcTreeNode>(resolution)
  {
      vectorOcTreeMemberInit.ensureLinking();
  }

  /// virtual constructor: creates a new object of same type
  /// (Covariant return type requires an up-to-date compiler)
  VectorOcTree* create() const
  {
      return new VectorOcTree(resolution);
  }

  std::string getTreeType() const
  {
      return "VectorOcTree";
  }

   /**
   * Prunes a node when it is collapsible. This overloaded
   * version only considers the node occupancy for pruning,
   * different colors of child nodes are ignored.
   * @return true if pruning was successful
   */
  virtual bool pruneNode(VectorOcTreeNode* node)
  {
      if (!isNodeCollapsible(node))
          return false;

      // set value to children's values (all assumed equal)
      node->copyData(*(getNodeChild(node, 0)));

      if (node->isVectorSet()) // TODO check
        node->setVector(node->getAverageChildVector());

      // delete children
      for (unsigned int i=0;i<8;i++)
      {
        deleteNodeChild(node, i);
      }
      delete[] node->children;
      node->children = NULL;

      return true;
    }

  virtual bool isNodeCollapsible(const VectorOcTreeNode* node) const
  {
      // all children must exist, must not have children of
      // their own and have the same occupancy probability
      if (!nodeChildExists(node, 0))
        return false;

      const VectorOcTreeNode* firstChild = getNodeChild(node, 0);
      if (nodeHasChildren(firstChild))
        return false;

      for (unsigned int i = 1; i<8; i++) {
        // compare nodes only using their occupancy, ignoring color for pruning
        if (!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) || !(getNodeChild(node, i)->getValue() == firstChild->getValue()))
          return false;
      }

      return true;
  }

  // set node color at given key or coordinate. Replaces previous color.
  VectorOcTreeNode* setNodeVector(const OcTreeKey& key, const Vector &v)
  {
      VectorOcTreeNode* n = search (key);
      if (n != 0)
      {
          n->setVector(v);
      }
      return n;
  }

  VectorOcTreeNode* setNodeVector(float x, float y,  float z, const Vector &v)
  {
        OcTreeKey key;
        if (!this->coordToKeyChecked(point3d(x,y,z), key))
        {
            return nullptr;
        }
        return setNodeVector(key,v);
  }

  // integrate color measurement at given key or coordinate. Average with previous color
  VectorOcTreeNode* averageNodeVector(const OcTreeKey& key, const Vector &v)
  {
     VectorOcTreeNode* n = search(key);
     if (n != 0)
     {
         if (n->isVectorSet())
         {
            Vector prev_vector = n->getVector();
            n->setVector((prev_vector + v)/2);
         }
         else
         {
            n->setVector(v);
         }
     }
     return n;
  }

  VectorOcTreeNode* averageNodeVector(float x, float y, float z, const Vector &v)
  {
        OcTreeKey key;
        if (!this->coordToKeyChecked(point3d(x,y,z), key))
        {
            return nullptr;
        }
        return averageNodeVector(key,v);
  }

  // integrate color measurement at given key or coordinate. Average with previous color
  VectorOcTreeNode* integrateNodeVector(const OcTreeKey& key, const Vector &v)
  {
    VectorOcTreeNode* n = search (key);
    if (n != 0)
    {
        if (n->isVectorSet())
        {
            Vector prev_vector = n->getVector();
            double node_prob = n->getOccupancy();
            Vector new_vector = prev_vector * node_prob +  v * (0.99-node_prob);
            n->setVector(new_vector);
        }
        else
        {
            n->setVector(v);
        }
    }
    return n;
  }

  VectorOcTreeNode* integrateNodeVector(float x, float y, float z, const Vector &v)
  {
        OcTreeKey key;
        if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
        return integrateNodeVector(key,v);
  }

  // update inner nodes, sets color to average child color
  void updateInnerOccupancy()
  {
      this->updateInnerOccupancyRecurs(this->root, 0);
  }

protected:
  void updateInnerOccupancyRecurs(VectorOcTreeNode* node, unsigned int depth)
  {
     // only recurse and update for inner nodes:
     if (nodeHasChildren(node))
     {
       // return early for last level:
       if (depth < this->tree_depth)
       {
         for (unsigned int i=0; i<8; i++)
         {
           if (nodeChildExists(node, i))
           {
             updateInnerOccupancyRecurs(getNodeChild(node, i), depth+1);
           }
         }
       }
       node->updateOccupancyChildren();
       node->updateVectorChildren();
     }
   }

  /**
   * Static member object which ensures that this OcTree's prototype
   * ends up in the classIDMapping only once. You need this as a
   * static member in any derived octree class in order to read .ot
   * files through the AbstractOcTree factory. You should also call
   * ensureLinking() once from the constructor.
   */
  class StaticMemberInitializer
  {
    public:
        StaticMemberInitializer()
        {
            VectorOcTree* tree = new VectorOcTree(0.1);
            tree->clearKeyRays();
            AbstractOcTree::registerTreeType(tree);
        }

       /**
       * Dummy function to ensure that MSVC does not drop the
       * StaticMemberInitializer, causing this tree failing to register.
       * Needs to be called from the constructor of this octree.
       */
       void ensureLinking() {};
  };
  /// static member to ensure static initialization (only once)
  static StaticMemberInitializer vectorOcTreeMemberInit;
};

std::ostream& operator<<(std::ostream& out, Vector const& v)
{
    out << '(';
    for(size_t i=0; i<v.size()-1; i++)
    {
        out << v[i] << ' ';
    }
    out << v[v.size()-1] << ')';
    return out;
}

VectorOcTree::StaticMemberInitializer VectorOcTree::vectorOcTreeMemberInit;

// Reachability 6D OcTree
class Reach6DOcTreeNode : public OcTreeNode
{
public:
    friend class Reach6DOcTree; // needs access to node children (inherited)

public:

    Reach6DOcTreeNode() : OcTreeNode(), rotMap(ROT_MAP_RES) {}

    Reach6DOcTreeNode(double resolution) : OcTreeNode(), rotMap(resolution) {}

    Reach6DOcTreeNode(const Reach6DOcTreeNode& rhs) : OcTreeNode(rhs), rotMap(rhs.rotMap) {}

    bool operator==(const Reach6DOcTreeNode& rhs) const
    {
        return (rhs.value == value && rhs.rotMap == rotMap);
    }

    void copyData(const Reach6DOcTreeNode& from)
    {
        OcTreeNode::copyData(from);
        OcTree map(from.getRotMap());
        rotMap.swapContent(map);
    }

    inline const OcTree& getRotMap() const
    {
        return rotMap;
    }

    OcTree& getRotMap()
    {
        return rotMap;
    }

    inline void setRotMap(const OcTree &map)
    {
        OcTree map_(map);
        rotMap.swapContent(map_);
    }

    inline bool isRotMapSet() const
    {
        return (rotMap.size()>0);
    }

    void updateRotMapChildren()
    {
        setRotMap(getAverageChildRotMap());
    }

    OcTree getAverageChildRotMap() const
    {
        OcTree m(rotMap.getResolution());

        if (children != NULL)
        {
            for (int i=0; i<8; i++)
            {
                Reach6DOcTreeNode* child = static_cast<Reach6DOcTreeNode*>(children[i]);

                if (child != nullptr && child->isRotMapSet())
                {
                    for(auto it=child->getRotMap().begin_leafs(), end=child->getRotMap().end_leafs(); it!=end; it++)
                    {
                        m.updateNode(it.getX(), it.getY(), it.getZ(), it->getValue(), true);
                    }
                }
            }

            if(m.getRoot())
            {
                m.updateInnerOccupancy();
            }
        }

        return m;
    }

    // file I/O
    std::istream& readData(std::istream &s)
    {
        rotMap.clear();

        if(!s.good())
        {
            return s;
        }

        s.read((char*)&value, sizeof(value));
        std::string id;
        size_t idSize;
        s.read((char*)&idSize, sizeof(size_t));
        id.resize(idSize);
        s.read((char*)id.c_str(), idSize*sizeof(char));
        size_t treeSize;
        s.read((char*)&treeSize, sizeof(size_t));
        double res;
        s.read((char*)&res, sizeof(double));

        if(id!=rotMap.getTreeType())
        {
            yError() << "Invalid ColorOcTree type";
            s.setstate(ios_base::badbit);
            return s;
        }

        rotMap.setResolution(res);

        if (treeSize > 0)
          rotMap.readBinaryData(s);

        return s;
    }

    std::ostream& writeData(std::ostream &s) const
    {
       s.write((const char*)&value, sizeof(value));
       std::string id = rotMap.getTreeType();
       size_t idSize = id.size();
       s.write((const char*)&idSize, sizeof(size_t));
       s.write((const char*)id.c_str(), idSize*sizeof(char));
       size_t treeSize = rotMap.size();
       s.write((const char*)&treeSize, sizeof(size_t));
       double res = rotMap.getResolution();
       s.write((const char*)&res, sizeof(double));

       if(rotMap.size() > 0)
          rotMap.writeBinaryData(s);

       return s;
    }

    ColorOcTree* ConvertNodeToColorOcTree()
    {
        ColorOcTree *m = new ColorOcTree(rotMap.getResolution());

        int max=0;
        for(auto it=rotMap.begin_leafs(), end=rotMap.end_leafs(); it!=end; it++)
        {
            ColorOcTreeNode* node = m->updateNode(it.getX(), it.getY(), it.getZ(), true, true);
            int v = it->getValue();
            node->setValue(v);
            if(v > max)
                max = v;
        }

        for(auto it=m->begin_leafs(), end=m->end_leafs(); it!=end; it++)
        {
            uint8_t r,g,b;
            heatMap(0,max, it->getValue(), r,g,b);
            it->setColor(r,g,b);
        }

        if(m->getRoot())
        {
            m->updateInnerOccupancy();
        }

        return m;
    }

protected:
    OcTree rotMap;
};


class Reach6DOcTree: public OccupancyOcTreeBase<Reach6DOcTreeNode>
{
public:
  Reach6DOcTree(): OccupancyOcTreeBase<Reach6DOcTreeNode>(0.1)
  {
      rotMapOcTreeMemberInit.ensureLinking();
      this->setClampingThresMax(probability(100));
  }
  /// Default constructor, sets resolution of leafs
  Reach6DOcTree(double res): OccupancyOcTreeBase<Reach6DOcTreeNode>(res)
  {
      rotMapOcTreeMemberInit.ensureLinking();
      this->setClampingThresMax(probability(100));
  }

  /// virtual constructor: creates a new object of same type
  /// (Covariant return type requires an up-to-date compiler)
  Reach6DOcTree* create() const
  {
      return new Reach6DOcTree(resolution);
  }

  std::string getTreeType() const
  {
      return "Reach6DOcTree";
  }

   /**
   * Prunes a node when it is collapsible. This overloaded
   * version only considers the node occupancy for pruning,
   * different colors of child nodes are ignored.
   * @return true if pruning was successful
   */
  virtual bool pruneNode(Reach6DOcTreeNode* node)
  {
      if (!isNodeCollapsible(node))
          return false;

      // set value to children's values (all assumed equal)
      node->copyData(*(getNodeChild(node, 0)));

      if (node->isRotMapSet()) // TODO check
        node->setRotMap(node->getAverageChildRotMap());

      // delete children
      for (unsigned int i=0;i<8;i++)
      {
        deleteNodeChild(node, i);
      }
      delete[] node->children;
      node->children = NULL;

      return true;
    }

  virtual bool isNodeCollapsible(const Reach6DOcTreeNode* node) const
  {
      // all children must exist, must not have children of
      // their own and have the same occupancy probability
      if (!nodeChildExists(node, 0))
        return false;

      const Reach6DOcTreeNode* firstChild = getNodeChild(node, 0);
      if (nodeHasChildren(firstChild))
        return false;

      for (unsigned int i = 1; i<8; i++) {
        // compare nodes only using their occupancy, ignoring color for pruning
        if (!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) || !(getNodeChild(node, i)->getValue() == firstChild->getValue()))
          return false;
      }

      return true;
  }

  // set node color at given key or coordinate. Replaces previous color.
  Reach6DOcTreeNode* setNodeRotMap(const OcTreeKey& key, const OcTree &map)
  {
      Reach6DOcTreeNode* n = search (key);
      if (n != 0)
      {
          n->setRotMap(map);
      }
      return n;
  }

  Reach6DOcTreeNode* setNodeRotMap(float x, float y,  float z, const OcTree &map)
  {
        OcTreeKey key;
        if (!this->coordToKeyChecked(point3d(x,y,z), key))
        {
            return nullptr;
        }
        return setNodeRotMap(key,map);
  }

  // integrate color measurement at given key or coordinate. Average with previous color
  Reach6DOcTreeNode* averageNodeRotMap(const OcTreeKey& key, const OcTree &map)
  {
     Reach6DOcTreeNode* n = search(key);
     if (n != 0)
     {
         if (n->isRotMapSet())
         {
            //TODO
            //Vector prev_vector = n->getVector();
            //n->setVector((prev_vector + v)/2);
         }
         else
         {
            n->setRotMap(map);
         }
     }
     return n;
  }

  Reach6DOcTreeNode* averageNodeVector(float x, float y, float z, const OcTree &map)
  {
        OcTreeKey key;
        if (!this->coordToKeyChecked(point3d(x,y,z), key))
        {
            return nullptr;
        }
        return averageNodeRotMap(key,map);
  }

  // integrate color measurement at given key or coordinate. Average with previous color
  Reach6DOcTreeNode* integrateNodeVector(const OcTreeKey& key, const OcTree &map)
  {
    Reach6DOcTreeNode* n = search (key);
    if (n != 0)
    {
        if (n->isRotMapSet())
        {
            //TODO
            //Vector prev_vector = n->getVector();
            //double node_prob = n->getOccupancy();
            //Vector new_vector = prev_vector * node_prob +  v * (0.99-node_prob);
            //n->setVector(new_vector);
        }
        else
        {
            n->setRotMap(map);
        }
    }
    return n;
  }

  Reach6DOcTreeNode* integrateNodeVector(float x, float y, float z, const OcTree &map)
  {
        OcTreeKey key;
        if (!this->coordToKeyChecked(point3d(x,y,z), key)) return NULL;
        return integrateNodeVector(key,map);
  }

  OcTree* ConvertToOcTree()
  {
      OcTree *m = new OcTree(this->getResolution());

      for(auto it=this->begin_leafs(), end=this->end_leafs(); it!=end; it++)
      {
          OcTreeNode* node = m->updateNode(it.getX(), it.getY(), it.getZ(), true, true);
          int v = it->getRotMap().getNumLeafNodes();
          node->setValue(v);
      }

      if(m->getRoot())
      {
          m->updateInnerOccupancy();
      }

      return m;
  }

  ColorOcTree* ConvertValueToColorOcTree()
  {
      ColorOcTree *m = new ColorOcTree(this->getResolution());

      int max=0;
      for(auto it=this->begin_leafs(), end=this->end_leafs(); it!=end; it++)
      {
          ColorOcTreeNode* node = m->updateNode(it.getX(), it.getY(), it.getZ(), true, true);
          int v = it->getValue();
          node->setValue(v);
          if(v > max)
              max = v;
      }

      for(auto it=m->begin_leafs(), end=m->end_leafs(); it!=end; it++)
      {
          uint8_t r,g,b;
          heatMap(0,max, it->getValue(), r,g,b);
          it->setColor(r,g,b);
      }

      if(m->getRoot())
      {
          m->updateInnerOccupancy();
      }

      return m;
  }

  ColorOcTree* ConvertRotDensityToColorOcTree()
  {
      ColorOcTree *m = new ColorOcTree(this->getResolution());

      int max=0;
      for(auto it=this->begin_leafs(), end=this->end_leafs(); it!=end; it++)
      {
          ColorOcTreeNode* node = m->updateNode(it.getX(), it.getY(), it.getZ(), true, true);
          int v = it->getRotMap().getNumLeafNodes();
          node->setValue(v);
          if(v > max)
              max = v;
      }

      for(auto it=m->begin_leafs(), end=m->end_leafs(); it!=end; it++)
      {
          uint8_t r,g,b;
          heatMap(0,max, it->getValue(), r,g,b);
          it->setColor(r,g,b);
      }

      if(m->getRoot())
      {
          m->updateInnerOccupancy();
      }

      return m;
  }

  // update inner nodes, sets color to average child color
  void updateInnerOccupancy()
  {
      this->updateInnerOccupancyRecurs(this->root, 0);
  }

protected:
  void updateInnerOccupancyRecurs(Reach6DOcTreeNode* node, unsigned int depth)
  {
     // only recurse and update for inner nodes:
     if (nodeHasChildren(node))
     {
       // return early for last level:
       if (depth < this->tree_depth)
       {
         for (unsigned int i=0; i<8; i++)
         {
           if (nodeChildExists(node, i))
           {
             updateInnerOccupancyRecurs(getNodeChild(node, i), depth+1);
           }
         }
       }
       node->updateOccupancyChildren();
       node->updateRotMapChildren();
     }
   }

  /**
   * Static member object which ensures that this OcTree's prototype
   * ends up in the classIDMapping only once. You need this as a
   * static member in any derived octree class in order to read .ot
   * files through the AbstractOcTree factory. You should also call
   * ensureLinking() once from the constructor.
   */
  class StaticMemberInitializer
  {
    public:
        StaticMemberInitializer()
        {
            Reach6DOcTree* tree = new Reach6DOcTree(0.1);
            tree->clearKeyRays();
            AbstractOcTree::registerTreeType(tree);
        }

       /**
       * Dummy function to ensure that MSVC does not drop the
       * StaticMemberInitializer, causing this tree failing to register.
       * Needs to be called from the constructor of this octree.
       */
       void ensureLinking() {};
  };
  /// static member to ensure static initialization (only once)
  static StaticMemberInitializer rotMapOcTreeMemberInit;
};

Reach6DOcTree::StaticMemberInitializer Reach6DOcTree::rotMapOcTreeMemberInit;

/****************************************************************/
Reach6DOcTree* SliceOcTreeZ(const Reach6DOcTree *const map, double level)
{
    double res = map->getResolution();
    double lim_l = res*(floor(level/res)-0.5);
    double lim_u = lim_l+res;

    Reach6DOcTree *m = new Reach6DOcTree(res);

    for(auto it=map->begin_leafs(), end=map->end_leafs(); it!=end; it++)
    {
        if(it.getZ() >= lim_l && it.getZ() < lim_u )
        {
            Reach6DOcTreeNode *node = m->updateNode(it.getX(), it.getY(), it.getZ(), true, true);
            node->setRotMap(it->getRotMap());
        }
    }

    return m;
}

/****************************************************************/
Reach6DOcTree* MoveInvMap(const Reach6DOcTree *const imap, const Matrix &Hm)
{
    Reach6DOcTree *m = new Reach6DOcTree(imap->getResolution());

    for(auto it=imap->begin_leafs(), end=imap->end_leafs(); it!=end; it++)
    {
        Vector p(3);
        p[0]=it.getX();
        p[1]=it.getY();
        p[2]=it.getZ();

        for(auto it2=it->getRotMap().begin_leafs(), end2=it->getRotMap().end_leafs(); it2!=end2; it2++)
        {
            Vector o(4,0.0);
            o[0] = it2.getX();
            o[1] = it2.getY();
            o[2] = it2.getZ();
            double n = norm(o);
            if(n > std::numeric_limits<double>::epsilon())
            {
                o = 1.0/n * o;
                o[3] = n;
            }

            Matrix H = axis2dcm(o);
            H[0][3]=p[0];
            H[1][3]=p[1];
            H[2][3]=p[2];
            H = Hm*H;
            Vector oi= dcm2axis(H);
            oi = oi[3]*oi;

            Reach6DOcTreeNode *node = m->updateNode(H[0][3], H[1][3], H[2][3], true, true);
            node->getRotMap().updateNode(oi[0], oi[1], oi[2], true, true);
        }
    }

    if(m->getRoot())
    {
        m->updateInnerOccupancy();
    }

    return m;
}

/****************************************************************/
Reach6DOcTree* ExtractGroundMap(const Reach6DOcTree *const map, const Matrix &Hm, const Vector &normalPlane, double thresPlane)
{
    double resp = map->getResolution();
    double limp_l = -0.5*resp;
    double limp_u = 0.5*resp;
    double reso = map->begin_leafs()->getRotMap().getResolution();
    double limo_l = -0.5*reso;
    double limo_u = 0.5*reso;
yDebug() << limp_l << limp_u << limo_l << limo_u;
    Reach6DOcTree *m = new Reach6DOcTree(map->getResolution());

    //std::unique_ptr<Reach6DOcTree> m2(MoveInvMap(map, Hm));

    Vector BBm(3);
    map->getMetricMin(BBm[0],BBm[1],BBm[2]);
    Vector BBM(3);
    map->getMetricMax(BBM[0],BBM[1],BBM[2]);

    Vector center=0.5*(BBM+BBm);
    double maxSize = 0.5*norm(BBM-BBm);

    Matrix R = Hm.submatrix(0,2, 0,2);
    Vector T = Hm.getCol(3).subVector(0,2);
    Matrix Hinv = SE3inv(Hm);
    Matrix Rinv = Hinv.submatrix(0,2, 0,2);
    Vector Tinv = Hinv.getCol(3).subVector(0,2);

    Vector centerW = R*center+T;
    centerW[2] = 0;
    Vector centerMap = Rinv*centerW+Tinv;

    Vector X = Rinv.getCol(0);
    Vector Y = Rinv.getCol(1);

    double n = norm(normalPlane);
    Vector normalMap(3,0.0);
    double thresMap=0;
    if(n > std::numeric_limits<double>::epsilon())
    {
        normalMap = 1.0/n*Rinv*normalPlane;
        thresMap = dot(Rinv*(thresPlane/n*normalPlane)+Tinv, normalMap);
    }

    for(double x=-maxSize; x<=maxSize; x+=resp)
    {
        for(double y=-maxSize; y<=maxSize; y+=resp)
        {
            Vector vmap = centerMap + x*X + y*Y;

            if(dot(vmap, normalMap)>thresMap) continue;

            int depthp=map->getTreeDepth();
            Reach6DOcTreeNode *nodep = map->search(vmap[0],vmap[1],vmap[2], depthp);
            depthp--;
            /*while(!nodep && depthp>map->getTreeDepth()-2)
            {
                nodep = map->search(vmap[0],vmap[1],vmap[2], depthp);
                depthp--;
            }*/

            if(nodep)
            {
                OcTree &rotMap = nodep->getRotMap();
//yDebug() << "Found pos" << x << y << level << "depth" << depthp;
                Vector oz(4,0.0);
                oz[2] = 1;
                for(oz[3]=-M_PI; oz[3]<=M_PI; oz[3]+=reso)
                {
                    Vector Z = dcm2axis(Rinv*axis2dcm(oz).submatrix(0,2, 0,2)*R);
                    int deptho=rotMap.getTreeDepth();
                    OcTreeNode *nodeo = rotMap.search(Z[3]*Z[0],Z[3]*Z[1],Z[3]*Z[2], deptho);
                    deptho--;
                    while(!nodeo && deptho>rotMap.getTreeDepth()-4)
                    {
                        nodeo = rotMap.search(Z[3]*Z[0],Z[3]*Z[1],Z[3]*Z[2], deptho);
                        deptho--;
                    }

                    if(nodeo)
                    {
//yDebug() << "      ang" << oz[3] << "depth" << deptho;
                        Reach6DOcTreeNode *node = m->updateNode(centerW[0]+x,centerW[1]+y,0,true, true);
                        node->getRotMap().updateNode(0,0,oz[3],true, true);
                    }
                }
            }
        }
    }

    /*for(auto it=map->begin_leafs(), end=map->end_leafs(); it!=end; it++)
    {
        Vector p(3);
        p[0]=it.getX();
        p[1]=it.getY();
        p[2]=it.getZ();

        for(auto it2=it->getRotMap().begin_leafs(), end2=it->getRotMap().end_leafs(); it2!=end2; it2++)
        {
            Vector o(4,0.0);
            o[0] = it2.getX();
            o[1] = it2.getY();
            o[2] = it2.getZ();
            double n = norm(o);
            if(n > std::numeric_limits<double>::epsilon())
            {
                o = 1.0/n * o;
                o[3] = n;
            }

            Matrix H = axis2dcm(o);
            H[0][3]=p[0];
            H[1][3]=p[1];
            H[2][3]=p[2];
            H = Hm*H;
            Vector oi= dcm2axis(H);
            oi = oi[3]*oi;
//yDebug() << H[0][3] << H[1][3] << H[2][3] << oi.toString();
//yDebug() << (oi[0] >= limo_l) << (oi[0] < limo_u) << (oi[1] >= limo_l) << (oi[1] < limo_u) << (H[2][3] >= limp_l) << (H[2][3] < limp_u);
            if( oi[0] >= limo_l && oi[0] < limo_u &&
                oi[1] >= limo_l && oi[1] < limo_u &&
                H[2][3] >= limp_l && H[2][3] < limp_u)
            {
                Reach6DOcTreeNode *node = m->updateNode(H[0][3], H[1][3], H[2][3], true, true);
                OcTreeNode *node2 = node->getRotMap().updateNode(oi[0], oi[1], oi[2], true, true);
            }
            else
            {

            }
        }
    }*/

    return m;
}

/****************************************************************/
Reach6DOcTree* NoisyZVerticalBaseMap(const Reach6DOcTree *const imap, const Matrix &Hm, const Vector &normalPlane, double thresPlane, const Vector &positionNoise, const Vector &orientationNoise)
{
    double resp = imap->getResolution();
    double reso = imap->begin_leafs()->getRotMap().getResolution();

    std::vector<Matrix> sampledH;

    for(double x=-positionNoise[0]; x<=positionNoise[0]; x+=resp)
    {
        for(double y=-positionNoise[1]; y<=positionNoise[1]; y+=resp)
        {
            for(double z=-positionNoise[2]; z<=positionNoise[2]; z+=resp)
            {
                for(double tx=-orientationNoise[0]; tx<=orientationNoise[0]; tx+=reso)
                {
                    for(double ty=-orientationNoise[1]; ty<=orientationNoise[1]; ty+=reso)
                    {
                        for(double tz=-orientationNoise[2]; tz<=orientationNoise[2]; tz+=reso)
                        {
                            Vector o(4,0.0);
                            o[0] = tx;
                            o[1] = ty;
                            o[2] = tz;
                            double n = norm(o);
                            if(n > std::numeric_limits<double>::epsilon())
                            {
                                o = 1.0/n * o;
                                o[3] = n;
                            }

                            Matrix H = axis2dcm(o);
                            H[0][3]=x;
                            H[1][3]=y;
                            H[2][3]=z;
                            sampledH.push_back(Hm*H);
                        }
                    }
                }
            }
        }
    }
yDebug() << "***************************************************************************************************************************************************************************";
    Reach6DOcTree *m = new Reach6DOcTree(resp);

    for(int i=0; i<sampledH.size(); i++)
    {
        std::unique_ptr<Reach6DOcTree> m2(ExtractGroundMap(imap, sampledH[i], normalPlane, thresPlane));

        for(auto it=m2->begin_leafs(), end=m2->end_leafs(); it!=end; it++)
        {
            Vector p(3);
            p[0]=it.getX();
            p[1]=it.getY();
            p[2]=it.getZ();

            for(auto it2=it->getRotMap().begin_leafs(), end2=it->getRotMap().end_leafs(); it2!=end2; it2++)
            {
                Vector o(3);
                o[0]=it2.getX();
                o[1]=it2.getY();
                o[2]=it2.getZ();

                Reach6DOcTreeNode *node = m->updateNode(p[0], p[1], p[2], true, true);
                node->getRotMap().updateNode(o[0], o[1], o[2], true, true);
            }
        }
    }

    return m;
}

/****************************************************************/
class ManipulabilityMap : public RFModule
{
    MobileArmSolver solver;
    RpcServer rpcPort;
    int verbosity;
    cer::robot_model::RobotModel *robotModel;

    int nbJointLevels;
    Reach6DOcTree reach6DMap;
    Reach6DOcTree iReach6DMap;

private:

    /****************************************************************/
    bool getBounds(const string &remote, const string &local, Matrix &lim) const
    {
        Property option;
        option.put("device","remote_controlboard");
        option.put("remote",remote);
        option.put("local",local);

        PolyDriver driver;
        if (driver.open(option))
        {
            IEncoders      *ienc;
            IControlLimits *ilim;

            driver.view(ienc);
            driver.view(ilim);

            int nAxes;
            ienc->getAxes(&nAxes);

            lim.resize(nAxes,2);
            for (int i=0; i<nAxes; i++)
                ilim->getLimits(i,&lim(i,0),&lim(i,1));

            driver.close();
            return true;
        }
        else
        {
            yError("Unable to connect to %s!",remote.c_str());
            return false;
        }
    }

    /****************************************************************/
    bool alignJointsBounds(const string &robot, const string &arm_type)
    {
        ArmParameters p=solver.getArmParameters();
        Matrix lim;

        if (getBounds("/"+robot+"/torso","/cer_manipulability-map/"+arm_type+"/torso",lim))
        {
            p.torso.l_min=lim(0,0);
            p.torso.l_max=lim(0,1);
            p.torso.alpha_max=fabs(lim(1,1));

            iCub::iKin::iKinChain &chain=*p.upper_arm.asChain();
            chain[0].setMin(CTRL_DEG2RAD*lim(3,0));
            chain[0].setMax(CTRL_DEG2RAD*lim(3,1));

            yInfo("limits of %s part: heave=[%g,%g] [m], [pitch,roll]=[%g,%g] [deg], yaw=[%g,%g] [deg]",
                  ("/"+robot+"/torso").c_str(),p.torso.l_min,p.torso.l_max,
                  -p.torso.alpha_max,p.torso.alpha_max,lim(3,0),lim(3,1));
        }
        else
            return false;

        if (getBounds("/"+robot+"/"+arm_type+"_arm","/cer_manipulability-map/"+arm_type+"/"+arm_type+"_arm",lim))
        {
            iCub::iKin::iKinChain &chain=*p.upper_arm.asChain();
            for (int i=0; i<5; i++)
            {
                chain[1+i].setMin(CTRL_DEG2RAD*lim(i,0));
                chain[1+i].setMax(CTRL_DEG2RAD*lim(i,1));

                yInfo("limits of %s part: joint %d=[%g,%g] [deg]",
                      ("/"+robot+"/"+arm_type+"_arm").c_str(),i,
                      CTRL_RAD2DEG*chain[1+i].getMin(),
                      CTRL_RAD2DEG*chain[1+i].getMax());
            }

            p.lower_arm.l_min=lim(5,0);
            p.lower_arm.l_max=lim(5,1);
            p.lower_arm.alpha_max=fabs(lim(6,1));

            yInfo("limits of %s part: heave=[%g,%g] [m], [pitch,roll]=[%g,%g] [deg]",
                  ("/"+robot+"/"+arm_type+"_wrist").c_str(),p.lower_arm.l_min,p.lower_arm.l_max,
                  -p.lower_arm.alpha_max,p.lower_arm.alpha_max);
        }
        else
            return false;

        solver.setArmParameters(p);
        return true;
    }

    /****************************************************************/
    void buildNoisyGroundMap(const Vector &noiseP, const Vector &noiseO, double level)
    {yDebug() << "z = " << level;
        Vector o(4,0.0);
        o[0] = 1.0;
        o[3] = -M_PI/2;
        Matrix H=axis2dcm(o);
        H[2][3] = level;
        //std::unique_ptr<Reach6DOcTree> rotatedMap;rotatedMap->updateNode(0,0,0,true);//(MoveInvMap(&iReach6DMap, H));
        Vector b(3,0.0);
        b[0]=1.0;
        double thres = -0.1;
        std::unique_ptr<Reach6DOcTree> groundMap(ExtractGroundMap(&iReach6DMap, H, b, thres));
        std::unique_ptr<Reach6DOcTree> groundNoisyMap(NoisyZVerticalBaseMap(&iReach6DMap, H, b, thres, noiseP, noiseO));
yDebug() << "d 1";
        //std::unique_ptr<ColorOcTree> rotatedMapColor(rotatedMap->ConvertToColorOcTree());
        std::ostringstream fileNameS;
        //fileNameS << "iReach6DMapColorRotated_res" << rotatedMapColor->getResolution() << "_" << nbJointLevels << "levels.ot";
        std::string fileName = fileNameS.str();
        /*if(rotatedMapColor->write(fileName))
        {
            yInfo() << "Map saved in" << fileName;
        }
        else
        {
            yError() << "Could not save map in" << fileName;
        }*/

        std::unique_ptr<ColorOcTree> groundMapColor(groundMap->ConvertRotDensityToColorOcTree());

        fileNameS.str("");
        fileNameS << "iReach6DMapGroundColor_res" << groundMapColor->getResolution() << "_" << nbJointLevels << "levels_height" << level << ".ot";
        fileName = fileNameS.str();
        if(groundMapColor->write(fileName))
        {
            yInfo() << "Map saved in" << fileName;
        }
        else
        {
            yError() << "Could not save map in" << fileName;
        }

        std::unique_ptr<ColorOcTree> groundNoisyMapColor(groundNoisyMap->ConvertValueToColorOcTree());
        fileNameS.str("");
        fileNameS << "iReach6DNoisyMapGroundColor_res" << groundNoisyMapColor->getResolution() << "_" << nbJointLevels << "levels_height" << level << "_noise" << noiseP[0] << "-" << noiseP[1] << "-" << noiseP[2] << "-" << noiseO[0] << "-" << noiseO[1] << "-" << noiseO[2] << ".ot";
        fileName = fileNameS.str();
        if(groundNoisyMapColor->write(fileName))
        {
            yInfo() << "Map saved in" << fileName;
        }
        else
        {
            yError() << "Could not save map in" << fileName;
        }
    }

    /****************************************************************/
    void buildInvMap()
    {
        iReach6DMap.setResolution(reach6DMap.getResolution());

        int max = 0;
        Reach6DOcTreeNode* maxNode = nullptr;
        int i = 0;
        int i_max = reach6DMap.getNumLeafNodes();
        double cur_progress = 0;
        for(auto it=reach6DMap.begin_leafs(), end=reach6DMap.end_leafs(); it!=end; it++)
        {
            Vector p(3);
            p[0] = it.getX();
            p[1] = it.getY();
            p[2] = it.getZ();

            for(auto it2=it->getRotMap().begin_leafs(), end2=it->getRotMap().end_leafs(); it2!=end2; it2++)
            {
                Vector o(4,0.0);
                o[0] = it2.getX();
                o[1] = it2.getY();
                o[2] = it2.getZ();
                double n = norm(o);
                if(n > std::numeric_limits<double>::epsilon())
                {
                    o = 1.0/n * o;
                    o[3] = n;
                }

                Matrix H = axis2dcm(o);
                H[0][3]=p[0];
                H[1][3]=p[1];
                H[2][3]=p[2];
                H = SE3inv(H);
                Vector oi= dcm2axis(H);
                oi = oi[3]*oi;

                Reach6DOcTreeNode* node = iReach6DMap.updateNode(H[0][3], H[1][3], H[2][3], true, true);
                node->getRotMap().updateNode(oi[0], oi[1], oi[2], true, true);

                if(node->getRotMap().getNumLeafNodes() > max)
                {
                    max = node->getRotMap().getNumLeafNodes();
                    maxNode = node;
                }
            }

            i++;
            if( 100*(double)i/(double)i_max > cur_progress )
            {
                yDebug() << cur_progress << "\%\033[F";
                cur_progress+=0.1;
            }

        }

        std::ostringstream fileNameS;
        fileNameS << "iReach6DMap_res" << reach6DMap.getResolution() << "_" << nbJointLevels << "levels.ot";
        std::string fileName = fileNameS.str();
        if(iReach6DMap.write(fileName))
        {
            yInfo() << "Map saved in" << fileName;
        }
        else
        {
            yError() << "Could not save map in" << fileName;
        }

        std::unique_ptr<OcTree> treeSimple(iReach6DMap.ConvertToOcTree());
        fileNameS.str("");
        fileNameS << "iReach6DMapSimple_res" << reach6DMap.getResolution() << "_" << nbJointLevels << "levels.ot";
        fileName = fileNameS.str();
        if(treeSimple->write(fileName))
        {
            yInfo() << "Map saved in" << fileName;
        }
        else
        {
            yError() << "Could not save map in" << fileName;
        }

        std::unique_ptr<ColorOcTree> treeColor(iReach6DMap.ConvertRotDensityToColorOcTree());
        fileNameS.str("");
        fileNameS << "iReach6DMapColor_res" << reach6DMap.getResolution() << "_" << nbJointLevels << "levels.ot";
        fileName = fileNameS.str();
        if(treeColor->write(fileName))
        {
            yInfo() << "Map saved in" << fileName;
        }
        else
        {
            yError() << "Could not save map in" << fileName;
        }

        fileNameS.str("");
        fileNameS << "iReach6DRotMapSample_res" << reach6DMap.getResolution() << "_" << nbJointLevels << "levels.ot";
        fileName = fileNameS.str();
        if(maxNode)
        {
            if(maxNode->getRotMap().write(fileName))
            {
                yInfo() << "Map saved in" << fileName;
            }
            else
            {
                yError() << "Could not save map in" << fileName;
            }
        }
    }

    /****************************************************************/
    void buildReach6DMapForward(int nbLevels, double posRes)
    {
        assert(nbLevels>0);
        assert(posRes>0.01);

        nbJointLevels = nbLevels;

        reach6DMap.clear();
        reach6DMap.setResolution(posRes);

        Vector q(15);
        Vector q_l(15);
        Vector q_u(15);

        for(size_t i=0 ; i<7; i++)
        {
            q_l[i]=q_u[i]=0;
        }

        iCub::iKin::iKinLimb uarm = solver.getArmParameters().upper_arm;
        TripodParameters larm = solver.getArmParameters().lower_arm;

        iCub::iKin::iKinChain *chain = uarm.asChain();
        for (size_t j=1; j<uarm.getDOF(); j++)
        {
            q_l[6+j]=CTRL_RAD2DEG*(*chain)[j].getMin();
            q_u[6+j]=CTRL_RAD2DEG*(*chain)[j].getMax();
        }

        for (size_t j=uarm.getDOF(); j<3+uarm.getDOF(); j++)
        {
            q_l[6+j]=larm.l_min;
            q_u[6+j]=larm.l_max;
        }

        std::function<int(int, Vector&)> loopJoint = [this, &loopJoint, &q_l, &q_u](int joint_idx, Vector &q)
        {
            static int eff_steps[15];

            int max=0;
            if(joint_idx>=15)
            {
                Matrix H;
                solver.fkin(q,H);

                Vector o = dcm2axis(H);
                o *= o[3];
                o.pop_back();

                Reach6DOcTreeNode *node = reach6DMap.updateNode((float)H[0][3], (float)H[1][3], (float)H[2][3], true, true);
                node->getRotMap().updateNode(o[0], o[1], o[2], true, true);
                max = node->getRotMap().getNumLeafNodes();

                static unsigned long long it=0;
                static unsigned long long it_max=[](){unsigned long long m=1;for(int i=0; i<15; i++)m*=eff_steps[i];return m;}();
                static double cur_progress=0;
                if(joint_idx==0)
                {
                    it=0;
                    cur_progress=0;
                }
                it++;
                //yDebug() << it << it_max;
                if( 100*(double)it/(double)it_max > cur_progress )
                {
                    yDebug() << cur_progress << "\%\033[F";
                    cur_progress+=0.1;
                }
            }
            else
            {
                double step = (q_u[joint_idx]-q_l[joint_idx])/nbJointLevels;
                eff_steps[joint_idx]=((step>0)?(nbJointLevels+1):1);

                for (q[joint_idx]=q_l[joint_idx]; q[joint_idx]<=q_u[joint_idx]; q[joint_idx]+=((step>0)?step:1))
                {
                    int v = loopJoint(joint_idx+1, q);
                    if(v > max)
                        max = v;
                }
            }

            return max;
        };

        int max = loopJoint(0,q);
        yDebug() << "Max rot" << max;

        yInfo() << "Reach map 6D built, size tree" << reach6DMap.memoryUsage();

        size_t size = 0;
        for(auto it=reach6DMap.begin_leafs(), end=reach6DMap.end_leafs(); it!=end; it++)
        {
            size += it->getRotMap().memoryUsage();
        }

        yInfo() << "Reach map 6D built, size nodes" << size;

        std::ostringstream fileNameS;
        fileNameS << "Reach6DMap_res" << reach6DMap.getResolution() << "_" << nbJointLevels << "levels.ot";
        std::string fileName = fileNameS.str();
        if(reach6DMap.write(fileName))
        {
            yInfo() << "Map saved in" << fileName;
        }
        else
        {
            yError() << "Could not save map in" << fileName;
        }

        std::unique_ptr<OcTree> treeSimple(reach6DMap.ConvertToOcTree());
        fileNameS.str("");
        fileNameS << "Reach6DMapSimple_res" << reach6DMap.getResolution() << "_" << nbJointLevels << "levels.ot";
        fileName = fileNameS.str();
        if(treeSimple->write(fileName))
        {
            yInfo() << "Map saved in" << fileName;
        }
        else
        {
            yError() << "Could not save map in" << fileName;
        }

        std::unique_ptr<ColorOcTree> treeColor(reach6DMap.ConvertRotDensityToColorOcTree());
        fileNameS.str("");
        fileNameS << "Reach6DMapColor_res" << reach6DMap.getResolution() << "_" << nbJointLevels << "levels.ot";
        fileName = fileNameS.str();
        if(treeColor->write(fileName))
        {
            yInfo() << "Map saved in" << fileName;
        }
        else
        {
            yError() << "Could not save map in" << fileName;
        }

        std::unique_ptr<ColorOcTree> rotSample;
        for(auto it=reach6DMap.begin_leafs(), end=reach6DMap.end_leafs(); it!=end; it++)
        {
            if(it->getRotMap().getNumLeafNodes() == max)
                rotSample.reset(it->ConvertNodeToColorOcTree());
        }

        fileNameS.str("");
        fileNameS << "Reach6DRotMapSample_res" << reach6DMap.getResolution() << "_" << nbJointLevels << "levels.ot";
        fileName = fileNameS.str();
        if(rotSample->write(fileName))
        {
            yInfo() << "Map saved in" << fileName;
        }
        else
        {
            yError() << "Could not save map in" << fileName;
        }
    }

    /****************************************************************/
    void buildConstrainedReach6DMapForward(int nbLevels, double posRes)
    {
        assert(nbLevels>0);
        assert(posRes>0.01);

        nbJointLevels = nbLevels;

        reach6DMap.clear();
        reach6DMap.setResolution(posRes);

        Vector q(15);
        Vector q_l(15);
        Vector q_u(15);

        for(size_t i=0 ; i<7; i++)
        {
            q_l[i]=q_u[i]=0;
        }

        iCub::iKin::iKinLimb uarm = solver.getArmParameters().upper_arm;
        TripodParameters larm = solver.getArmParameters().lower_arm;

        iCub::iKin::iKinChain *chain = uarm.asChain();
        for (size_t j=1; j<uarm.getDOF(); j++)
        {
            q_l[6+j]=CTRL_RAD2DEG*(*chain)[j].getMin();
            q_u[6+j]=CTRL_RAD2DEG*(*chain)[j].getMax();
        }

        for (size_t j=uarm.getDOF(); j<3+uarm.getDOF(); j++)
        {
            q_l[6+j]=larm.l_min;
            q_u[6+j]=larm.l_max;
        }

        std::function<int(int, Vector&)> loopJoint = [this, &loopJoint, &q_l, &q_u](int joint_idx, Vector &q)
        {
            static int eff_steps[15];

            int max=0;
            if(joint_idx>=15)
            {
                Matrix H;
                solver.fkin(q,H);

                Vector o = dcm2axis(H);
                o *= o[3];
                o.pop_back();

                Reach6DOcTreeNode *node = reach6DMap.updateNode((float)H[0][3], (float)H[1][3], (float)H[2][3], true, true);
                node->getRotMap().updateNode(o[0], o[1], o[2], true, true);
                max = node->getRotMap().getNumLeafNodes();

                static unsigned long long it=0;
                static unsigned long long it_max=[](){unsigned long long m=1;for(int i=0; i<15; i++)m*=eff_steps[i];return m;}();
                static double cur_progress=0;
                if(joint_idx==0)
                {
                    it=0;
                    cur_progress=0;
                }
                it++;
                //yDebug() << it << it_max;
                if( 100*(double)it/(double)it_max > cur_progress )
                {
                    yDebug() << cur_progress << "\%\033[F";
                    cur_progress+=0.1;
                }
            }
            else
            {
                double step = (q_u[joint_idx]-q_l[joint_idx])/nbJointLevels;
                eff_steps[joint_idx]=((step>0)?(nbJointLevels+1):1);

                for (q[joint_idx]=q_l[joint_idx]; q[joint_idx]<=q_u[joint_idx]; q[joint_idx]+=((step>0)?step:1))
                {
                    int v = loopJoint(joint_idx+1, q);
                    if(v > max)
                        max = v;
                }
            }

            return max;
        };

        int max = loopJoint(0,q);
        yDebug() << "Max rot" << max;

        yInfo() << "Reach map 6D built, size tree" << reach6DMap.memoryUsage();

        size_t size = 0;
        for(auto it=reach6DMap.begin_leafs(), end=reach6DMap.end_leafs(); it!=end; it++)
        {
            size += it->getRotMap().memoryUsage();
        }

        yInfo() << "Reach map 6D built, size nodes" << size;

        std::ostringstream fileNameS;
        fileNameS << "Reach6DMap_res" << reach6DMap.getResolution() << "_" << nbJointLevels << "levels.ot";
        std::string fileName = fileNameS.str();
        if(reach6DMap.write(fileName))
        {
            yInfo() << "Map saved in" << fileName;
        }
        else
        {
            yError() << "Could not save map in" << fileName;
        }

        std::unique_ptr<OcTree> treeSimple(reach6DMap.ConvertToOcTree());
        fileNameS.str("");
        fileNameS << "Reach6DMapSimple_res" << reach6DMap.getResolution() << "_" << nbJointLevels << "levels.ot";
        fileName = fileNameS.str();
        if(treeSimple->write(fileName))
        {
            yInfo() << "Map saved in" << fileName;
        }
        else
        {
            yError() << "Could not save map in" << fileName;
        }

        std::unique_ptr<ColorOcTree> treeColor(reach6DMap.ConvertRotDensityToColorOcTree());
        fileNameS.str("");
        fileNameS << "Reach6DMapColor_res" << reach6DMap.getResolution() << "_" << nbJointLevels << "levels.ot";
        fileName = fileNameS.str();
        if(treeColor->write(fileName))
        {
            yInfo() << "Map saved in" << fileName;
        }
        else
        {
            yError() << "Could not save map in" << fileName;
        }

        std::unique_ptr<ColorOcTree> rotSample;
        for(auto it=reach6DMap.begin_leafs(), end=reach6DMap.end_leafs(); it!=end; it++)
        {
            if(it->getRotMap().getNumLeafNodes() == max)
                rotSample.reset(it->ConvertNodeToColorOcTree());
        }

        fileNameS.str("");
        fileNameS << "Reach6DRotMapSample_res" << reach6DMap.getResolution() << "_" << nbJointLevels << "levels.ot";
        fileName = fileNameS.str();
        if(rotSample->write(fileName))
        {
            yInfo() << "Map saved in" << fileName;
        }
        else
        {
            yError() << "Could not save map in" << fileName;
        }
    }

    /****************************************************************/
    void buildReach6DMapInverse(double posRes)
    {
        assert(posRes>0.01);

        SolverParameters p = solver.getSolverParameters();
        p.setMode("no_base_no_torso_no_heave");
        solver.setSolverParameters(p);

        Vector q0(15);

        for(size_t i=0 ; i<7; i++)
        {
            q0[i]=q0[i]=0;
        }

        iCub::iKin::iKinLimb uarm = solver.getArmParameters().upper_arm;
        TripodParameters larm = solver.getArmParameters().lower_arm;

        iCub::iKin::iKinChain *chain = uarm.asChain();
        for (size_t j=1; j<uarm.getDOF(); j++)
        {
            q0[6+j]=0.5*CTRL_RAD2DEG*((*chain)[j].getMax()+(*chain)[j].getMin());
        }

        for (size_t j=uarm.getDOF(); j<3+uarm.getDOF(); j++)
        {
            q0[6+j]=0.5*(larm.l_max+larm.l_min);
        }

        solver.setInitialGuess(q0);

        reach6DMap.clear();
        reach6DMap.setResolution(posRes);

        Vector x(6);
        Vector res(6);
        res[0]=res[1]=res[2]=posRes;
        res[3]=res[4]=res[5]=ROT_MAP_RES;
        Vector x_l(6);
        x_l[0]=x_l[1]=x_l[2]=-2;
        x_l[3]=x_l[4]=x_l[5]=-M_PI;
        Vector x_u(6);
        x_u[0]=x_u[1]=x_u[2]=2;
        x_u[3]=x_u[4]=x_u[5]=M_PI;

        std::function<int(int, Vector&)> loopSpace = [this, &loopSpace, &res, &x_l, &x_u](int space_idx, Vector &x)
        {
            static unsigned long long it_max=(x_u[0]-x_l[0])*(x_u[1]-x_l[1])*(x_u[2]-x_l[2])*(x_u[3]-x_l[3])*(x_u[4]-x_l[4])*(x_u[5]-x_l[5])/(res[0]*res[1]*res[2]*res[3]*res[4]*res[5]);
            static unsigned long long it=0;
            static double cur_progress=0;
            if(space_idx==0)
            {
                it=0;
                cur_progress=0;
            }

            int max=0;
            if(space_idx>=6)
            {
                it++;
                if( 100*(double)it/(double)it_max > cur_progress )
                {
                    yDebug() << cur_progress << "\%\033[F";
                    cur_progress+=0.1;
                }

                Vector o(4,0.0);
                o.setSubvector(0,x.subVector(3,5));
                double n = norm(o);
                if(n > std::numeric_limits<double>::epsilon())
                {
                    o = 1.0/n * o;
                    o[3] = n;
                }

                Matrix H = axis2dcm(o);
                H[0][3]=x[0];
                H[1][3]=x[1];
                H[2][3]=x[2];

                Vector q(15);
                if(solver.ikin(H,q))
                {
                    Reach6DOcTreeNode *node = reach6DMap.updateNode(x[0], x[1], x[2], true, true);
                    node->getRotMap().updateNode(x[3], x[4], x[5], true, true);
                    max = node->getRotMap().getNumLeafNodes();
                }
            }
            else
            {
                for (x[space_idx]=x_l[space_idx]; x[space_idx]<=x_u[space_idx]; x[space_idx]+=res[space_idx])
                {
                    int v = loopSpace(space_idx+1, x);
                    if(v > max)
                        max = v;
                }
            }

            return max;
        };

        int max = loopSpace(0,x);
        yDebug() << "Max rot" << max;

        yInfo() << "Reach map 6D built, size tree" << reach6DMap.memoryUsage();

        size_t size = 0;
        for(auto it=reach6DMap.begin_leafs(), end=reach6DMap.end_leafs(); it!=end; it++)
        {
            size += it->getRotMap().memoryUsage();
        }

        yInfo() << "Reach map 6D built, size nodes" << size;

        std::ostringstream fileNameS;
        fileNameS << "Reach6DMap_res" << reach6DMap.getResolution() << "_inv";
        std::string fileName = fileNameS.str();
        if(reach6DMap.write(fileName))
        {
            yInfo() << "Map saved in" << fileName;
        }
        else
        {
            yError() << "Could not save map in" << fileName;
        }

        std::unique_ptr<OcTree> treeSimple(reach6DMap.ConvertToOcTree());
        fileNameS.str("");
        fileNameS << "Reach6DMapSimple_res" << reach6DMap.getResolution() << "_inv";
        fileName = fileNameS.str();
        if(treeSimple->write(fileName))
        {
            yInfo() << "Map saved in" << fileName;
        }
        else
        {
            yError() << "Could not save map in" << fileName;
        }

        std::unique_ptr<ColorOcTree> treeColor(reach6DMap.ConvertRotDensityToColorOcTree());
        fileNameS.str("");
        fileNameS << "Reach6DMapColor_res" << reach6DMap.getResolution() << "_inv";
        fileName = fileNameS.str();
        if(treeColor->write(fileName))
        {
            yInfo() << "Map saved in" << fileName;
        }
        else
        {
            yError() << "Could not save map in" << fileName;
        }

        std::unique_ptr<ColorOcTree> rotSample;
        for(auto it=reach6DMap.begin_leafs(), end=reach6DMap.end_leafs(); it!=end; it++)
        {
            if(it->getRotMap().getNumLeafNodes() == max)
                rotSample.reset(it->ConvertNodeToColorOcTree());
        }

        fileNameS.str("");
        fileNameS << "Reach6DRotMapSample_res" << reach6DMap.getResolution() << "_inv";
        fileName = fileNameS.str();
        if(rotSample->write(fileName))
        {
            yInfo() << "Map saved in" << fileName;
        }
        else
        {
            yError() << "Could not save map in" << fileName;
        }
    }

    /****************************************************************/
    bool loadReachMap(const std::string &path)
    {
        Reach6DOcTree* tree = (Reach6DOcTree*)Reach6DOcTree::read(path);
        reach6DMap.setResolution(tree->getResolution());
        reach6DMap.swapContent(*tree);
        delete tree;

        return true;
    }

    /****************************************************************/
    bool loadInvReachMap(const std::string &path)
    {
        Reach6DOcTree* tree = (Reach6DOcTree*)Reach6DOcTree::read(path);
        iReach6DMap.setResolution(tree->getResolution());
        iReach6DMap.swapContent(*tree);
        delete tree;

        return true;
    }

    /****************************************************************/
    void buildReachMap()
    {
        ManipulabilityOctTree tree(0.1);  // create empty tree with resolution 0.1

        ManipOcTreeDataNode *node = tree.updateNode(0.0, 0.0, 0.0, true, true);
        node->setColor((uint8_t)0, (uint8_t)0, (uint8_t)255);

        Vector q(15);
        Vector q_l(15);
        Vector q_u(15);

        for(size_t i=0 ; i<7; i++)
        {
            q_l[i]=q_u[i]=0;
        }

        iCub::iKin::iKinLimb uarm = solver.getArmParameters().upper_arm;
        TripodParameters larm = solver.getArmParameters().lower_arm;

        iCub::iKin::iKinChain *chain = uarm.asChain();
        for (size_t j=1; j<uarm.getDOF(); j++)
        {
            q_l[6+j]=CTRL_RAD2DEG*(*chain)[j].getMin();
            q_u[6+j]=CTRL_RAD2DEG*(*chain)[j].getMax();
        }

        for (size_t j=uarm.getDOF(); j<3+uarm.getDOF(); j++)
        {
            q_l[6+j]=larm.l_min;
            q_u[6+j]=larm.l_max;
        }

        std::function<int(int, Vector&)> loopJoint = [this, &loopJoint, &tree, &q_l, &q_u](int joint_idx, Vector &q)
        {
            int max=0;
            if(joint_idx>=15)
            {
                Matrix H;
                solver.fkin(q,H);

                ManipOcTreeDataNode *node = tree.updateNode((float)H[0][3], (float)H[1][3], (float)H[2][3], true, true);

                node->addValue(1);
                max = node->getValue();
            }
            else
            {
                const double nbStep = 6;
                double step = (q_u[joint_idx]-q_l[joint_idx])/nbStep;

                for (q[joint_idx]=q_l[joint_idx]; q[joint_idx]<=q_u[joint_idx]; q[joint_idx]+=((step>0)?step:1))
                {
                    int v = loopJoint(joint_idx+1, q);
                    if(v > max)
                        max = v;
                }
            }

            return max;
        };

        int max = loopJoint(0,q);

        for(auto it=tree.begin_leafs(), end=tree.end_leafs(); it!=end; it++)
        {
            uint8_t r,g,b;
            heatMap(0,max, it->getValue(), r,g,b);
            it->setColor(r,g,b);
        }

        tree.updateInnerOccupancy();


        if(tree.write("ReachMap.ot"))
        {
            yInfo() << "Map saved in ReachMap.ot";
        }
        else
        {
            yError() << "Could not save map in ReachMap.ot";
        }
    }

    /****************************************************************/
    void buildSelfCollideMap()
    {
        ManipulabilityOctTree tree(0.1);  // create empty tree with resolution 0.1

        ManipOcTreeDataNode *node = tree.updateNode(0.0, 0.0, 0.0, true, true);
        node->setColor((uint8_t)0, (uint8_t)0, (uint8_t)255);

        Vector q(15);
        Vector q_l(15);
        Vector q_u(15);

        for(size_t i=0 ; i<7; i++)
        {
            q_l[i]=q_u[i]=0;
        }

        iCub::iKin::iKinLimb uarm = solver.getArmParameters().upper_arm;
        TripodParameters larm = solver.getArmParameters().lower_arm;

        iCub::iKin::iKinChain *chain = uarm.asChain();
        for (size_t j=1; j<uarm.getDOF(); j++)
        {
            q_l[6+j]=CTRL_RAD2DEG*(*chain)[j].getMin();
            q_u[6+j]=CTRL_RAD2DEG*(*chain)[j].getMax();
        }

        for (size_t j=uarm.getDOF(); j<3+uarm.getDOF(); j++)
        {
            q_l[6+j]=larm.l_min;
            q_u[6+j]=larm.l_max;
        }

        std::function<void(int, Vector&)> loopJoint = [this, &loopJoint, &tree, &q_l, &q_u](int joint_idx, Vector &q)
        {
            if(joint_idx>=15)
            {
                Matrix H;
                solver.fkin(q,H);

                ManipOcTreeDataNode *node = tree.updateNode((float)H[0][3], (float)H[1][3], (float)H[2][3], true, true);

                cer::robot_model::Matrix qm(robotModel->getNDOF(),1,true);
                // qm (torso(4), left arm(8), right arm(8), head(2))
                for(int i=0; i<4; i++)
                    qm(i,0) = q_l[3+i];

                for(int i=0; i<8; i++)
                {
                    qm(4+i,0) = q_u[7+i];
                    qm(12+i,0) = q[7+i];
                }

                qm(20,0) = 0.0;
                qm(21,0) = 0.0;

                try
                {

                robotModel->calcConfig(qm);

                cer::robot_model::Matrix d;
                const cer::robot_model::Matrix& J = robotModel->calcInterference(d);
                /*Matrix JJ(J.R,J.C);
                for(int i=0; i<J.R; i++)
                    for(int j=0; j<J.C; j++)
                        JJ[i][j] = J(i,j);*/

                Matrix dd(d.R,d.C);
                for(int i=0; i<d.R; i++)
                    for(int j=0; j<d.C; j++)
                        dd[i][j] = d(i,j);

                double min=std::numeric_limits<double>::max();
                for(int i=0; i<dd.rows(); i++)
                {
                    if(dd[i][0] < min)
                        min = dd[i][0];
                }
yDebug() << min << "d: " << dd.toString();
                uint8_t r,g,b;
                heatMap(0, 0.17, min, r,g,b);
                node->setColor(r, g, b);
                }
                catch(...)
                {
                    node->setColor(0, 0, 0);
                }
            }
            else
            {
                const double nbStep = 5;
                double step = (q_u[joint_idx]-q_l[joint_idx])/nbStep;

                for (q[joint_idx]=q_l[joint_idx]; q[joint_idx]<=q_u[joint_idx]; q[joint_idx]+=((step>0)?step:1))
                {
                    loopJoint(joint_idx+1, q);
                }
            }
        };

        loopJoint(0,q);

        if(tree.write("SelfCollideMap.ot"))
        {
            yInfo() << "Map saved in SelfCollideMap.ot";
        }
        else
        {
            yError() << "Could not save map in SelfCollideMap.ot";
        }
    }

    /****************************************************************/
    void buildManipMap()
    {
        ManipulabilityOctTree tree(0.1);  // create empty tree with resolution 0.1

        ManipOcTreeDataNode *node = tree.updateNode(0.0, 0.0, 0.0, true, true);
        node->setColor((uint8_t)0, (uint8_t)0, (uint8_t)255);

        Vector q(15);
        Vector q_l(15);
        Vector q_u(15);

        for(size_t i=0 ; i<7; i++)
        {
            q_l[i]=q_u[i]=0;
        }

        iCub::iKin::iKinLimb uarm = solver.getArmParameters().upper_arm;
        TripodParameters larm = solver.getArmParameters().lower_arm;

        iCub::iKin::iKinChain *chain = uarm.asChain();
        for (size_t j=1; j<uarm.getDOF(); j++)
        {
            q_l[6+j]=CTRL_RAD2DEG*(*chain)[j].getMin();
            q_u[6+j]=CTRL_RAD2DEG*(*chain)[j].getMax();
        }

        for (size_t j=uarm.getDOF(); j<3+uarm.getDOF(); j++)
        {
            q_l[6+j]=larm.l_min;
            q_u[6+j]=larm.l_max;
        }

        std::function<void(int, Vector&)> loopJoint = [this, &loopJoint, &tree, &q_l, &q_u](int i, Vector &q)
        {
            if(i>=15)
            {
                Matrix H;
                solver.fkin(q,H);
                double manip=solver.getManip(q,manip);
                ManipOcTreeDataNode *node = tree.updateNode((float)H[0][3], (float)H[1][3], (float)H[2][3], true, true);
                node->setValue(manip);
                uint8_t r,g,b;
                heatMap(0, 0.05, std::max(node->getValue(), (float)manip), r,g,b);
                node->setColor(r, g, b); // set color
            }
            else
            {
                const double nbStep = 5;
                double step = (q_u[i]-q_l[i])/nbStep;

                for (q[i]=q_l[i]; q[i]<=q_u[i]; q[i]+=((step>0)?step:1))
                {
                    loopJoint(i+1, q);
                }
            }
        };

        loopJoint(0,q);

        tree.write("ManipMap.ot");
        yInfo() << "Map saved in ManipMap.ot";
    }

public:
    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string name=rf.check("name",Value("cer_manipulability-map")).asString();
        string arm_type=rf.check("arm-type",Value("left")).asString();
        verbosity=rf.check("verbosity",Value(0)).asInt();

        SolverParameters p=solver.getSolverParameters();
        p.setMode("full_pose");
        p.torso_heave=0.0;
        p.lower_arm_heave=0.02;
        p.warm_start=true;
        p.max_cpu_time=1;
        p.weight_postural_torso=0*1;
        p.weight_postural_torso_yaw=0*1e-2;
        p.weight_postural_upper_arm=0*1e-2;
        p.weight_postural_lower_arm=0*1;

        solver.setArmParameters(ArmParameters(arm_type));
        solver.setSolverParameters(p);
        solver.setVerbosity(verbosity);

        alignJointsBounds("SIM_CER_ROBOT",arm_type);

        rpcPort.open("/"+name+"/"+arm_type+"/rpc");
        attach(rpcPort);

        robotModel = new cer::robot_model::r1::R1Model();

        return true;
    }

    /****************************************************************/
    bool close()
    {
        rpcPort.close();
        return true;
    }

    /****************************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /****************************************************************/
    bool updateModule()
    {
        return true;
    }

    /****************************************************************/
    bool respond(const Bottle &cmd, Bottle &reply)
    {
        reply.clear();
        if(verbosity>0)
            yInfo() << "Received command:" << cmd.toString();

        if(cmd.get(0).asString() == "build_reach_map")
        {
            buildReachMap();
            reply.addVocab(Vocab::encode("ack"));
        }
        else if(cmd.get(0).asString() == "load_reach_map")
        {
            if(cmd.size() < 2)
            {
                yError() << "Missing path to mesh list file for load_all";
                reply.addVocab(Vocab::encode("nack"));
                return false;
            }
            std::string path = cmd.get(1).asString();
            if(loadReachMap(path))
                reply.addVocab(Vocab::encode("ack"));
            else
                reply.addVocab(Vocab::encode("ack"));
        }
        else if(cmd.get(0).asString() == "load_inv_reach_map")
        {
            if(cmd.size() < 2)
            {
                yError() << "Missing path to mesh list file for load_all";
                reply.addVocab(Vocab::encode("nack"));
                return false;
            }
            std::string path = cmd.get(1).asString();
            if(loadInvReachMap(path))
                reply.addVocab(Vocab::encode("ack"));
            else
                reply.addVocab(Vocab::encode("ack"));
        }
        else if(cmd.get(0).asString() == "build_inv_map")
        {
            buildInvMap();
            reply.addVocab(Vocab::encode("ack"));
        }
        else if(cmd.get(0).asString() == "build_reach_map_6D_forward")
        {
            int nbLevels = 4;
            if(cmd.size() > 1)
            {
               nbLevels = cmd.get(1).asInt();
            }
            double posRes = 0.1;
            if(cmd.size() > 2)
            {
               posRes = cmd.get(2).asDouble();
            }
            yInfo() << "Received build_reach_map_6D_forward with" << nbLevels << "levels and res" << posRes;
            buildReach6DMapForward(nbLevels, posRes);
            reply.addVocab(Vocab::encode("ack"));
        }
        else if(cmd.get(0).asString() == "build_reach_map_6D_inverse")
        {
            double posRes = 0.1;
            if(cmd.size() > 1)
            {
               posRes = cmd.get(1).asDouble();
            }
            yInfo() << "Received build_reach_map_6D_inverse with res" << posRes;
            buildReach6DMapInverse(posRes);
            reply.addVocab(Vocab::encode("ack"));
        }
        else if(cmd.get(0).asString() == "build_noisy_ground_map")
        {
            Vector noiseP(3,0.0);
            Vector noiseO(3,0.0);
            if(cmd.size() > 1)
            {
                Bottle *noise = cmd.get(1).asList();
                if(noise->size() != 6)
                {
                    yError() << "Invalid noise dimension for build_noisy_ground_map";
                    reply.addVocab(Vocab::encode("nack"));
                    return false;
                }
                noiseP[0] = noise->get(0).asDouble();
                noiseP[1] = noise->get(1).asDouble();
                noiseP[2] = noise->get(2).asDouble();
                noiseO[0] = noise->get(3).asDouble();
                noiseO[1] = noise->get(4).asDouble();
                noiseO[2] = noise->get(5).asDouble();
            }
            double z = -0.8;
            if(cmd.size() > 2)
            {
               z = cmd.get(2).asDouble();
            }

            buildNoisyGroundMap(noiseP, noiseO, z);
            reply.addVocab(Vocab::encode("ack"));
        }
        else if(cmd.get(0).asString() == "build_selfCollide_map")
        {
            buildSelfCollideMap();
            reply.addVocab(Vocab::encode("ack"));
        }
        else if(cmd.get(0).asString() == "build_manip_map")
        {
            buildManipMap();
            reply.addVocab(Vocab::encode("ack"));
        }
        else if(cmd.get(0).asString() == "help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("build_reach_map");
            reply.addString("load_reach_map path");
            reply.addString("load_inv_reach_map path");
            reply.addString("build_inv_map");
            reply.addString("build_reach_map_6D_forward nbLevels posRes");
            reply.addString("build_reach_map_6D_inverse posRes");
            reply.addString("build_noisy_ground_map (noiseDim6) level");
            reply.addString("build_selfCollide_map");
            reply.addString("build_manip_map");
        }
        else
        {
            reply.addVocab(Vocab::encode("nack"));
        }

        return true;
    }
};


/****************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return 1;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    ManipulabilityMap map;
    return map.runModule(rf);
}

