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
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/manager/utility.h>

#include <tinyxml.h>

#include <cer_mobile_kinematics/mobile_arm.h>

#include <fcl/fcl.h>

#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::manager;
using namespace cer::kinematics;

#define CTRL_DEG2RAD (M_PI/180.0)
#define CTRL_RAD2DEG (180.0/M_PI)

namespace fcl{

template <typename S>
struct CollisionData
{
  CollisionData()
  {
    done = false;
  }

  /// @brief Collision request
  CollisionRequest<S> request;

  /// @brief Collision result
  CollisionResult<S> result;

  /// @brief Whether the collision iteration can stop
  bool done;
};


/// @brief Distance data stores the distance request and the result given by distance algorithm.
template <typename S>
struct DistanceData
{
  DistanceData()
  {
    done = false;
  }

  /// @brief Distance request
  DistanceRequest<S> request;

  /// @brief Distance result
  DistanceResult<S> result;

  /// @brief Whether the distance iteration can stop
  bool done;

};


/// @brief Continuous collision data stores the continuous collision request and result given the continuous collision algorithm.
template <typename S>
struct ContinuousCollisionData
{
  ContinuousCollisionData()
  {
    done = false;
  }

  /// @brief Continuous collision request
  ContinuousCollisionRequest<S> request;

  /// @brief Continuous collision result
  ContinuousCollisionResult<S> result;

  /// @brief Whether the continuous collision iteration can stop
  bool done;
};

/// @brief Default collision callback for two objects o1 and o2 in broad phase. return value means whether the broad phase can stop now.
template <typename S>
bool defaultCollisionFunction(CollisionObject<S>* o1, CollisionObject<S>* o2, void* cdata_);

/// @brief Default distance callback for two objects o1 and o2 in broad phase. return value means whether the broad phase can stop now. also return dist, i.e. the bmin distance till now
template <typename S>
bool defaultDistanceFunction(CollisionObject<S>* o1, CollisionObject<S>* o2, void* cdata_, S& dist);

template <typename S>
bool defaultContinuousCollisionFunction(ContinuousCollisionObject<S>* o1, ContinuousCollisionObject<S>* o2, void* cdata_);

template <typename S>
bool defaultContinuousDistanceFunction(ContinuousCollisionObject<S>* o1, ContinuousCollisionObject<S>* o2, void* cdata_, S& dist);

//==============================================================================
template <typename S>
bool defaultCollisionFunction(CollisionObject<S>* o1, CollisionObject<S>* o2, void* cdata_)
{
  auto* cdata = static_cast<CollisionData<S>*>(cdata_);
  const auto& request = cdata->request;
  auto& result = cdata->result;

  if(cdata->done) return true;

  collide(o1, o2, request, result);

  if(!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
    cdata->done = true;

  return cdata->done;
}

//==============================================================================
template <typename S>
bool defaultDistanceFunction(CollisionObject<S>* o1, CollisionObject<S>* o2, void* cdata_, S& dist)
{
  auto* cdata = static_cast<DistanceData<S>*>(cdata_);
  const DistanceRequest<S>& request = cdata->request;
  DistanceResult<S>& result = cdata->result;

  if(cdata->done) { dist = result.min_distance; return true; }

  distance(o1, o2, request, result);

  dist = result.min_distance;

  if(dist <= 0) return true; // in collision or in touch

  return cdata->done;
}

//==============================================================================
template <typename S>
bool defaultContinuousCollisionFunction(ContinuousCollisionObject<S>* o1, ContinuousCollisionObject<S>* o2, void* cdata_)
{
  auto* cdata = static_cast<ContinuousCollisionData<S>*>(cdata_);
  const ContinuousCollisionRequest<S>& request = cdata->request;
  ContinuousCollisionResult<S>& result = cdata->result;

  if(cdata->done) return true;

  collide(o1, o2, request, result);

  return cdata->done;
}

//==============================================================================
template <typename S>
bool defaultContinuousDistanceFunction(ContinuousCollisionObject<S>* o1, ContinuousCollisionObject<S>* o2, void* cdata_, S& dist)
{
  FCL_UNUSED(o1);
  FCL_UNUSED(o2);
  FCL_UNUSED(cdata_);
  FCL_UNUSED(dist);

  return true;
}

}

/****************************************************************/
class CollisionDetector : public RFModule
{
    PolyDriver  drivers[9];
    IEncodersTimed* ienc[9];

    MobileArmSolver solverRight;
    MobileArmSolver solverLeft;
    RpcServer rpcPort;
    int verbosity;

    std::vector<std::string> robotPartNames;
    std::vector<std::shared_ptr<fcl::CollisionGeometryd>> robotPartGeometry;
    std::vector<fcl::Transform3d> robotPartLinkPose;
    std::vector<fcl::Transform3d> robotPartLinkPoseInit;
    std::vector<fcl::Transform3d> robotPartGeometryOffsetPose;

    std::vector<fcl::CollisionObjectd*> collisionModels;
    fcl::BroadPhaseCollisionManagerd* collisionBody;
    fcl::BroadPhaseCollisionManagerd* collisionRightArm;
    fcl::BroadPhaseCollisionManagerd* collisionLeftArm;


public:



    /****************************************************************/
    CollisionDetector():
        collisionBody(nullptr),
        collisionRightArm(nullptr),
        collisionLeftArm(nullptr)
    {

    }

    /****************************************************************/
    ~CollisionDetector()
    {

    }

    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        std::string robot=rf.check("robot",Value("cer")).asString();
        verbosity=rf.check("verbosity",Value(0)).asInt();

        ArmParameters arm_r("right");
        arm_r.upper_arm.setAllConstraints(false);
        solverRight.setArmParameters(arm_r);
        solverRight.setVerbosity(verbosity);

        ArmParameters arm_l("left");
        arm_l.upper_arm.setAllConstraints(false);
        solverLeft.setArmParameters(arm_l);
        solverLeft.setVerbosity(verbosity);

        Property option;

        option.put("device","remote_controlboard");
        option.put("remote","/"+robot+"/torso_tripod");
        option.put("local","/cer_collision-detection/torso_tripod");
        option.put("writeStrict","on");
        if (!drivers[0].open(option))
        {
            yError("Unable to connect to %s",string("/"+robot+"/torso_tripod").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote","/"+robot+"/torso");
        option.put("local","/cer_collision-detection/torso");
        option.put("writeStrict","on");
        if (!drivers[1].open(option))
        {
            yError("Unable to connect to %s",string("/"+robot+"/torso").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote","/"+robot+"/right_arm");
        option.put("local","/cer_collision-detection/right_arm");
        option.put("writeStrict","on");
        if (!drivers[2].open(option))
        {
            yError("Unable to connect to %s",string("/"+robot+"/right_arm").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote","/"+robot+"/right_wrist_tripod");
        option.put("local","/cer_collision-detection/right_wrist_tripod");
        option.put("writeStrict","on");
        if (!drivers[3].open(option))
        {
            yError("Unable to connect to %s",string("/"+robot+"/right_wrist_tripod").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote","/"+robot+"/right_hand");
        option.put("local","/cer_collision-detection/right_hand");
        option.put("writeStrict","on");
        if (!drivers[4].open(option))
        {
            yError("Unable to connect to %s",string("/"+robot+"/right_hand").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote","/"+robot+"/left_arm");
        option.put("local","/cer_collision-detection/left_arm");
        option.put("writeStrict","on");
        if (!drivers[5].open(option))
        {
            yError("Unable to connect to %s",string("/"+robot+"/left_arm").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote","/"+robot+"/left_wrist_tripod");
        option.put("local","/cer_collision-detection/left_wrist_tripod");
        option.put("writeStrict","on");
        if (!drivers[6].open(option))
        {
            yError("Unable to connect to %s",string("/"+robot+"/left_wrist_tripod").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote","/"+robot+"/left_hand");
        option.put("local","/cer_collision-detection/left_hand");
        option.put("writeStrict","on");
        if (!drivers[7].open(option))
        {
            yError("Unable to connect to %s",string("/"+robot+"/left_hand").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote","/"+robot+"/head");
        option.put("local","/cer_collision-detection/head");
        option.put("writeStrict","on");
        if (!drivers[8].open(option))
        {
            yError("Unable to connect to %s",string("/"+robot+"/head").c_str());
            close();
            return false;
        }

        for (int i=0; i<9; i++)
        {
            drivers[i].view(ienc[i]);
            if(!ienc[i])
            {
                yError("Unable to get interface for %d", i);
                close();
                return false;
            }
        }

        rpcPort.open("/cer_collision-detection/rpc");
        attach(rpcPort);

        return true;
    }

    void addMesh(const aiMesh *srcMesh, fcl::BVHModel<fcl::OBBRSSd>* dstMesh, const Vector &scaling=Vector(3,1.0))
    {
        assert(srcMesh);
        assert(dstMesh);
        assert(scaling.size()==3);

        for (int i=0; i<srcMesh->mNumFaces; i++)
        {
            fcl::Vector3d vertices[3];
            for (int j=0; j<3; j++)
            {
                const aiVector3D& vertex = srcMesh->mVertices[srcMesh->mFaces[i].mIndices[j]];
                vertices[j] = fcl::Vector3d(scaling[0]*vertex.x, scaling[1]*vertex.y, scaling[2]*vertex.z);
                //yDebug() << vertex.x << vertex.y << vertex.z << vertices[j][0] << vertices[j][1] << vertices[j][2];
            }
            dstMesh->addTriangle(vertices[0], vertices[1], vertices[2]);
        }
    }

    void addMeshFromAssimpNode(aiNode* node, const aiScene* scene, fcl::BVHModel<fcl::OBBRSSd>* dstMesh, const Vector &scaling=Vector(3,1.0))
    {
        for (int i=0; i<node->mNumMeshes; i++)
        {
            addMesh(scene->mMeshes[node->mMeshes[i]], dstMesh, scaling);
        }

        for (int i=0; i<node->mNumChildren; i++)
        {
            this->addMeshFromAssimpNode(node->mChildren[i], scene, dstMesh, scaling);
        }
    }

    /****************************************************************/
    bool loadMeshModel(std::string path, const Vector &scaling=Vector(3,1.0))
    {
        Assimp::Importer import;
        const aiScene* scene = import.ReadFile(path, aiProcess_Triangulate);

        if(!scene || scene->mFlags == AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
        {
            yError() << __FUNCTION__ << ": Could not load mesh model:" << import.GetErrorString();
            return false;
        }

        fcl::BVHModel<fcl::OBBRSSd>* mesh = new fcl::BVHModel<fcl::OBBRSSd>;

        mesh->beginModel();
        addMeshFromAssimpNode(scene->mRootNode, scene, mesh, scaling);
        mesh->endModel();

        robotPartGeometry.push_back(std::shared_ptr<fcl::CollisionGeometryd>(mesh));

        yInfo() << "Loaded file" << path << "with scaling" << scaling.toString();

        return true;
    }

    /****************************************************************/
    bool loadAll(std::string path)
    {
        ifstream file(path);
        if(!file.is_open())
        {
            yError() << __FUNCTION__ << ": Could not open file" << path;
            return false;
        }

        robotPartNames.clear();
        robotPartGeometry.clear();

        std::string meshFile;
        double scaling;
        bool success = true;
        while (success && (file >> meshFile >> scaling))
        {
            success = loadMeshModel(meshFile, scaling);
        }

        if(success)
        {
            yInfo() << "Loaded" << robotPartGeometry.size() << "meshes";
            return true;
        }
        else
        {
            robotPartGeometry.clear();
            yError() << "Could not load mesh list";
            return false;
        }
    }

    bool loadSDF(std::string path)
    {
        TiXmlDocument doc(path);
        if(!doc.LoadFile())
        {
            yError() << __FUNCTION__ << ": Syntax error while loading " << path << "at line " << doc.ErrorRow() << ":\n" << doc.ErrorDesc();
            return false;
        }

        size_t c = path.find_last_of("/");
        if(c == std::string::npos)
        {
            yError() << __FUNCTION__ << ": Could not deduce file directory";
            return false;
        }

        std::string fileDir = path.substr(0,c+1);
        yInfo() << "Found file dir" << fileDir;

        TiXmlElement *root = doc.RootElement();
        if(!root)
        {
            yError() << __FUNCTION__ << ": Syntax error while loading" << path << ":\nNo root element.";
            return false;
        }

        if(!compareString(root->Value(), "sdf"))
        {
            yError() << __FUNCTION__ << ": Not a .sdf file:" << path;
            return false;
        }

        TiXmlElement* model = (TiXmlElement*) root->FirstChild("model");
        if(!model)
        {
            yError() << __FUNCTION__ << ": Module from" << path << " has no model 2.";
            return false;
        }

        yInfo() << "Found robot model" << model->Attribute("name");

        TiXmlElement* link=nullptr;
        while(link=(TiXmlElement*) model->IterateChildren("link", link))
        {
            robotPartNames.push_back(link->Attribute("name"));

            yInfo() << "Found link" << robotPartNames.back();

            TiXmlElement* linkPose = (TiXmlElement*) link->FirstChild("pose");
            if(!linkPose || !linkPose->GetText())
            {
                yError() << __FUNCTION__ << ": Link" << robotPartNames.back() << "has no pose.";
                return false;
            }

            Bottle u;
            u.fromString(linkPose->GetText());

            Vector linkPositionV(3,0.0);
            linkPositionV[0]=u.get(0).asDouble();
            linkPositionV[1]=u.get(1).asDouble();
            linkPositionV[2]=u.get(2).asDouble();
            Vector linkOrientationV(3,0.0);
            linkOrientationV[0]=u.get(3).asDouble();
            linkOrientationV[1]=u.get(4).asDouble();
            linkOrientationV[2]=u.get(5).asDouble();
            linkOrientationV=dcm2axis(euler2dcm(linkOrientationV));

            fcl::Transform3d initPose;
            initPose.fromPositionOrientationScale(
                        fcl::Vector3d(linkPositionV[0], linkPositionV[1], linkPositionV[2]),
                        fcl::AngleAxisd(linkOrientationV[3], fcl::Vector3d(linkOrientationV[0],linkOrientationV[1],linkOrientationV[2])),
                        fcl::Vector3d(1,1,1));
            robotPartLinkPose.push_back(initPose);

            yInfo() << "Found link pose" << linkPositionV.toString() << linkOrientationV.toString();

            TiXmlElement* collision = (TiXmlElement*) link->FirstChild("collision");
            if(!collision)
            {
                yError() << __FUNCTION__ << ": Link" << robotPartNames.back() << "has no collision.";
                return false;
            }

            yInfo() << "Found collision" << collision->Attribute("name");

            TiXmlElement* collisionpPose = (TiXmlElement*) collision->FirstChild("pose");
            if(!collisionpPose || !collisionpPose->GetText())
            {
                yError() << __FUNCTION__ << ": Collision" << collision->Attribute("name") << "has no pose.";
                return false;
            }

            u.clear();
            u.fromString(collisionpPose->GetText());

            Vector collisionPositionV(3,0.0);
            collisionPositionV[0]=u.get(0).asDouble();
            collisionPositionV[1]=u.get(1).asDouble();
            collisionPositionV[2]=u.get(2).asDouble();
            Vector collisionOrientationV(3,0.0);
            collisionOrientationV[0]=u.get(3).asDouble();
            collisionOrientationV[1]=u.get(4).asDouble();
            collisionOrientationV[2]=u.get(5).asDouble();
            collisionOrientationV = dcm2axis(euler2dcm(collisionOrientationV));

            fcl::Transform3d collisionPose;
            collisionPose.fromPositionOrientationScale(
                        fcl::Vector3d(collisionPositionV[0], collisionPositionV[1], collisionPositionV[2]),
                        fcl::AngleAxisd(collisionOrientationV[3], fcl::Vector3d(collisionOrientationV[0],collisionOrientationV[1],collisionOrientationV[2])),
                        fcl::Vector3d(1,1,1));
            robotPartGeometryOffsetPose.push_back(collisionPose);

            yInfo() << "Found collision pose" << collisionPositionV.toString() << collisionOrientationV.toString();

            TiXmlElement* geometry = (TiXmlElement*) collision->FirstChild("geometry");
            if(!geometry)
            {
                yError() << __FUNCTION__ << ": Collision" << collision->Attribute("name") << "has no geometry.";
                return false;
            }

            yInfo() << "Found geometry";

            // Fork depending on type

            if(TiXmlElement* mesh = (TiXmlElement*) geometry->FirstChild("mesh"))
            {
                yInfo() << "Found mesh";

                TiXmlElement* scale = (TiXmlElement*) mesh->FirstChild("scale");
                if(!scale)
                {
                    yError() << __FUNCTION__ << ": Mesh of collision" << collision->Attribute("name") << "has no scale.";
                    return false;
                }

                u.clear();
                u.fromString(scale->GetText());
                Vector scaleV;
                u.write(scaleV);

                yInfo() << "Found scale" << scaleV.size() << scaleV.toString();

                TiXmlElement* uri = (TiXmlElement*) mesh->FirstChild("uri");
                if(!uri || !uri->GetText())
                {
                    yError() << __FUNCTION__ << ": Mesh of collision" << collision->Attribute("name") << "has no uri.";
                    return false;
                }

                yInfo() << "Found uri" << uri->GetText();

                const std::string fileToken = "model://cer/";
                std::string meshFile = uri->GetText();
                c = meshFile.find_first_of(fileToken);

                if(c == std::string::npos)
                {
                    yError() << __FUNCTION__ << ": Could not deduce mesh file path for collision" << collision->Attribute("name");
                    return false;
                }

                meshFile = fileDir + meshFile.substr(c+fileToken.size(),std::string::npos);

                yInfo() << "Found mesh file" << meshFile;

                if(!loadMeshModel(meshFile, scaleV))
                {
                    yError() << __FUNCTION__ << ": Could not load mesh file" << meshFile;
                }
            }
            else if(TiXmlElement* cylinder = (TiXmlElement*) geometry->FirstChild("cylinder"))
            {
                yInfo() << "Found cylinder";

                TiXmlElement* length = (TiXmlElement*) cylinder->FirstChild("length");
                if(!length || !length->GetText())
                {
                    yError() << __FUNCTION__ << ": Cylinder of collision" << collision->Attribute("name") << "has no length.";
                    return false;
                }

                u.clear();
                u.fromString(length->GetText());
                double lengthD=u.get(0).asDouble();

                yInfo() << "Found length" << lengthD;

                TiXmlElement* radius = (TiXmlElement*) cylinder->FirstChild("radius");
                if(!radius || !radius->GetText())
                {
                    yError() << __FUNCTION__ << ": Cylinder of collision" << collision->Attribute("name") << "has no radius.";
                    return false;
                }
                u.clear();
                u.fromString(radius->GetText());
                double radiusD=u.get(0).asDouble();

                yInfo() << "Found radius" << radiusD;

                fcl::Cylinderd* cylinderModel = new fcl::Cylinderd(radiusD, lengthD);
                robotPartGeometry.push_back(std::shared_ptr<fcl::CollisionGeometryd>(cylinderModel));
            }
            else
            {
                yError() << __FUNCTION__ << ": Collision" << collision->Attribute("name") << " has unkown collision geometry.";
                return false;
            }
        }

        robotPartLinkPoseInit = robotPartLinkPose;

        {
        fcl::CollisionRequestd requestC;
        fcl::CollisionResultd resultC;
        fcl::DistanceRequestd requestD;
        fcl::DistanceResultd resultD;


        fcl::Transform3d t = fcl::Transform3d::Identity();
        t.fromPositionOrientationScale(
                    fcl::Vector3d(0,0,0.1),
                    fcl::AngleAxisd(0, fcl::Vector3d(1,0,0)),
                    fcl::Vector3d(1,1,1));


        fcl::CollisionObjectd* o0 = new fcl::CollisionObjectd(robotPartGeometry[0], robotPartLinkPose[0]*robotPartGeometryOffsetPose[0]);
        fcl::CollisionObjectd* o1 = new fcl::CollisionObjectd(robotPartGeometry[10], robotPartLinkPose[10]*robotPartGeometryOffsetPose[10]);

        fcl::collide(o0, o1, requestC, resultC);
        yDebug() << (resultC.isCollision()?"yes":"no");
        yDebug() << fcl::distance(o0, o1, requestD, resultD);
        yDebug() << resultD.min_distance;

        delete o0;
        delete o1;
    }

        if(!generateCollisionModels())
        {
            yError() << __FUNCTION__ << ": Error during generation of collision models";
            return false;
        }
yDebug() << "Collision models generated";


        /*if(!generateCollisionManagers())
        {
            yError() << __FUNCTION__ << ": Error during generation of collision managers";
            return false;
        }*/

        return true;
    }

    /****************************************************************/
    bool generateCollisionModels()
    {
        collisionModels.clear();

        for(size_t i=0; i<robotPartGeometry.size(); i++)
        {
            fcl::CollisionObjectd* p=new fcl::CollisionObjectd(robotPartGeometry[i], robotPartLinkPose[i]*robotPartGeometryOffsetPose[i]);
            if(!p)
            {
                yError() << "Could not create collision model for" << robotPartNames[i];
                return false;
            }
            collisionModels.push_back(p);
            fcl::Vector3d v= collisionModels.back()->collisionGeometry().get()->computeCOM();
            yDebug() << robotPartNames[i] << v[0] << v[1] << v[2] << collisionModels.back()->collisionGeometry().get()->computeVolume() << collisionModels.back()->collisionGeometry().get()->aabb_radius;
        }

        return true;
    }

    /****************************************************************/
    bool generateCollisionManagers()
    {
        if(collisionBody)
            delete collisionBody;
        collisionBody = new fcl::DynamicAABBTreeCollisionManagerd();
        if(!collisionBody)
        {
            yError() << "Could not create collision manager for robot body";
            return false;
        }

        if(collisionRightArm)
            delete collisionRightArm;
        collisionRightArm = new fcl::DynamicAABBTreeCollisionManagerd();
        if(!collisionRightArm)
        {
            yError() << "Could not create collision manager for robot right arm";
            return false;
        }

        if(collisionLeftArm)
            delete collisionLeftArm;
        collisionLeftArm = new fcl::DynamicAABBTreeCollisionManagerd();
        if(!collisionLeftArm)
        {
            yError() << "Could not create collision manager for robot left arm";
            return false;
        }

        for(size_t i=0; i<robotPartNames.size(); i++)
        {
            if(robotPartNames[i].find_first_of("r_") == 0)
            {
                collisionRightArm->registerObject(collisionModels[i]);
                yDebug() << robotPartNames[i] << "registered in right arm collision manager";
            }
            else if(robotPartNames[i].find_first_of("l_") == 0 && robotPartNames[i].find("clavicle") == std::string::npos && robotPartNames[i].find("shoulder") == std::string::npos)
            {
                collisionLeftArm->registerObject(collisionModels[i]);
                yDebug() << robotPartNames[i] << "registered in left arm collision manager";
            }
            else if(robotPartNames[i].find_first_of("l_") != 0)
            {
                collisionBody->registerObject(collisionModels[i]);
                yDebug() << robotPartNames[i] << "registered in body collision manager";
            }
        }

        collisionBody->setup();
        collisionRightArm->setup();
        collisionLeftArm->setup();

        yDebug() << "CollisionBody registered" << collisionBody->size() << "objects";
        yDebug() << "CollisionRightArm registered" << collisionRightArm->size() << "objects";
        yDebug() << "CollisionLeftArm registered" << collisionLeftArm->size() << "objects";

        for(size_t i=0; i<robotPartGeometry.size(); i++)
        {
            fcl::Vector3d v= collisionModels[i]->collisionGeometry().get()->computeCOM();
            yDebug() << robotPartNames[i] << v[0] << v[1] << v[2] << collisionModels[i]->collisionGeometry().get()->computeVolume() << collisionModels[i]->collisionGeometry().get()->aabb_radius;
        }

        return true;
    }

    /****************************************************************/
    bool updateMeshPosition(const Vector &q)
    {
        if(robotPartGeometry.size() < 2)
        {
            yError() << "Not enough meshes to check";
        }

        // Update mesh transforms

        Vector q_right(15,0.0);
        q_right.setSubvector(0,q.subVector(0,2));
        q_right.setSubvector(3,q.subVector(3,5));
        q_right[6]=q[6];
        q_right.setSubvector(7,q.subVector(7,14));

        Vector q_left(15,0.0);
        q_left.setSubvector(0,q.subVector(0,2));
        q_left.setSubvector(3,q.subVector(3,5));
        q_left[6]=q[6];
        q_left.setSubvector(7,q.subVector(19,26));

yDebug() << "q_r" << q_right.toString();
yDebug() << "q_l" << q_left.toString();

        Matrix H(4,4);

        // Torso
        for(size_t i=6 ; i<7; i++)
        {
            solverRight.fkin(q_right,H,i-6);

            Matrix Hdb(4,4);
            for(size_t r=0 ; r<4; r++)
                for(size_t c=0 ; c<4; c++)
                    Hdb[r][c]=robotPartLinkPose[i](r,c);
yDebug() << robotPartNames[i] << "from\n" << Hdb.toString(4,4) << "\nto\n" << H.toString(4,4);

fcl::Transform3d pose = fcl::Transform3d::Identity();
            pose.translation() = fcl::Vector3d(H[0][3], H[1][3], H[2][3]);
            fcl::Matrix3d R;
            for(size_t r=0 ; r<3; r++)
                for(size_t c=0 ; c<3; c++)
                    R(r,c)=H[r][c];

            pose.linear()=R;
            pose = pose * robotPartGeometryOffsetPose[i];
            collisionModels[i]->setTransform(pose);
        }

        // Left upper arm
        for(size_t i=7 ; i<12; i++)
        {
            solverLeft.fkin(q_left,H,i-7+4);

            Matrix Hdb(4,4);
            for(size_t r=0 ; r<4; r++)
                for(size_t c=0 ; c<4; c++)
                    Hdb[r][c]=robotPartLinkPose[i](r,c);
yDebug() << robotPartNames[i] << "from\n" << Hdb.toString(4,4) << "\nto\n" << H.toString(4,4);

            fcl::Transform3d pose = fcl::Transform3d::Identity();
            pose.translation() = fcl::Vector3d(H[0][3], H[1][3], H[2][3]);
            fcl::Matrix3d R;
            for(size_t r=0 ; r<3; r++)
                for(size_t c=0 ; c<3; c++)
                    R(r,c)=H[r][c];

            pose.linear()=R;
            pose = pose * robotPartGeometryOffsetPose[i];
            collisionModels[i]->setTransform(pose);
        }

        // Left hand
        for(size_t i=14 ; i<15; i++)
        {
            solverLeft.fkin(q_left,H,-1);

            Matrix Hdb(4,4);
            for(size_t r=0 ; r<4; r++)
                for(size_t c=0 ; c<4; c++)
                    Hdb[r][c]=robotPartLinkPose[i](r,c);
yDebug() << robotPartNames[i] << "from\n" << Hdb.toString(4,4) << "\nto\n" << H.toString(4,4);

            fcl::Transform3d pose = fcl::Transform3d::Identity();
            pose.translation() = fcl::Vector3d(H[0][3], H[1][3], H[2][3]);
            fcl::Matrix3d R;
            for(size_t r=0 ; r<3; r++)
                for(size_t c=0 ; c<3; c++)
                    R(r,c)=H[r][c];

            pose.linear()=R;
            pose = pose * robotPartGeometryOffsetPose[i];
            collisionModels[i]->setTransform(pose);
        }

        // Right upper arm
        for(size_t i=21 ; i<26; i++)
        {
            solverRight.fkin(q_right,H,i-21+4);

            Matrix Hdb(4,4);
            for(size_t r=0 ; r<4; r++)
                for(size_t c=0 ; c<4; c++)
                    Hdb[r][c]=robotPartLinkPose[i](r,c);
yDebug() << robotPartNames[i] << "from\n" << Hdb.toString(4,4) << "\nto\n" << H.toString(4,4);

fcl::Transform3d pose = fcl::Transform3d::Identity();
            pose.translation() = fcl::Vector3d(H[0][3], H[1][3], H[2][3]);
            fcl::Matrix3d R;
            for(size_t r=0 ; r<3; r++)
                for(size_t c=0 ; c<3; c++)
                    R(r,c)=H[r][c];

            pose.linear()=R;
            pose = pose * robotPartGeometryOffsetPose[i];
            collisionModels[i]->setTransform(pose);
        }

        // Right hand
        for(size_t i=28 ; i<29; i++)
        {
            solverLeft.fkin(q_left,H,-1);

            Matrix Hdb(4,4);
            for(size_t r=0 ; r<4; r++)
                for(size_t c=0 ; c<4; c++)
                    Hdb[r][c]=robotPartLinkPose[i](r,c);
yDebug() << robotPartNames[i] << "from\n" << Hdb.toString(4,4) << "\nto\n" << H.toString(4,4);

            fcl::Transform3d pose = fcl::Transform3d::Identity();
            pose.translation() = fcl::Vector3d(H[0][3], H[1][3], H[2][3]);
            fcl::Matrix3d R;
            for(size_t r=0 ; r<3; r++)
                for(size_t c=0 ; c<3; c++)
                    R(r,c)=H[r][c];

            pose.linear()=R;
            pose = pose * robotPartGeometryOffsetPose[i];
            collisionModels[i]->setTransform(pose);
        }
    }

    /****************************************************************/
    Vector getEncoders(double *timeStamp=NULL)
    {
        Vector encs(33,0.0);
        Vector stamps(encs.length());

        Vector encs_=encs;
        Vector stamps_=stamps;

        // Base (TODO)

        encs.setSubvector(0, Vector(3,0.0));
        stamps.setSubvector(0, Vector(3,0.0));

        // Joints

        // Torso tripod
        ienc[0]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs.setSubvector(3,encs_.subVector(0,2));
        stamps.setSubvector(3,stamps_.subVector(0,2));

        // Torso yaw
        ienc[1]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs[6]=encs_[3];
        stamps[6]=stamps_[3];

        // Right upper arm
        ienc[2]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs.setSubvector(7,encs_.subVector(0,4));
        stamps.setSubvector(7,stamps_.subVector(0,4));

        // Right arm tripod
        ienc[3]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs.setSubvector(12,encs_.subVector(0,2));
        stamps.setSubvector(12,stamps_.subVector(0,2));

        // Right hand fingers
        ienc[4]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs.setSubvector(15,encs_.subVector(0,3));
        stamps.setSubvector(15,stamps_.subVector(0,3));

        // Left upper arm
        ienc[5]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs.setSubvector(19,encs_.subVector(0,4));
        stamps.setSubvector(19,stamps_.subVector(0,4));

        // Left arm tripod
        ienc[6]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs.setSubvector(24,encs_.subVector(0,2));
        stamps.setSubvector(24,stamps_.subVector(0,2));

        // Left hand fingers
        ienc[7]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs.setSubvector(27,encs_.subVector(0,3));
        stamps.setSubvector(27,stamps_.subVector(0,3));

        // Head
        ienc[8]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs.setSubvector(31,encs_.subVector(0,1));
        stamps.setSubvector(31,stamps_.subVector(0,1));

        if (timeStamp!=NULL)
            *timeStamp=findMax(stamps);
yDebug() << "get encoders" << encs.toString();
        return encs;
    }

    /****************************************************************/
    bool checkInnerCollision()
    {
        if(!collisionBody || !collisionRightArm || !collisionLeftArm)
        {
            yError() << __FUNCTION__ << ": Collision managers are not initialized";

            for(int i=0; i<collisionModels.size(); i++)
            {
                for(int j=0; j<collisionModels.size(); j++)
                {
                    fcl::CollisionRequestd requestC;
                    fcl::CollisionResultd resultC;
                    fcl::DistanceRequestd requestD;
                    fcl::DistanceResultd resultD;

                    fcl::collide(collisionModels[i], collisionModels[j], requestC, resultC);
                    yDebug() << robotPartNames[i] << " / " << robotPartNames[j];
                    yDebug() << (resultC.isCollision()?"yes":"no");
                    yDebug() << fcl::distance(collisionModels[i], collisionModels[j], requestD, resultD);
                    yDebug() << resultD.min_distance;
                }
            }
            return false;
        }

        fcl::CollisionData<double> collision_data1;
        collisionBody->collide(collisionRightArm, &collision_data1, fcl::defaultCollisionFunction);
        yInfo() << "Body / right arm collision:" << (collision_data1.result.isCollision()?"yes":"no") << collision_data1.result.getContact(0).penetration_depth;

        fcl::CollisionData<double> collision_data2;
        collisionBody->collide(collisionLeftArm, &collision_data2, fcl::defaultCollisionFunction);
        yInfo() << "Body / left arm collision:" << (collision_data2.result.isCollision()?"yes":"no") << (collision_data2.result.isCollision()?collision_data2.result.getContact(0).pos[0]:0) << (collision_data2.result.isCollision()?collision_data2.result.getContact(0).pos[1]:0) << (collision_data2.result.isCollision()?collision_data2.result.getContact(0).pos[2]:0);

        fcl::CollisionData<double> collision_data3;
        collisionRightArm->collide(collisionLeftArm, &collision_data3, fcl::defaultCollisionFunction);
        yInfo() << "Right arm / left arm collision:" << (collision_data3.result.isCollision()?"yes":"no");

        fcl::DistanceData<double> distance_data1;
        collisionBody->distance(collisionRightArm, &distance_data1, fcl::defaultDistanceFunction);
        yInfo() << "Body / right arm distance:" << distance_data1.result.min_distance;

        fcl::DistanceData<double> distance_data2;
        collisionBody->distance(collisionLeftArm, &distance_data2, fcl::defaultDistanceFunction);
        yInfo() << "Body / left arm distance:" << distance_data2.result.min_distance;

        fcl::DistanceData<double> distance_data3;
        collisionRightArm->distance(collisionLeftArm, &distance_data3, fcl::defaultDistanceFunction);
        yInfo() << "Right arm / left arm distance:" << distance_data3.result.min_distance;

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

        if(cmd.get(0).asString() == "load_model")
        {
            if(cmd.size() < 2)
            {
                yError() << "Missing path to mesh file for load_model";
                reply.addVocab(Vocab::encode("nack"));
                return false;
            }
            std::string path = cmd.get(1).asString();

            double scale = 1.0;

            if(cmd.size() > 2)
                scale = cmd.get(2).asDouble();

            if(loadMeshModel(path, Vector(3,scale)))
                reply.addVocab(Vocab::encode("ack"));
            else
                reply.addVocab(Vocab::encode("nack"));
        }
        else if(cmd.get(0).asString() == "load_all")
        {
            if(cmd.size() < 2)
            {
                yError() << "Missing path to mesh list file for load_all";
                reply.addVocab(Vocab::encode("nack"));
                return false;
            }
            std::string path = cmd.get(1).asString();
            if(loadAll(path))
                reply.addVocab(Vocab::encode("ack"));
            else
                reply.addVocab(Vocab::encode("nack"));
        }
        else if(cmd.get(0).asString() == "check_inner_collision")
        {
            checkInnerCollision();
            reply.addVocab(Vocab::encode("ack"));
        }
        else if(cmd.get(0).asString() == "load_sdf")
        {
            if(cmd.size() < 2)
            {
                yError() << "Missing path to sdf file for load_sdf";
                reply.addVocab(Vocab::encode("nack"));
                return false;
            }
            std::string path = cmd.get(1).asString();
            if(loadSDF(path))
                reply.addVocab(Vocab::encode("ack"));
            else
                reply.addVocab(Vocab::encode("nack"));
        }
        else if(cmd.get(0).asString() == "update_mesh")
        {
            Vector q=getEncoders();

            if(updateMeshPosition(q))
                reply.addVocab(Vocab::encode("ack"));
            else
                reply.addVocab(Vocab::encode("nack"));
        }
        else if(cmd.get(0).asString() == "test")
        {
            //loadMeshModel("/home/jchevrie/Libraries/assimp/repository/test/models/STL/triangle_with_two_solids.stl", Vector(3,1));
            //loadMeshModel("/home/jchevrie/Libraries/assimp/repository/test/models/STL/triangle.stl", Vector(3,2));
            loadMeshModel("/home/jchevrie/Libraries/cer-sim/repository/gazebo/cer/meshes/dae/sim_cer_chest.dae", Vector(3,0.001));
            loadMeshModel("/home/jchevrie/Libraries/cer-sim/repository/gazebo/cer/meshes/dae/sim_cer_head.dae", Vector(3,0.001));

            //std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> b0((fcl::BVHModel<fcl::OBBRSSd>*)(robotPartGeometry[0].get()));
            //std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> b1((fcl::BVHModel<fcl::OBBRSSd>*)(robotPartGeometry[1].get()));
yDebug() << robotPartGeometry.size();
            fcl::CollisionRequestd requestC;
            fcl::CollisionResultd resultC;
            fcl::DistanceRequestd requestD;
            fcl::DistanceResultd resultD;


            fcl::Transform3d t = fcl::Transform3d::Identity();
            t.fromPositionOrientationScale(
                        fcl::Vector3d(0,0,0.1),
                        fcl::AngleAxisd(0, fcl::Vector3d(1,0,0)),
                        fcl::Vector3d(1,1,1));


            fcl::CollisionObjectd* o0 = new fcl::CollisionObjectd(robotPartGeometry[0], fcl::Transform3d::Identity());
            fcl::CollisionObjectd* o1 = new fcl::CollisionObjectd(robotPartGeometry[1], t);

            fcl::collide(o0, o1, requestC, resultC);
            yDebug() << (resultC.isCollision()?"yes":"no");
            yDebug() << fcl::distance(o0, o1, requestD, resultD);
            yDebug() << resultD.min_distance;

            reply.addVocab(Vocab::encode("ack"));
        }
        else if(cmd.get(0).asString() == "help")
        {
            reply.addVocab(Vocab::encode("nack"));
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

    CollisionDetector module;
    return module.runModule(rf);
}

