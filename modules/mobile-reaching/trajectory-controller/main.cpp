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
#include <vector>
#include <algorithm>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <yarp/dev/ILocalization2D.h>
#include <yarp/dev/IMap2D.h>
#include <yarp/dev/INavigation2D.h>

#include <iCub/ctrl/minJerkCtrl.h>
#include <cer_mobile_kinematics/mobile_arm.h>

#define MIN_TS  0.01    // [s]

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace cer::kinematics;


/****************************************************************/
class Controller : public RFModule
{
    PolyDriver        drivers[7];
    vector<int>       jointsIndexes[4];
    IControlMode*     imod[4];
    IEncodersTimed*   ienc[4];
    IPositionControl* ipos[4];
    IPositionDirect*  iposd[4];
    ILocalization2D*  iloc;
    IMap2D*           imap;
    INavigation2D*    inav;

    vector<int> posDirectMode;
    vector<int> curMode;

    MobileArmSolver solver;
    Bottle lastPlannedTrajectory;
    minJerkTrajGen* gen;
    
    BufferedPort<Vector> statePort;
    BufferedPort<Vector> manipPort;
    RpcServer rpcPort;
    RpcClient solverPort;
    Stamp txInfo;

    Mutex mutex;
    int verbosity;
    bool closing;
    int controlling;
    double stop_threshold_revolute;
    double stop_threshold_prismatic;
    double Ts;

    vector<Vector> qd;
    Map2DLocation bd;

    string mapName;

    /****************************************************************/
    Vector getEncoders(double *timeStamp=NULL)
    {
        Vector encs(12,0.0);
        Vector stamps(encs.length());

        Vector encs_=encs;
        Vector stamps_=stamps;

        ienc[0]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs.setSubvector(0,encs_.subVector(0,2));
        stamps.setSubvector(0,stamps_.subVector(0,2));

        ienc[1]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs[3]=encs_[3];
        stamps[3]=stamps_[3];

        ienc[2]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs.setSubvector(4,encs_.subVector(0,4));
        stamps.setSubvector(4,stamps_.subVector(0,4));

        ienc[3]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs.setSubvector(9,encs_.subVector(0,2));
        stamps.setSubvector(9,stamps_.subVector(0,2));

        if (timeStamp!=NULL)
            *timeStamp=findMax(stamps);

        return encs;
    }

    /****************************************************************/
    void getCurrentMode()
    {
        imod[0]->getControlModes((int)jointsIndexes[0].size(),jointsIndexes[0].data(),&curMode[0]);
        imod[1]->getControlModes((int)jointsIndexes[1].size(),jointsIndexes[1].data(),&curMode[3]);
        imod[2]->getControlModes((int)jointsIndexes[2].size(),jointsIndexes[2].data(),&curMode[4]);
        imod[3]->getControlModes((int)jointsIndexes[3].size(),jointsIndexes[3].data(),&curMode[9]);
    }

    /****************************************************************/
    bool areJointsHealthy()
    {
        for (size_t i=0; i<curMode.size(); i++)
            if ((curMode[i]==VOCAB_CM_HW_FAULT) || (curMode[i]==VOCAB_CM_IDLE))
                return false;
        return true;
    }

    /****************************************************************/
    bool setPositionDirectMode()
    {
        for (size_t i=0; i<3; i++)
        {
            if (curMode[i]!=posDirectMode[i])
            {
                imod[0]->setControlModes((int)jointsIndexes[0].size(),jointsIndexes[0].data(),&posDirectMode[0]);
                break;
            }
        }

        for (size_t i=3; i<4; i++)
        {
            if (curMode[i]!=posDirectMode[i])
            {
                imod[1]->setControlModes((int)jointsIndexes[1].size(),jointsIndexes[1].data(),&posDirectMode[3]);
                break;
            }
        }

        for (size_t i=4; i<9; i++)
        {
            if (curMode[i]!=posDirectMode[i])
            {
                imod[2]->setControlModes((int)jointsIndexes[2].size(),jointsIndexes[2].data(),&posDirectMode[4]);
                break;
            }
        }

        for (size_t i=9; i<curMode.size(); i++)
        {
            if (curMode[i]!=posDirectMode[i])
            {
                imod[3]->setControlModes((int)jointsIndexes[3].size(),jointsIndexes[3].data(),&posDirectMode[9]);
                break;
            }
        }

        return true;
    }

    /****************************************************************/
    void stopControl()
    {
        for (int i=0; i<4; i++)
            ipos[i]->stop((int)jointsIndexes[i].size(),jointsIndexes[i].data());
        controlling=-1;

        inav->stopNavigation();
    }

    /****************************************************************/
    bool dist(const Vector &e)
    {
        Vector e_revolute=e.subVector(3,8);
        Vector e_prismatic=cat(e.subVector(0,2),e.subVector(9,11));
        return (norm(e_revolute)<stop_threshold_revolute) &&
               (norm(e_prismatic)<stop_threshold_prismatic);
    }

    /****************************************************************/
    Property prepareSolverOptions(const string &key, const Value &val)
    {
        Bottle b;
        Bottle &payLoad=b.addList().addList();
        payLoad.addString(key);
        payLoad.add(val);

        Property option;
        option.put("parameters",b.get(0));

        return option;
    }

    /****************************************************************/
    Value parseSolverOptions(const Bottle &rep, const string &key)
    {
        Value val;
        if (Bottle *payLoad=rep.find("parameters").asList())
            val=payLoad->find(key);

        return val;
    }

public:
    /****************************************************************/
    Controller() : gen(NULL), closing(false), controlling(-1)
    {
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("cer")).asString();
        string arm_type=rf.check("arm-type",Value("left")).asString();
        verbosity=rf.check("verbosity",Value(0)).asInt();
        stop_threshold_revolute=rf.check("stop-threshold-revolute",Value(2.0)).asDouble();
        stop_threshold_prismatic=rf.check("stop-threshold-prismatic",Value(0.002)).asDouble();
        string map_server=rf.check("map-server",Value("/mapServer")).asString();
        string loc_server=rf.check("loc-server",Value("/localizationServer")).asString();
        string nav_server=rf.check("nav-server",Value("/navigationServer")).asString();
        mapName=rf.check("map-name",Value("testMap")).asString();
        double T=rf.check("T",Value(2.0)).asDouble();
        Ts=rf.check("Ts",Value(MIN_TS)).asDouble();
        Ts=std::max(Ts,MIN_TS);

        Property option;

        option.put("device","remote_controlboard");
        option.put("remote","/"+robot+"/torso_tripod");
        option.put("local","/cer_mobile-reaching-trajectory-controller/"+arm_type+"/torso_tripod");
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
        option.put("local","/cer_mobile-reaching-trajectory-controller/"+arm_type+"/torso");
        option.put("writeStrict","on");
        if (!drivers[1].open(option))
        {
            yError("Unable to connect to %s",string("/"+robot+"/torso").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote","/"+robot+"/"+arm_type+"_arm");
        option.put("local","/cer_mobile-reaching-trajectory-controller/"+arm_type+"/"+arm_type+"_arm");
        option.put("writeStrict","on");
        if (!drivers[2].open(option))
        {
            yError("Unable to connect to %s",string("/"+robot+"/"+arm_type+"_arm").c_str());
            close();
            return false;
        }

        option.clear();
        option.put("device","remote_controlboard");
        option.put("remote","/"+robot+"/"+arm_type+"_wrist_tripod");
        option.put("local","/cer_mobile-reaching-trajectory-controller/"+arm_type+"/"+arm_type+"_wrist_tripod");
        option.put("writeStrict","on");
        if (!drivers[3].open(option))
        {
            yError("Unable to connect to %s",string("/"+robot+"/"+arm_type+"_wrist_tripod").c_str());
            close();
            return false;
        }

        for (int i=0; i<4; i++)
        {
            drivers[i].view(imod[i]);
            drivers[i].view(ienc[i]);
            drivers[i].view(ipos[i]);
            drivers[i].view(iposd[i]);
        }

        // torso_tripod
        jointsIndexes[0].push_back(0);
        jointsIndexes[0].push_back(1);
        jointsIndexes[0].push_back(2);

        // torso (yaw)
        jointsIndexes[1].push_back(3);

        // arm
        jointsIndexes[2].push_back(0);
        jointsIndexes[2].push_back(1);
        jointsIndexes[2].push_back(2);
        jointsIndexes[2].push_back(3);
        jointsIndexes[2].push_back(4);

        // wrist_tripod
        jointsIndexes[3].push_back(0);
        jointsIndexes[3].push_back(1);
        jointsIndexes[3].push_back(2);

        // localization
        option.clear();
        option.put("device","localization2DClient");
        option.put("remote",loc_server);
        option.put("local","/cer_mobile-reaching-trajectory-controller/"+arm_type);
        if (!drivers[4].open(option))
        {
            yError("Unable to connect to %s",(loc_server+"/rpc").c_str());
            close();
            return false;
        }
        if (!drivers[4].view(iloc))
        {
            yError() << "Unable to open ILocalization2D interface";
            return false;
        }

        // map
        option.clear();
        option.put("device","map2DClient");
        option.put("remote",map_server);
        option.put("local","/cer_mobile-reaching-trajectory-controller/"+arm_type);
        if (!drivers[5].open(option))
        {
            yError("Unable to connect to %s",(map_server+"/rpc").c_str());
            close();
            return false;
        }
        if (!drivers[5].view(imap))
        {
            yError() << "Unable to open IMap2D interface";
            return false;
        }

        // navigation
        option.clear();
        option.put("device","navigation2DClient");
        option.put("local","/cer_mobile-reaching-trajectory-controller/"+arm_type+"navigation");
        option.put("navigation_server",nav_server);
        option.put("map_locations_server",map_server);
        option.put("localization_server",loc_server);
        if (!drivers[6].open(option))
        {
            yError("Unable to connect to %s",(nav_server+"/rpc").c_str());
            close();
            return false;
        }
        if (!drivers[6].view(inav))
        {
            yError() << "Unable to open INavigation2D interface";
            return false;
        }
        inav->stopNavigation();
        yarp::os::Time::delay(0.1);

        //
        statePort.open("/cer_mobile-reaching-trajectory-controller/"+arm_type+"/state:o");
        manipPort.open("/cer_mobile-reaching-trajectory-controller/"+arm_type+"/manip:o");
        solverPort.open("/cer_mobile-reaching-trajectory-controller/"+arm_type+"/solver:rpc");

        rpcPort.open("/cer_mobile-reaching-trajectory-controller/"+arm_type+"/rpc");
        attach(rpcPort);

        qd.resize(1);
        qd[0]=getEncoders();
        for (size_t i=0; i<qd[0].length(); i++)
            posDirectMode.push_back(VOCAB_CM_POSITION_DIRECT);
        curMode=posDirectMode;

        getCurrentMode();
        
        ArmParameters arm(arm_type);
        arm.upper_arm.setAllConstraints(false);
        solver.setArmParameters(arm);

        gen=new minJerkTrajGen(qd[0],Ts,T);

        return true;
    }

    /****************************************************************/
    bool close()
    {
        closing=true;

        if (controlling!=-1)
            stopControl();

        if (!manipPort.isClosed())
            manipPort.close();

        if (!statePort.isClosed())
            statePort.close();

        if (!solverPort.asPort().isOpen())
            solverPort.close();

        if (!rpcPort.asPort().isOpen())
            rpcPort.close();
                
        for (int i=0; i<6; i++)
            if (drivers[i].isValid())
                drivers[i].close(); 

        delete gen;
        return true;
    }

    /****************************************************************/
    double getPeriod()
    {
        return Ts;
    }

    /****************************************************************/
    bool go(Bottle &trajectory)
    {
        if (closing)
            return false;

        if (verbosity>0)
            yInfo("Received [go] request: %s",trajectory.toString().c_str());

        if (trajectory.size()>1)
        {
            Bottle *payLoadJointsList=trajectory.find("q").asList();

            if ((payLoadJointsList!=NULL))
            {
                Bottle *payLoadJoints=payLoadJointsList->get(0).asList();

                LockGuard lg(mutex);
                Map2DLocation p;
                iloc->getCurrentPosition(p);

                bd.x=payLoadJoints->get(0).asDouble();
                bd.y=payLoadJoints->get(1).asDouble();
                bd.theta=payLoadJoints->get(2).asDouble();
                bd.map_id=p.map_id;

                NavigationStatusEnum navStatus;
                inav->getNavigationStatus(navStatus);
                if (navStatus!=navigation_status_idle)
                    inav->stopNavigation();

                if (!inav->gotoTargetByAbsoluteLocation(bd))
                {
                    yError("Cannot navigate to destination!");
                    return false;
                }

                return true;
            }
        }

        return false;
    }

    /****************************************************************/
    bool goLast()
    {
        if (lastPlannedTrajectory.size()>0)
            return go(lastPlannedTrajectory);
        else
            return false;
    }

    /****************************************************************/
    bool ask(Property &request, Bottle &reply, bool localIn=false, bool localOut=false)
    {
        if (closing)
            return false;

        if (verbosity>0)
            yInfo("Received [ask] request: %s",request.toString().c_str());

        if (!request.check("q"))
        {
            Vector qj=getEncoders();

            Vector q(3+qj.size(), 0.0);
            q.setSubvector(3,qj);

            if (iloc)
            {
                Map2DLocation p;
                iloc->getCurrentPosition(p);
                q[0]=p.x;
                q[1]=p.y;
                q[2]=p.theta;
            }

            Bottle b; b.addList().read(q);
            request.put("q",b.get(0));
        }

        Bottle *robotPose = request.find("q").asList();
        Vector tu(4,0.0);
        tu[2]=1.0;
        tu[3]=M_PI/180.0*robotPose->get(2).asDouble();
        Matrix baseTransform=axis2dcm(tu);
        baseTransform[0][3]=robotPose->get(0).asDouble();
        baseTransform[1][3]=robotPose->get(1).asDouble();

        if (!request.check("domain") && iloc && imap)
        {
            Map2DLocation loc;
            iloc->getCurrentPosition(loc);
            vector<string> areaNames;
            imap->getAreasList(areaNames);

            if (verbosity>0)
            {
                yDebug() << "Area list:";
                for(int i=0 ; i<areaNames.size() ; i++)
                    yDebug() << "\t" << areaNames[i];
            }

            Map2DArea area;
            bool in_known_area=false;
            for(size_t i=0 ; i<areaNames.size() ; i++)
            {
                imap->getArea(areaNames[i], area);
                if (area.checkLocationInsideArea(loc))
                {
                    if (verbosity>0)
                        yDebug() << "Currently in" << areaNames[i];
                    in_known_area=true;
                    break;
                }
            }

            Bottle b;
            Bottle &coord = b.addList();

            if (in_known_area)
            {
                if (verbosity>0)
                    yDebug() << "Area points list:";
                for(size_t i=0 ; i<area.points.size() ; i++)
                {
                    if (verbosity>0)
                        yDebug() << "\t" << area.points[i].x << area.points[i].y;
                    area.points.size();
                    coord.addDouble(area.points[i].x);
                    coord.addDouble(area.points[i].y);
                }
            }
            request.put("domain",b.get(0));
        }

        if (localIn)
        {
            if (!convertRequestFromLocalToMap(request, baseTransform))
            {
                reply.clear();
                reply.addVocab(Vocab::encode("nack"));
                return false;
            }
        }

        if (verbosity>0)
            yInfo("Forwarding request to solver: %s",request.toString().c_str());

        reply.clear();
        if (solverPort.write(request,reply))
        {
            if (verbosity>0)
                yInfo("Received reply from solver: %s",reply.toString().c_str());

            bool success = (reply.get(0).asVocab()==Vocab::encode("ack"));
            if (success || (reply.get(0).asVocab()==Vocab::encode("nack")))
            {
                reply=reply.tail();

                if (localOut)
                {
                    if (!convertReplyFromMapToLocal(reply, baseTransform))
                    {
                        yError("Could not process solver reply");
                        return false;
                    }
                }

                return success;
            }
            else
                yError("Invalid reply from solver!");
        }
        else
            yError("Unable to communicate with the solver");

        return false;
    }

    /****************************************************************/
    bool plan(Property &request, Bottle &reply, bool local=false)
    {
        if (verbosity>0)
            yInfo("Received [plan] request: %s",request.toString().c_str());

        if (!ask(request, reply, local, false))
        {
            yError() << "Trajectory planning failed";
            return false;
        }

        this->stopControl();
        lastPlannedTrajectory=reply;
        return true;
    }

    /****************************************************************/
    bool execLast()
    {
        if (verbosity>0)
            yInfo("Received [execLast] request");

        if (lastPlannedTrajectory.size()>0)
            return exec(lastPlannedTrajectory);
        else
            return false;
    }

    /****************************************************************/
    bool exec(Bottle &trajectory)
    {
        if (closing)
            return false;

        if (verbosity>0)
            yInfo("Received [exec] request: %s",trajectory.toString().c_str());

        if (trajectory.size()>1)
        {
            Bottle *payLoadJointsList=trajectory.find("q").asList();

            if ((payLoadJointsList!=NULL))
            {
                LockGuard lg(mutex);

                Vector qref=getEncoders();
                qd.resize(payLoadJointsList->size());
                for (size_t i=0; i<payLoadJointsList->size(); i++)
                {
                    Bottle *payLoadJoints=payLoadJointsList->get(i).asList();

                    qd[i].resize(qref.size(),0.0);
                    for (size_t j=0; j<qd[i].length(); j++)
                        qd[i][j]=payLoadJoints->get(3+j).asDouble();
                }

                if (controlling==-1)
                    gen->init(getEncoders());

                controlling=0;
                if (verbosity>0)
                    yInfo("Going to: %s",qd[controlling].toString(3,3).c_str());

                return true;
            }
        }

        return false;
    }

    /****************************************************************/
    bool execStep(int step)
    {
    }

    /****************************************************************/
    bool convertRequestFromLocalToMap(Property &request, const Matrix &torsoTransform)
    {
        if (Bottle *targetList=request.find("target").asList())
        {
            for (size_t i=0; i<targetList->size(); i++)
            {
                Bottle *target=targetList->get(i).asList();
                if (!target)
                {
                    yError("convertRequestFromLocalToMap: wrong target list format!");
                    return false;
                }

                if (target->size()<7)
                {
                    yError("convertRequestFromLocalToMap: wrong target size, must be 7!");
                    return false;
                }

                Vector xd(3);
                xd[0]=target->get(0).asDouble();
                xd[1]=target->get(1).asDouble();
                xd[2]=target->get(2).asDouble();
                Vector ud(4);
                ud[0]=target->get(3).asDouble();
                ud[1]=target->get(4).asDouble();
                ud[2]=target->get(5).asDouble();
                ud[3]=target->get(6).asDouble();

                Matrix Hd = axis2dcm(ud);
                Hd.setSubcol(xd,0,3);
                Hd = torsoTransform*Hd;

                Vector x = Hd.subcol(0, 3, 3);
                Vector u = dcm2axis(Hd);
                target->clear();
                target->addDouble(x[0]);
                target->addDouble(x[1]);
                target->addDouble(x[2]);
                target->addDouble(u[0]);
                target->addDouble(u[1]);
                target->addDouble(u[2]);
                target->addDouble(u[3]);
                if (verbosity>0)
                {
                    yDebug() << "Target transformed from local:";
                    yDebug() << "\t" << xd.toString() << ud.toString();
                    yDebug() << "                    to global:";
                    yDebug() << "\t" << x.toString() << u.toString();
                }
            }
        }

        return true;
    }

    /****************************************************************/
    bool convertReplyFromMapToLocal(Bottle &reply, const Matrix &torsoTransform)
    {
        if (Bottle *replyPoseList = reply.find("q").asList())
        {
            for (int i=0 ; i<replyPoseList->size(); i++)
            {
                Bottle *replyPose = replyPoseList->get(i).asList();

                if (verbosity>0)
                {
                    yDebug() << "Final base pose transformed from global:";
                    yDebug() << "\t" << replyPose->get(0).asDouble() << replyPose->get(1).asDouble() << replyPose->get(2).asDouble();
                }
                Vector new_tu(4,0.0);
                new_tu[2]=1.0;
                new_tu[3]=M_PI/180.0*replyPose->get(2).asDouble();
                Matrix newTorsoTransform=axis2dcm(new_tu);
                newTorsoTransform[0][3]=replyPose->get(0).asDouble();
                newTorsoTransform[1][3]=replyPose->get(1).asDouble();
                newTorsoTransform=SE3inv(torsoTransform)*newTorsoTransform;
                replyPose->get(0)=Value(newTorsoTransform[0][3]);
                replyPose->get(1)=Value(newTorsoTransform[1][3]);
                new_tu=dcm2axis(newTorsoTransform);
                replyPose->get(2)=Value(180.0/M_PI*new_tu[2]*new_tu[3]);

                if (verbosity>0)
                {
                    yDebug() << "                              to local:";
                    yDebug() << "\t" << replyPose->get(0).asDouble() << replyPose->get(1).asDouble() << replyPose->get(2).asDouble();
                }
            }
        }
        else
        {
            yError() << "convertReplyFromMapToLocal: missing joint list";
            return false;
        }

        if (Bottle *targetList=reply.find("x").asList())
        {
            for (int i=0 ; i<targetList->size(); i++)
            {
                Bottle *target=targetList->get(i).asList();
                if (!target)
                {
                    yError("convertReplyFromMapToLocal: wrong target list format!");
                    reply.clear();
                    reply.addVocab(Vocab::encode("nack"));
                    return false;
                }

                if (target->size()<7)
                {
                    yError("convertReplyFromMapToLocal: wrong target size!");
                    reply.clear();
                    reply.addVocab(Vocab::encode("nack"));
                    return false;
                }

                Vector xd(3);
                xd[0]=target->get(0).asDouble();
                xd[1]=target->get(1).asDouble();
                xd[2]=target->get(2).asDouble();
                Vector ud(4);
                ud[0]=target->get(3).asDouble();
                ud[1]=target->get(4).asDouble();
                ud[2]=target->get(5).asDouble();
                ud[3]=target->get(6).asDouble();

                Matrix Hd = axis2dcm(ud);
                Hd.setSubcol(xd,0,3);
                Hd = SE3inv(torsoTransform)*Hd;

                Vector x = Hd.subcol(0, 3, 3);
                Vector u = dcm2axis(Hd);
                target->clear();
                target->addDouble(x[0]);
                target->addDouble(x[1]);
                target->addDouble(x[2]);
                target->addDouble(u[0]);
                target->addDouble(u[1]);
                target->addDouble(u[2]);
                target->addDouble(u[3]);
                if (verbosity>0)
                {
                    yDebug() << "Target transformed from global:";
                    yDebug() << "\t" << xd.toString() << ud.toString();
                    yDebug() << "                      to local:";
                    yDebug() << "\t" << x.toString() << u.toString();
                }
            }
        }
        else
        {
            yError() << "convertReplyFromMapToLocal: missing end effector pose list";
            return false;
        }

        return true;
    }

    /****************************************************************/
    bool updateModule()
    {
        LockGuard lg(mutex);
        getCurrentMode();        

        Matrix Hee;
        double timeStamp;
        Vector qj=getEncoders(&timeStamp);
        Vector q(3+qj.size(), 0.0);
        q.setSubvector(3,qj);
        solver.fkin(q,Hee);

        Vector &pose=statePort.prepare();
        pose=Hee.getCol(3).subVector(0,2);

        Vector oee=dcm2axis(Hee);
        pose=cat(pose,oee);

        if (timeStamp>=0.0)
            txInfo.update(timeStamp);
        else
            txInfo.update();

        statePort.setEnvelope(txInfo);
        statePort.writeStrict();

        Vector &manip=manipPort.prepare();
        solver.getManip(q,manip);
        manipPort.writeStrict();

        if (controlling!=-1)
        {
            gen->computeNextValues(qd[controlling]);
            Vector ref=gen->getPos();

            if (verbosity>1)
                yInfo("Commanding new set-points: %s",ref.toString(3,3).c_str());

            if (areJointsHealthy())
            {
                setPositionDirectMode(); 
                iposd[0]->setPositions((int)jointsIndexes[0].size(),jointsIndexes[0].data(),&ref[0]);
                iposd[1]->setPositions((int)jointsIndexes[1].size(),jointsIndexes[1].data(),&ref[3]);
                iposd[2]->setPositions((int)jointsIndexes[2].size(),jointsIndexes[2].data(),&ref[4]);
                iposd[3]->setPositions((int)jointsIndexes[3].size(),jointsIndexes[3].data(),&ref[9]);

                if (dist(qd[controlling]-q.subVector(3,q.size()-1)))
                {
                    if (controlling+1<qd.size())
                    {
                        controlling++;
                        if (verbosity>0)
                            yInfo("Just reached point at: %s",q.toString(3,3).c_str());
                    }
                    else
                    {
                        controlling=-1;
                        if (verbosity>0)
                            yInfo("Just stopped point at: %s",q.toString(3,3).c_str());
                    }
                }
            }
            else
            {
                yWarning("Detected joints in HW_FAULT and/or IDLE => stopping control");
                stopControl();                
            }
        }

        return true;
    }

    /****************************************************************/
    bool respond(const Bottle &cmd, Bottle &reply)
    {
        string cmd_0=cmd.get(0).asString();
        if (cmd.size()==3)
        {
            if (cmd_0=="set")
            {
                string cmd_1=cmd.get(1).asString();
                if (cmd_1=="T")
                {
                    mutex.lock();
                    gen->setT(cmd.get(2).asDouble());
                    mutex.unlock();

                    reply.addVocab(Vocab::encode("ack"));
                }
                else if (cmd_1=="Ts")
                {
                    Ts=cmd.get(2).asDouble();
                    Ts=std::max(Ts,MIN_TS);

                    mutex.lock();
                    gen->setTs(Ts);
                    mutex.unlock();

                    reply.addVocab(Vocab::encode("ack"));
                }
                else if (cmd_1=="verbosity")
                {
                    mutex.lock();
                    verbosity=cmd.get(2).asInt();
                    mutex.unlock();

                    reply.addVocab(Vocab::encode("ack"));
                }
                else if (cmd_1=="mode")
                {
                    Value mode(cmd.get(2).asString());
                    Property p=prepareSolverOptions("mode",mode);

                    Bottle b;
                    if (ask(p,b))
                        reply.addVocab(Vocab::encode("ack"));
                }
                else if (cmd_1=="torso_heave")
                {
                    Value torso_heave(cmd.get(2).asDouble());
                    Property p=prepareSolverOptions("torso_heave",torso_heave);

                    Bottle b;
                    if (ask(p,b))
                        reply.addVocab(Vocab::encode("ack"));
                }
                else if (cmd_1=="lower_arm_heave")
                {
                    Value lower_arm_heave(cmd.get(2).asDouble());
                    Property p=prepareSolverOptions("lower_arm_heave",lower_arm_heave);

                    Bottle b;
                    if (ask(p,b))
                        reply.addVocab(Vocab::encode("ack"));
                }
                else if (cmd_1=="tol")
                {
                    Value tol(cmd.get(2).asDouble());
                    Property p=prepareSolverOptions("tol",tol);

                    Bottle b;
                    if (ask(p,b))
                        reply.addVocab(Vocab::encode("ack"));
                }
                else if (cmd_1=="constr_tol")
                {
                    Value constr_tol(cmd.get(2).asDouble());
                    Property p=prepareSolverOptions("constr_tol",constr_tol);

                    Bottle b;
                    if (ask(p,b))
                        reply.addVocab(Vocab::encode("ack"));
                }
                else if (cmd_1=="nb_check_points")
                {
                    Value nb_check_points(cmd.get(2).asDouble());
                    Property p=prepareSolverOptions("nb_check_points",nb_check_points);

                    Bottle b;
                    if (ask(p,b))
                        reply.addVocab(Vocab::encode("ack"));
                }
                else if (cmd_1=="out_domain_height")
                {
                    Value out_domain_height(cmd.get(2).asDouble());
                    Property p=prepareSolverOptions("out_domain_height",out_domain_height);

                    Bottle b;
                    if (ask(p,b))
                        reply.addVocab(Vocab::encode("ack"));
                }
                else if (cmd_1=="obstacle_margin")
                {
                    Value obstacle_margin(cmd.get(2).asDouble());
                    Property p=prepareSolverOptions("obstacle_margin",obstacle_margin);

                    Bottle b;
                    if (ask(p,b))
                        reply.addVocab(Vocab::encode("ack"));
                }
            }
        }
        else if (cmd.size()==2)
        {
            if (cmd_0=="get")
            {
                string cmd_1=cmd.get(1).asString();
                if (cmd_1=="done")
                {
                    reply.addVocab(Vocab::encode("ack"));

                    mutex.lock();
                    reply.addInt((controlling==-1)?1:0);
                    mutex.unlock();
                }
                else if (cmd_1=="T")
                {
                    reply.addVocab(Vocab::encode("ack"));

                    mutex.lock();
                    reply.addDouble(gen->getT());
                    mutex.unlock();
                }
                else if (cmd_1=="Ts")
                {
                    reply.addVocab(Vocab::encode("ack"));

                    mutex.lock();
                    reply.addDouble(Ts);
                    mutex.unlock();
                }
                else if (cmd_1=="verbosity")
                {
                    reply.addVocab(Vocab::encode("ack"));

                    mutex.lock();
                    reply.addInt(verbosity);
                    mutex.unlock();
                }
                else if (cmd_1=="mode")
                {
                    Bottle req,rep;
                    req.addList().addString("get");
                    if (solverPort.write(req,rep))
                    {
                        Value mode=parseSolverOptions(rep,"mode");
                        reply.addVocab(Vocab::encode("ack"));
                        reply.add(mode);
                    }
                }
                else if (cmd_1=="torso_heave")
                {
                    Bottle req,rep;
                    req.addList().addString("get");
                    if (solverPort.write(req,rep))
                    {
                        Value torso_heave=parseSolverOptions(rep,"torso_heave");
                        reply.addVocab(Vocab::encode("ack"));
                        reply.add(torso_heave);
                    }
                }
                else if (cmd_1=="lower_arm_heave")
                {
                    Bottle req,rep;
                    req.addList().addString("get");
                    if (solverPort.write(req,rep))
                    {
                        Value lower_arm_heave=parseSolverOptions(rep,"lower_arm_heave");
                        reply.addVocab(Vocab::encode("ack"));
                        reply.add(lower_arm_heave);
                    }
                }
                else if (cmd_1=="tol")
                {
                    Bottle req,rep;
                    req.addList().addString("get");
                    if (solverPort.write(req,rep))
                    {
                        Value tol=parseSolverOptions(rep,"tol");
                        reply.addVocab(Vocab::encode("ack"));
                        reply.add(tol);
                    }
                }
                else if (cmd_1=="constr_tol")
                {
                    Bottle req,rep;
                    req.addList().addString("get");
                    if (solverPort.write(req,rep))
                    {
                        Value constr_tol=parseSolverOptions(rep,"constr_tol");
                        reply.addVocab(Vocab::encode("ack"));
                        reply.add(constr_tol);
                    }
                }
                else if (cmd_1=="nb_check_points")
                {
                    Bottle req,rep;
                    req.addList().addString("get");
                    if (solverPort.write(req,rep))
                    {
                        Value nb_check_points=parseSolverOptions(rep,"nb_check_points");
                        reply.addVocab(Vocab::encode("ack"));
                        reply.add(nb_check_points);
                    }
                }
                else if (cmd_1=="out_domain_height")
                {
                    Bottle req,rep;
                    req.addList().addString("get");
                    if (solverPort.write(req,rep))
                    {
                        Value out_domain_height=parseSolverOptions(rep,"out_domain_height");
                        reply.addVocab(Vocab::encode("ack"));
                        reply.add(out_domain_height);
                    }
                }
                else if (cmd_1=="obstacle_margin")
                {
                    Bottle req,rep;
                    req.addList().addString("get");
                    if (solverPort.write(req,rep))
                    {
                        Value obstacle_margin=parseSolverOptions(rep,"obstacle_margin");
                        reply.addVocab(Vocab::encode("ack"));
                        reply.add(obstacle_margin);
                    }
                }
            }
            else if (cmd_0=="go")
            {
                if (Bottle *b=cmd.get(1).asList())
                {
                    if (go(*b))
                        reply.addVocab(Vocab::encode("ack"));
                }
            }
            else if (cmd_0=="exec")
            {
                if (Bottle *b=cmd.get(1).asList())
                {
                    if (exec(*b))
                        reply.addVocab(Vocab::encode("ack"));
                }
            }
            else if (cmd_0=="ask")
            {
                if (Bottle *b=cmd.get(1).asList())
                {
                    Property p(b->toString().c_str());
                    Bottle payLoad;
                    if (ask(p,payLoad,false,false))
                    {
                        reply.addVocab(Vocab::encode("ack"));
                        reply.append(payLoad);
                    }
                    else if (payLoad.size()>0)
                    {
                        reply.addVocab(Vocab::encode("nack"));
                        reply.append(payLoad);
                    }
                }
            }
            else if (cmd_0=="askLocal")
            {
                if (Bottle *b=cmd.get(1).asList())
                {
                    Property p(b->toString().c_str());
                    Bottle payLoad;
                    if (ask(p,payLoad,true,true))
                    {
                        reply.addVocab(Vocab::encode("ack"));
                        reply.append(payLoad);
                    }
                    else if (payLoad.size()>0)
                    {
                        reply.addVocab(Vocab::encode("nack"));
                        reply.append(payLoad);
                    }
                }
            }
            else if (cmd_0=="plan")
            {
                if (Bottle *b=cmd.get(1).asList())
                {
                    Property p(b->toString().c_str());
                    Bottle payLoad;
                    if (plan(p,payLoad,false))
                    {
                        reply.addVocab(Vocab::encode("ack"));
                        reply.append(payLoad);
                    }
                    else if (payLoad.size()>0)
                    {
                        reply.addVocab(Vocab::encode("nack"));
                        reply.append(payLoad);
                    }
                }
            }
            else if (cmd_0=="planLocal")
            {
                if (Bottle *b=cmd.get(1).asList())
                {
                    Property p(b->toString().c_str());
                    Bottle payLoad;
                    if (plan(p,payLoad,true))
                    {
                        reply.addVocab(Vocab::encode("ack"));
                        reply.append(payLoad);
                    }
                    else if (payLoad.size()>0)
                    {
                        reply.addVocab(Vocab::encode("nack"));
                        reply.append(payLoad);
                    }
                }
            }
        }
        else if (cmd_0=="goLast")
        {
            if (goLast())
                reply.addVocab(Vocab::encode("ack"));
        }
        else if (cmd_0=="execLast")
        {
            if (execLast())
                reply.addVocab(Vocab::encode("ack"));
        }
        else if (cmd_0=="stop")
        {
            mutex.lock();
            stopControl();
            mutex.unlock();

            reply.addVocab(Vocab::encode("ack"));
        }

        if (reply.size()==0)
            reply.addVocab(Vocab::encode("nack")); 
        
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

    Controller controller;
    return controller.runModule(rf);
}

