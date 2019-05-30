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

// forward declaration
class Controller;

/****************************************************************/
class TargetPort : public BufferedPort<Property>
{
    Controller *ctrl;

    /****************************************************************/
    void onRead(Property &request);

public:
    /****************************************************************/
    TargetPort() : ctrl(NULL) { }

    /****************************************************************/
    void setController(Controller *ctrl) { this->ctrl=ctrl; }
};


/****************************************************************/
class Controller : public RFModule
{
    PolyDriver        drivers[6];
    vector<int>       jointsIndexes[4];
    IControlMode*     imod[4];
    IEncodersTimed*   ienc[4];
    IPositionControl* ipos[4];
    IPositionDirect*  iposd[4];
    ILocalization2D*  iloc;
    IMap2D*           imap;

    vector<int> posDirectMode;
    vector<int> curMode;

    MobileArmSolver solver;
    minJerkTrajGen* gen;
    
    BufferedPort<Vector> statePort;
    TargetPort targetPort;
    RpcServer rpcPort;
    RpcClient solverPort;
    Stamp txInfo;

    Mutex mutex;
    int verbosity;
    bool closing;
    bool controlling;
    double stop_threshold_revolute;
    double stop_threshold_prismatic;
    double Ts;

    Bottle target;
    Vector qd,xd;

    string mapName;

    /****************************************************************/
    Vector getEncoders(double *timeStamp=NULL)
    {

        // TODO include mobile base position

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

        // TODO get mobile base mode
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

        // TODO set mobile base control mode

        return true;
    }

    /****************************************************************/
    void stopControl()
    {        
        for (int i=0; i<4; i++)
            ipos[i]->stop((int)jointsIndexes[i].size(),jointsIndexes[i].data());
        controlling=false;

        // TODO stop navigation if necessary
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
    Controller() : gen(NULL), closing(false), controlling(false)
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
        mapName=rf.check("map-name",Value("testMap")).asString();
        double T=rf.check("T",Value(2.0)).asDouble();
        Ts=rf.check("Ts",Value(MIN_TS)).asDouble();
        Ts=std::max(Ts,MIN_TS);

        Property option;

        option.put("device","remote_controlboard");
        option.put("remote","/"+robot+"/torso_tripod");
        option.put("local","/cer_mobile-reaching-controller/"+arm_type+"/torso_tripod");
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
        option.put("local","/cer_mobile-reaching-controller/"+arm_type+"/torso");
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
        option.put("local","/cer_mobile-reaching-controller/"+arm_type+"/"+arm_type+"_arm");
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
        option.put("local","/cer_mobile-reaching-controller/"+arm_type+"/"+arm_type+"_wrist_tripod");
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
        option.put("local","/cer_mobile-reaching-controller/"+arm_type);
        if (!drivers[4].open(option))
        {
            yError("Unable to connect to %s",(loc_server+"/rpc").c_str());
            close();
            return false;
        }
        drivers[4].view(iloc);

        // map
        option.clear();
        option.put("device","map2DClient");
        option.put("remote",map_server);
        option.put("local","/cer_mobile-reaching-controller/"+arm_type);
        if (!drivers[5].open(option))
        {
            yError("Unable to connect to %s",(map_server+"/rpc").c_str());
            close();
            return false;
        }
        drivers[5].view(imap);

        statePort.open("/cer_mobile-reaching-controller/"+arm_type+"/state:o");
        solverPort.open("/cer_mobile-reaching-controller/"+arm_type+"/solver:rpc");

        targetPort.open("/cer_mobile-reaching-controller/"+arm_type+"/target:i");
        targetPort.setController(this);
        targetPort.useCallback();

        rpcPort.open("/cer_mobile-reaching-controller/"+arm_type+"/rpc");
        attach(rpcPort);

        // prepare target bottle
        Bottle &payLoadJoints=target.addList();
        payLoadJoints.addString("q");
        payLoadJoints.addList();

        Bottle &payLoadPose=target.addList();
        payLoadPose.addString("x");
        payLoadPose.addList();

        qd=getEncoders();
        for (size_t i=0; i<qd.length(); i++)
            posDirectMode.push_back(VOCAB_CM_POSITION_DIRECT);
        curMode=posDirectMode;

        getCurrentMode();
        
        ArmParameters arm(arm_type);
        arm.upper_arm.setAllConstraints(false);
        solver.setArmParameters(arm);

        gen=new minJerkTrajGen(qd,Ts,T);        

        return true;
    }

    /****************************************************************/
    bool close()
    {
        closing=true;

        if (controlling)
            stopControl();

        if (!targetPort.isClosed())
            targetPort.close(); 

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
    bool go(Property &request)
    {
        if (closing)
            return false;

        if (verbosity>0)
            yInfo("Received [go] request: %s",request.toString().c_str());

        if (request.check("stop"))
        {
            LockGuard lg(mutex);
            stopControl();
            return true;
        }

        if (!request.check("q"))
        {
            Vector qj=getEncoders();

            Vector q(3+qj.size(), 0.0);
            q.setSubvector(3,qj);

            if(iloc)
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


        if (!request.check("domain") && iloc && imap)
        {
            Map2DLocation loc;
            iloc->getCurrentPosition(loc);
            vector<string> areaNames;
            imap->getAreasList(areaNames);

            Map2DArea area;
            for(size_t i=0 ; i<areaNames.size() ; i++)
            {
                imap->getArea(areaNames[i], area);
                if(area.checkLocationInsideArea(loc))
                    break;
            }

            Bottle b;
            Bottle &coord = b.addList();
            for(size_t i=0 ; i<area.points.size() ; i++)
            {
                area.points.size();
                coord.addDouble(area.points[i].x);
                coord.addDouble(area.points[i].y);
            }

            request.put("domain",b.get(0));
        }

        if (verbosity>0)
            yInfo("Forwarding request to solver: %s",request.toString().c_str());

        Bottle reply;
        bool latch_controlling=controlling;
        if (solverPort.write(request,reply))
        {
            if (verbosity>0)
                yInfo("Received reply from solver: %s",reply.toString().c_str());

            if (reply.get(0).asVocab()==Vocab::encode("ack"))
            {
                if ((reply.size()>1) && !reply.check("parameters"))
                {
                    Bottle *payLoadJoints=reply.find("q").asList();
                    Bottle *payLoadPose=reply.find("x").asList();

                    if ((payLoadJoints!=NULL) && (payLoadPose!=NULL))
                    {
                        // TODO navigate to location

                        LockGuard lg(mutex);
                        // process only if we didn't receive
                        // a stop request in the meanwhile
                        if (controlling==latch_controlling)
                        {
                            for (size_t i=0; i<qd.length(); i++)
                                qd[i]=payLoadJoints->get(3+i).asDouble();

                            if (xd.length()==0)
                                xd.resize((size_t)payLoadPose->size());

                            for (size_t i=0; i<xd.length(); i++)
                                xd[i]=payLoadPose->get(i).asDouble();

                            target=reply.tail();

                            if (!controlling)
                                gen->init(getEncoders());

                            controlling=true;
                            if (verbosity>0)
                                yInfo("Going to: %s",qd.toString(3,3).c_str());
                        }
                    }
                }

                return true;
            }
            else
                yError("Malformed target type!");
        }
        else
            yError("Unable to communicate with the solver");

        return false;
    }

    /****************************************************************/
    bool ask(Property &request, Bottle &reply)
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

            if(iloc)
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
                    yDebug() << "/t" << areaNames[i];
            }

            Map2DArea area;
            for(size_t i=0 ; i<areaNames.size() ; i++)
            {
                imap->getArea(areaNames[i], area);
                if(area.checkLocationInsideArea(loc))
                {
                    if (verbosity>0)
                        yDebug() << "Currently in" << areaNames[i];
                    break;
                }
            }

            Bottle b;
            Bottle &coord = b.addList();
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

            request.put("domain",b.get(0));
        }

        if (verbosity>0)
            yInfo("Forwarding request to solver: %s",request.toString().c_str());

        reply.clear();
        if (solverPort.write(request,reply))
        {
            if (verbosity>0)
                yInfo("Received reply from solver: %s",reply.toString().c_str());

            if (reply.get(0).asVocab()==Vocab::encode("ack"))
            {
                reply=reply.tail();
                return true;
            }
            else
                yError("Malformed target type!");
        }
        else
            yError("Unable to communicate with the solver");

        return false;
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

        if (controlling)
        {
            gen->computeNextValues(qd);
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

                if (dist(qd-q.subVector(3,q.size()-1)))
                {
                    controlling=false;
                    if (verbosity>0)
                        yInfo("Just stopped at: %s",q.toString(3,3).c_str());
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
        int cmd_0=cmd.get(0).asVocab();
        if (cmd.size()==3)
        {
            if (cmd_0==Vocab::encode("set"))
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

                    if (go(p))
                        reply.addVocab(Vocab::encode("ack"));
                }
                else if (cmd_1=="torso_heave")
                {
                    Value torso_heave(cmd.get(2).asDouble());
                    Property p=prepareSolverOptions("torso_heave",torso_heave);

                    if (go(p))
                        reply.addVocab(Vocab::encode("ack"));
                }
                else if (cmd_1=="lower_arm_heave")
                {
                    Value lower_arm_heave(cmd.get(2).asDouble());
                    Property p=prepareSolverOptions("lower_arm_heave",lower_arm_heave);

                    if (go(p))
                        reply.addVocab(Vocab::encode("ack"));
                }
                else if (cmd_1=="tol")
                {
                    Value tol(cmd.get(2).asDouble());
                    Property p=prepareSolverOptions("tol",tol);

                    if (go(p))
                        reply.addVocab(Vocab::encode("ack"));
                }
                else if (cmd_1=="constr_tol")
                {
                    Value constr_tol(cmd.get(2).asDouble());
                    Property p=prepareSolverOptions("constr_tol",constr_tol);

                    if (go(p))
                        reply.addVocab(Vocab::encode("ack"));
                }
            }
        }
        else if (cmd.size()==2)
        {
            if (cmd_0==Vocab::encode("get"))
            {
                string cmd_1=cmd.get(1).asString();
                if (cmd_1=="done")
                {
                    reply.addVocab(Vocab::encode("ack"));

                    mutex.lock();
                    reply.addInt(controlling?0:1);
                    mutex.unlock();
                }
                else if (cmd_1=="target")
                {
                    reply.addVocab(Vocab::encode("ack"));

                    mutex.lock();
                    reply.addList()=target;
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
            }
            else if (cmd_0==Vocab::encode("go"))
            {
                if (Bottle *b=cmd.get(1).asList())
                {
                    Property p(b->toString().c_str());
                    if (go(p))
                        reply.addVocab(Vocab::encode("ack"));
                }
            }
            else if (cmd_0==Vocab::encode("ask"))
            {
                if (Bottle *b=cmd.get(1).asList())
                {
                    Property p(b->toString().c_str());
                    Bottle payLoad;
                    if (ask(p,payLoad))
                    {
                        reply.addVocab(Vocab::encode("ack"));
                        reply.append(payLoad);
                    }
                }
            }
        }
        else if (cmd_0==Vocab::encode("stop"))
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
void TargetPort::onRead(Property &request)
{
    if (ctrl!=NULL)
        ctrl->go(request);
}


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

