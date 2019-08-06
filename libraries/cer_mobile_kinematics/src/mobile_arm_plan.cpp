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
#include <cmath>
#include <limits>
#include <algorithm>

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/os/LockGuard.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <iCub/ctrl/math.h>
#include <iCub/iKin/iKinFwd.h>

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

#include <cer_mobile_kinematics/mobile_arm_plan.h>
#include <cer_kinematics/private/helpers.h>

// COMMON PART -- begin
#define DELTA_RHO       1e-6

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace cer::kinematics;
// COMMON PART -- end

namespace cer {
    namespace kinematics {
        #include <cer_mobile_kinematics/private/mobile_arm_plan_common.h>
        #include <cer_mobile_kinematics/private/mobile_arm_plan_full_nobase_notorso_noheave.h>
    }
}


/****************************************************************/
MobileArmPlanSolver::MobileArmPlanSolver(const ArmParameters &armParams,
                     const SolverParameters &slvParams,
                     const Vector &domain,
                     const int nb_points,
                     const int verb) :
                     Solver(verb),
                     armParameters(armParams),
                     slvParameters(slvParams)
{
    this->setDomain(domain);
    this->setNbCheckPoints(nb_points);
    this->setOutDomainHeight(0.8);
    this->setNavigationMargin(0.3);
    this->setObstacleMargin(0.1);
    q0.resize(3+3+armParameters.upper_arm.getDOF()+3,0.0);
    curMode=computeMode();
}


/****************************************************************/
int MobileArmPlanSolver::computeMode() const
{
    return ((slvParameters.full_pose?0x01:0x00) | 
            (slvParameters.configuration<<1));
}


/****************************************************************/
bool MobileArmPlanSolver::setInitialGuess(const Vector &q0)
{
    size_t L=3+3+armParameters.upper_arm.getDOF()+3;
    yAssert(q0.length()>=L);

    this->q0=q0.subVector(0,L-1);
    return true;
}


/****************************************************************/
bool MobileArmPlanSolver::fkin(const Vector &q, Matrix &H, const int frame)
{
    size_t L=3+3+armParameters.upper_arm.getDOF()+3;
    yAssert(q.length()>=L);

    Ipopt::SmartPtr<MobileArmPlanFullNoBaseNoTorsoNoHeaveNLP_ForwardDiff> nlp=new MobileArmPlanFullNoBaseNoTorsoNoHeaveNLP_ForwardDiff(*this);
    H=nlp->fkin(q,frame);
    return true;
}


/****************************************************************/
bool MobileArmPlanSolver::ikin(const vector<Matrix> &Hd, Vector &q, int *exit_code)
{
    LockGuard lg(makeThreadSafe);
    for(size_t i=0; i<Hd.size(); i++)
        yAssert((Hd[i].rows()==4)&&(Hd[i].cols()==4));

    int mode=computeMode();
    int print_level=std::max(verbosity-5,0);
    string warm_start_str="no";    

    Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue("tol",slvParameters.tol);
    app->Options()->SetNumericValue("constr_viol_tol",slvParameters.constr_tol);
    app->Options()->SetIntegerValue("acceptable_iter",0);
    app->Options()->SetStringValue("mu_strategy","monotone");
    app->Options()->SetIntegerValue("max_iter",slvParameters.max_iter);
    app->Options()->SetNumericValue("max_cpu_time",slvParameters.max_cpu_time);
    app->Options()->SetStringValue("nlp_scaling_method","none");
    app->Options()->SetStringValue("hessian_approximation","limited-memory");
    app->Options()->SetStringValue("fixed_variable_treatment","make_parameter");
    app->Options()->SetStringValue("derivative_test",print_level>=4?"first-order":"none");
    app->Options()->SetNumericValue("derivative_test_perturbation",DELTA_RHO);
    app->Options()->SetStringValue("derivative_test_print_all", print_level>=5?"yes":"no");
    app->Options()->SetIntegerValue("print_level",print_level);
    if (slvParameters.warm_start)
    {
        if ((zL.length()>0) && (zU.length()>0) && (lambda.length()>0) && (curMode==mode))
        {
            warm_start_str="yes";
            app->Options()->SetNumericValue("warm_start_bound_push",1e-6);
            app->Options()->SetNumericValue("warm_start_mult_bound_push",1e-6);
            app->Options()->SetNumericValue("mu_init",1e-6);
        }
        else if (verbosity>0)
            yWarning()<<" *** Arm Solver: requested \"warm start\" but values are not available => \"warm start\" is disabled!";
    }
    app->Options()->SetStringValue("warm_start_init_point",warm_start_str.c_str());
    app->Initialize();

    Ipopt::SmartPtr<MobileArmPlanCommonNLP> nlp;
    if (slvParameters.full_pose)
    {
        switch (slvParameters.configuration)
        {
        case configuration::no_torso_no_heave:
        case configuration::no_torso_heave:
        case configuration::heave:
        case configuration::torso_yaw_no_heave:
        case configuration::no_base_no_torso_no_heave:
        default:
        case configuration::no_base_no_torso_no_heave_plan:
            if (slvParameters.use_central_difference)
                nlp=new MobileArmPlanFullNoBaseNoTorsoNoHeaveNLP_CentralDiff(*this,Hd.size(),nb_checkPoints);
            else
                nlp=new MobileArmPlanFullNoBaseNoTorsoNoHeaveNLP_ForwardDiff(*this,Hd.size(),nb_checkPoints);
            break;
        }
    }
    else
    {
        switch (slvParameters.configuration)
        {
        case configuration::no_torso_no_heave:
        case configuration::no_torso_heave:
        case configuration::heave:
        case configuration::torso_yaw_no_heave:
        case configuration::no_base_no_torso_no_heave:
        default:
        case configuration::no_base_no_torso_no_heave_plan:
            if (slvParameters.use_central_difference)
                nlp=new MobileArmPlanFullNoBaseNoTorsoNoHeaveNLP_CentralDiff(*this,Hd.size(),nb_checkPoints);
            else
                nlp=new MobileArmPlanFullNoBaseNoTorsoNoHeaveNLP_ForwardDiff(*this,Hd.size(),nb_checkPoints);
            break;
        }
    }

    nlp->set_q0(q0);
    nlp->set_warm_start(zL,zU,lambda);
    nlp->set_targets(Hd);
    nlp->set_domain(domainPoly);
    nlp->set_out_domain_height(outDomainHeight);
    nlp->set_navigation_margin(navigationMargin);
    nlp->set_obstacle_margin(obstacleMargin);

    double t0=Time::now();
    Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));
    double t1=Time::now();

    curMode=mode;
    q=nlp->get_result();
    nlp->get_warm_start(zL,zU,lambda);
    if (exit_code!=NULL)
        *exit_code=status;

    if (verbosity>0)
    {
        yInfo()<<" *** Arm Solver ******************************";
        yInfo()<<" *** Arm Solver:              arm ="<<armParameters.upper_arm.getType();
        yInfo()<<" *** Arm Solver:             mode ="<<nlp->get_mode();
        yInfo()<<" *** Arm Solver:       warm_start ="<<warm_start_str;
        yInfo()<<" *** Arm Solver:          tol [*] ="<<slvParameters.tol;
        yInfo()<<" *** Arm Solver:   constr_tol [*] ="<<slvParameters.constr_tol;
        yInfo()<<" *** Arm Solver:     max_iter [#] ="<<slvParameters.max_iter;
        yInfo()<<" *** Arm Solver: max_cpu_time [s] ="<<slvParameters.max_cpu_time;
        yInfo()<<" *** Arm Solver:           q0 [*] = ("<<q0.toString(4,4)<<")";
        yInfo()<<" *** Arm Solver:          hd1 [m] ="<<slvParameters.torso_heave;
        yInfo()<<" *** Arm Solver:          hd2 [m] ="<<slvParameters.lower_arm_heave;
        yInfo()<<" *** Arm Solver: table height [m] ="<<outDomainHeight;
        yInfo()<<" *** Arm Solver:  nav. margin [m] ="<<navigationMargin;
        yInfo()<<" *** Arm Solver: obst. margin [m] ="<<obstacleMargin;
        yInfo()<<" *** Arm Solver:          dt [ms] ="<<1000.0*(t1-t0);
        int nb_kin_DOF = (q.size()-3)/((nb_checkPoints+1)*Hd.size());
        for(size_t i=0; i<Hd.size(); i++)
        {
            Vector xd=Hd[i].getCol(3).subVector(0,2);
            Vector ud=dcm2axis(Hd[i]);
            Vector qt(3+nb_kin_DOF);
            qt.setSubvector(0,q.subVector(0,2));
            qt.setSubvector(3,q.subVector(3+((i+1)*(nb_checkPoints+1)-1)*nb_kin_DOF,3+(i+1)*(nb_checkPoints+1)*nb_kin_DOF-1));
            Matrix H=nlp->fkin(qt);
            TripodState din1,din2;
            nlp->tripod_fkin(1,qt,&din1);
            nlp->tripod_fkin(2,qt,&din2);
            Vector x=H.getCol(3).subVector(0,2);
            Vector e_u=dcm2axis(Hd[i]*H.transposed());
            e_u*=e_u[3]; e_u.pop_back();
            vector<Vector> check_points(nb_checkPoints,Vector(3));
            for(size_t j=0; j<nb_checkPoints; j++)
            {
                qt.setSubvector(3,q.subVector(3+(i*(nb_checkPoints+1)+j)*nb_kin_DOF,3+(i*(nb_checkPoints+1)+j+1)*nb_kin_DOF-1));
                H=nlp->fkin(qt);
                check_points[j]=cat(H.getCol(3).subVector(0,2), dcm2axis(H));
            }


            yInfo()<<" *** Arm Solver: ** Target" << i;
            yInfo()<<" *** Arm Solver:           xd [m] = ("<<xd.toString(4,4)<<")";
            yInfo()<<" *** Arm Solver:         ud [rad] = ("<<ud.toString(4,4)<<")";
            yInfo()<<" *** Arm Solver:            q [*] = ("<<qt.toString(4,4)<<")";
            yInfo()<<" *** Arm Solver:          e_x [m] ="<<norm(xd-x);
            yInfo()<<" *** Arm Solver:        e_u [rad] ="<<norm(e_u);
            yInfo()<<" *** Arm Solver:         e_h1 [m] ="<<fabs(slvParameters.torso_heave-din1.p[2]);
            yInfo()<<" *** Arm Solver:     alpha1 [deg] ="<<CTRL_RAD2DEG*acos(din1.n[2]);
            yInfo()<<" *** Arm Solver:         e_h2 [m] ="<<fabs(slvParameters.lower_arm_heave-din2.p[2]);
            yInfo()<<" *** Arm Solver:     alpha2 [deg] ="<<CTRL_RAD2DEG*acos(din2.n[2]);
            yInfo()<<" *** Arm Solver:    *  Checkpoints:";
            for(size_t j=0; j<nb_checkPoints; j++)
            {
            yInfo()<<" *** Arm Solver:        ("<<check_points[j].toString(4,4)<<")";
            }
        }

        yInfo()<<" *** Arm Solver ******************************";
    }

    switch (status)
    {
        case Ipopt::Solve_Succeeded:
        case Ipopt::Solved_To_Acceptable_Level:
        case Ipopt::Feasible_Point_Found:
        {
            if (verbosity>0)
                yInfo()<<" *** Arm Solver: IpOpt return code"<<status;
            return true;
        } 

        default:
        {
            if (verbosity>0)
                yWarning()<<" *** Arm Solver: IpOpt return code"<<status;
            return false;
        } 
    }
}

/****************************************************************/
bool MobileArmPlanSolver::ikin(const Matrix &Hd, Vector &q, int *exit_code)
{
    vector<Matrix> Hd_(1,Hd);
    return ikin(Hd_,q,exit_code);
}
