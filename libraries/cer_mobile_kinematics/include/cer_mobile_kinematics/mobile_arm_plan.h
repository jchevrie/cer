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

#ifndef __CER_KINEMATICS_ARM_H__
#define __CER_KINEMATICS_ARM_H__

#include <deque>
#include <vector>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <cer_kinematics/utils.h>

namespace cer {
namespace kinematics {

/**
 * Class to handle direct and inverse kinematics of the robot 
 * arm and mobile base.
 * 
 * @author Ugo Pattacini
 */
class MobileArmPlanSolver : public Solver
{
protected:
    ArmParameters armParameters;
    SolverParameters slvParameters;
    bool domain_constr;
    yarp::sig::Vector domainPoly;
    double outDomainHeight;
    double navigationMargin;
    double obstacleMargin;
    int nb_checkPoints;
    yarp::sig::Vector q0;
    yarp::sig::Vector zL;
    yarp::sig::Vector zU;
    yarp::sig::Vector lambda;
    int curMode;

    friend class MobileArmPlanCommonNLP;

    int computeMode() const;

public:
    /**
     * Constructor.
     * 
     * @param armParams arm parameters. 
     * @param slvParams solver parameters. 
     * @param domain    list of points describing the polygon in which
     *                  the mobile base should stay.
     * @param nb_points number of intermediate checkpoints between targets.
     * @param verb      integers greater than 0 enable successive 
     *                  levels of verbosity (default=0).
     */
    MobileArmPlanSolver(const ArmParameters &armParams=ArmParameters(),
              const SolverParameters &slvParams=SolverParameters(),
              const yarp::sig::Vector &domain=yarp::sig::Vector(),
              const int nb_points=0, const int verb=0);

    /**
     * Define parameters of the arm.
     * 
     * @param params arm parameters.
     */
    virtual void setArmParameters(const ArmParameters &params)
    {
        armParameters=params;
    }

    /**
     * Retrieve parameters of the arm.
     * 
     * @return arm parameters.
     */
    virtual const ArmParameters& getArmParameters() const
    {
        return armParameters;
    }

    /**
     * Define parameters of the solver.
     * 
     * @param params solver parameters.
     */
    virtual void setSolverParameters(const SolverParameters &params)
    {
        slvParameters=params;
    }

    /**
     * Retrieve parameters of the solver.
     *
     * @return solver parameters.
     */
    virtual const SolverParameters& getSolverParameters() const
    {
        return slvParameters;
    }

    /**
     * Define the mobile base domain polygon.
     *
     * @param poly mobile base domain polygon.
     */
    virtual void setDomain(const yarp::sig::Vector &domain)
    {
        if (domain.size()>5)
        {
            domainPoly=domain;
            domain_constr=true;
            if(domainPoly.size()%2!=0)
                domainPoly.pop_back();
        }
        else
        {
            domainPoly.clear();
            domain_constr=false;
        }
    }

    /**
     * Retrieve the mobile base domain polygon.
     * 
     * @return mobile base domain polygon.
     */
    virtual const yarp::sig::Vector& getDomain() const
    {
        return domainPoly;
    }

    /**
     * Define the height of the zone outside the authorized base
     * domain for collision avoidance (typically table).
     *
     * @param height height of the obstacles outside the base domain.
     */
    virtual void setOutDomainHeight(double height)
    {
        outDomainHeight=height;
    }

    /**
     * Retrieve the height of the zone outside the authorized base
     * domain for collision avoidance (typically table).
     *
     * @return height of the obstacles outside the base domain.
     */
    virtual double getOutDomainHeight() const
    {
        return outDomainHeight;
    }

    /**
     * Define the distance between the boundaries of the domain of
     * the mobile base and the actual start of the obstacles.
     *
     * @param margin distance between obstacles and base domain.
     */
    virtual void setNavigationMargin(double margin)
    {
        navigationMargin=margin;
    }

    /**
     * Retrieve the distance between the boundaries of the domain of
     * the mobile base and the actual start of the obstacles.
     *
     * @return distance between obstacles and base domain.
     */
    virtual double getNavigationeMargin() const
    {
        return navigationMargin;
    }

    /**
     * Define the margin between the end effector and the obstacles.
     *
     * @param margin margin for obstacle avoidance.
     */
    virtual void setObstacleMargin(double margin)
    {
        obstacleMargin=margin;
    }

    /**
     * Retrieve the margin between the end effector and the obstacles.
     *
     * @return margin for obstacle avoidance.
     */
    virtual double getObstacleMargin() const
    {
        return obstacleMargin;
    }

    /**
     * Specify the initial DOFs values.
     * 
     * @param q0   initial DOFs values ([m]-[deg]-[m]). 
     * @return true/false on success/failure. 
     */
    virtual bool setInitialGuess(const yarp::sig::Vector &q0);

    /**
     * Retrieve the initial guess used for DOFs.
     * 
     * @return the initial DOFs values ([m]-[deg]-[m]).
     */
    virtual yarp::sig::Vector getInitialGuess() const
    {
        return q0;
    }

    /**
     * Define the number of end effector checkpoints of the trajectory.
     *
     * @param nbPoints number of end effector checkpoints.
     */
    virtual void setNbCheckPoints(int nbPoints)
    {
        nb_checkPoints=std::max(0,nbPoints);
    }

    /**
     * Retrieve the number of end effector checkpoints of the trajectory.
     *
     * @return number of end effector checkpoints.
     */
    virtual int getNbCheckPoints() const
    {
        return nb_checkPoints;
    }

    /**
     * Forward Kinematics Law.
     * 
     * @param q      the DOFs values ([m]-[deg]-[m]).
     * @param H      the 4-by-4 homogeneous matrix of the specified 
     *               frame ([m]).
     * @param frame  specify the DOF number whose frame is returned. 
     *               Thus, frame is in [0...nDOF-1]; negative
     *               numbers account for the end-effector frame.
     * @return true/false on success/failure.
     */
    virtual bool fkin(const yarp::sig::Vector &q, yarp::sig::Matrix &H,
                      const int frame=-1);

    /**
     * Inverse Kinematics Law.
     * 
     * @param Hd        the list of desired 4-by-4 homogeneous matrices
     *                  representing the end-effector frame ([m]).
     * @param q         the solved DOFs ([m]-[deg]-[m]). 
     * @param exit_code pointer to solver's exit codes. 
     * @return true/false on success/failure.
     */
    virtual bool ikin(const std::vector<yarp::sig::Matrix> &Hd, yarp::sig::Vector &q,
                      int *exit_code=NULL);

    virtual bool ikin(const yarp::sig::Matrix &Hd, yarp::sig::Vector &q,
                      int *exit_code=NULL);

    /**
     * Destructor.
     */
    virtual ~MobileArmPlanSolver() { }
};

}

}

#endif

