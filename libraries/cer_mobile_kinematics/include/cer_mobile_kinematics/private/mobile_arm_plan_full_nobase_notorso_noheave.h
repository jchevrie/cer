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

/*#ifndef IEXPMAPDEF
#define IEXPMAPDEF
Vector iExpMap(const Matrix &M, const double &delta_t)
{
    Vector v(6,0.0);
    double theta,si,co,sinc,mcosc,msinc,det;
    Vector u;

    Matrix Rd=M.submatrix(0,2, 0,2);
    u=dcm2axis(Rd);
    u*=u[3];
    u.pop_back();

    for (int i=0; i<3; i++)
        v[3+i]=u[i];

    theta=sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
    si=sin(theta);
    co=cos(theta);
    if (fabs(theta)<1e-8)
        sinc=1.0;
    else
        sinc=si/theta;
    if (fabs(theta) < 2.4e-4)
    {
        mcosc=0.5;
        msinc=1./6.0;
    }
    else
    {
        mcosc=(1.0-co)/(theta*theta);
        msinc=((1.0-si/theta)/theta/theta) ;
    }

    Matrix a(3,3);
    a[0][0] = sinc + u[0]*u[0]*msinc;
    a[0][1] = u[0]*u[1]*msinc - u[2]*mcosc;
    a[0][2] = u[0]*u[2]*msinc + u[1]*mcosc;

    a[1][0] = u[0]*u[1]*msinc + u[2]*mcosc;
    a[1][1] = sinc + u[1]*u[1]*msinc;
    a[1][2] = u[1]*u[2]*msinc - u[0]*mcosc;

    a[2][0] = u[0]*u[2]*msinc - u[1]*mcosc;
    a[2][1] = u[1]*u[2]*msinc + u[0]*mcosc;
    a[2][2] = sinc + u[2]*u[2]*msinc;

    det = a[0][0]*a[1][1]*a[2][2] + a[1][0]*a[2][1]*a[0][2]
         + a[0][1]*a[1][2]*a[2][0] - a[2][0]*a[1][1]*a[0][2]
         - a[1][0]*a[0][1]*a[2][2] - a[0][0]*a[2][1]*a[1][2];

    if (fabs(det) > 1.e-5)
    {
        v[0] =  (M[0][3]*a[1][1]*a[2][2]
                +   M[1][3]*a[2][1]*a[0][2]
                +   M[2][3]*a[0][1]*a[1][2]
                -   M[2][3]*a[1][1]*a[0][2]
                -   M[1][3]*a[0][1]*a[2][2]
                -   M[0][3]*a[2][1]*a[1][2])/det;
        v[1] =  (a[0][0]*M[1][3]*a[2][2]
                +   a[1][0]*M[2][3]*a[0][2]
                +   M[0][3]*a[1][2]*a[2][0]
                -   a[2][0]*M[1][3]*a[0][2]
                -   a[1][0]*M[0][3]*a[2][2]
                -   a[0][0]*M[2][3]*a[1][2])/det;
        v[2] =  (a[0][0]*a[1][1]*M[2][3]
                +   a[1][0]*a[2][1]*M[0][3]
                +   a[0][1]*M[1][3]*a[2][0]
                -   a[2][0]*a[1][1]*M[0][3]
                -   a[1][0]*a[0][1]*M[2][3]
                -   a[0][0]*a[2][1]*M[1][3])/det;
    }
    else
    {
        v[0] = M[0][3];
        v[1] = M[1][3];
        v[2] = M[2][3];
    }

    // Apply the sampling time to the computed velocity
    v /= delta_t;

    return v;
}
#endif*/

/****************************************************************/
class MobileArmPlanFullNoBaseNoTorsoNoHeaveNLP_ForwardDiff : public MobileArmPlanCommonNLP
{
protected:
    const double s_pos=0.01;
    const double s_ang=0.001;
public:
    /****************************************************************/
    MobileArmPlanFullNoBaseNoTorsoNoHeaveNLP_ForwardDiff(MobileArmPlanSolver &slv_, int nb_targets=1, int nb_checkPoints=0) : MobileArmPlanCommonNLP(slv_, nb_targets, nb_checkPoints)
    {

    }

    /****************************************************************/
    string get_mode() const
    {
        return "full_pose+plan+no_base_no_torso_no_heave+forward_diff";
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {yDebug() << "d info s";
        n=x.length();
        m=nb_total_points*(2+1)+nb_targets*(3+3);
        nnz_jac_g=nb_total_points*(2*3+1*2)+nb_targets*(3*(nb_kin_DOF-4)+3*(nb_kin_DOF-4));

        /*if(nb_total_points>1)
        {
            m+=(nb_total_points-1);
            nnz_jac_g+=(2*(nb_kin_DOF-4)+3*(nb_kin_DOF-4)*(nb_total_points-2));
        }*/

        if(domain_constr)
        {
            m+=nb_total_points;
            nnz_jac_g+=nb_total_points*(nb_kin_DOF-4);
        }

        nnz_h_lag=0;
        index_style=TNLP::C_STYLE;
yDebug() << "d info e";
        return true;
    }

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
    {yDebug() << "d bound s";
        if(n>0)
        {
            for (size_t i=0; i<3; i++)
            {
                x_l[idx_b+i]=x_u[idx_b+i]=x0[idx_b+i];
            }

            for (size_t i=0; i<nb_total_points; i++)
            {
                for (size_t j=0; j<3; j++)
                    x_l[idx_t[i]+j]=x_u[idx_t[i]+j]=x0[idx_t[0]+j];

                x_l[idx_ua[i]+0]=x_u[idx_ua[i]+0]=x0[idx_ua[0]+0];
                iKinChain *chain=upper_arm.asChain();
                for (size_t j=1; j<upper_arm.getDOF(); j++)
                {
                    x_l[idx_ua[i]+j]=(*chain)[j].getMin();
                    x_u[idx_ua[i]+j]=(*chain)[j].getMax();
                }

                for (size_t j=0; j<3; j++)
                {
                    x_l[idx_la[i]+j]=lower_arm.l_min;
                    x_u[idx_la[i]+j]=lower_arm.l_max;
                }
            }
        }

        if(m>0)
        {
            latch_idx.clear();
            latch_gl.clear();
            latch_gu.clear();

            int nb_constr=3;
            if(domain_constr)
                nb_constr++;

            for (size_t i=0; i<nb_total_points; i++)
            {
                // g[0] fixed wrist heave
                g_l[nb_constr*i+0]=g_u[nb_constr*i+0]=0.0;

                // g[1] limit wrist angle
                g_l[nb_constr*i+1]=lower_arm.cos_alpha_max;
                g_u[nb_constr*i+1]=1.0;

                // g[2] cover constraints
                g_l[nb_constr*i+2]=cover_shoulder_avoidance[1];
                g_u[nb_constr*i+2]=std::numeric_limits<double>::max();

                //g[3] table avoidance
                if(domain_constr)
                {
                    if ((i%(nb_checkPoints+1))==nb_checkPoints)
                        g_l[nb_constr*i+3]=0.0;
                    else
                        g_l[nb_constr*i+3]=obstacle_margin;
                    g_u[nb_constr*i+3]=std::numeric_limits<double>::max();
                }

                latch_idx.push_back(1);
                latch_gl.push_back(g_l[nb_constr*i+2]);
                latch_gu.push_back(g_u[nb_constr*i+2]);
            }

            Ipopt::Index off = nb_constr*nb_total_points;
            for (Ipopt::Index i=0; i<nb_targets; i++)
            {
                // g[4] target position reaching
                g_l[off+6*i+0]=g_u[off+6*i+0]=0.0;
                g_l[off+6*i+1]=g_u[off+6*i+1]=0.0;
                g_l[off+6*i+2]=g_u[off+6*i+2]=0.0;

                // g[5] target orientation reaching
                g_l[off+6*i+3]=g_u[off+6*i+3]=0.0;
                g_l[off+6*i+4]=g_u[off+6*i+4]=0.0;
                g_l[off+6*i+5]=g_u[off+6*i+5]=0.0;
            }

            /*if(nb_total_points>1)
            {
                off=nb_constr*nb_total_points+6*nb_targets;
                for (Ipopt::Index i=0; i<nb_total_points-1; i++)
                {
                    g_l[off+i]=g_u[off+i]=0.0;
                }
            }*/
        }
yDebug() << "d bound e";
        return true;
    }

    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value)
    {yDebug() << "d f s";
        computeQuantities(x,new_x);

        Vector xe1=Hb*T0.getCol(3);
        yDebug() << "X0\n" << xe1.toString();
        xe1.pop_back();
        Vector xe2=xe1;

        obj_value=0;
        for(size_t i=0; i<nb_total_points; i++)
        {
            xe1=xe[i];
            obj_value+=norm2(xe1-xe2);
            xe2=xe1;

            //if(domain_constr)
            //    obj_value-=obs_dist[i];
        }

        for (Ipopt::Index i=0; i<nb_targets; i++)
        {
            obj_value+=norm2(xd[i]-xe[target_idx[i]]);
        }
yDebug() << "d f e" << (double)obj_value;
        return true;
    }

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {yDebug() << "d gf s";
        computeQuantities(x,new_x);

        // init
        for(size_t i=0; i<n; i++)
        {
            grad_f[i]=0;
        }

        // compute position gradient of each point
        Matrix grad(3*nb_total_points, nb_kin_DOF);

        Ipopt::Number x_dx[n];
        for (Ipopt::Index i=0; i<n; i++)
            x_dx[i]=x[i];

        TripodState d_fw;

        for(size_t i=0; i<nb_total_points; i++)
        {
            // base

            // torso

            // upper_arm
            grad.setSubmatrix(Hb.submatrix(0,2,0,2)*J_[i].submatrix(0,2,1,upper_arm.getDOF()-1), 3*i, 4);

            // lower_arm
            Vector fw;
            Matrix M=Hb*d1[i].T*H[i];

            for(size_t j=0; j<3; j++)
            {
                x_dx[idx_la[i]+j]=x[idx_la[i]+j]+drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                fw=(M*d_fw.T*TN).getCol(3).subVector(0,2);
                grad.setSubcol(1.0/drho*(fw-xe[i]), 3*i, idx_la[0]-3+j);
                x_dx[idx_la[i]+j]=x[idx_la[i]+j];
            }
        }

        // compute grad_f
        Vector xe1=xe[0];
        Vector xe2=Hb*T0.getCol(3);
        xe2.pop_back();
        Vector diff=xe1-xe2;
        for(size_t j=4; j<nb_kin_DOF; j++)
        {
            grad_f[idx_t[0]+j]+=2*dot(grad.getCol(j).subVector(0,2), diff);
        }
        xe2=xe1;

        /*if(domain_constr)
        {
            //g[3] table avoidance
            Vector xe_fw=xe[0];
            Vector gradXe(3,0.0);
            for (size_t j=0; j<3; j++)
            {
                xe_fw[j]=xe[0][j]+drho;
                gradXe[j]=(distanceFromOutDomainObstacle(domain_poly,out_domain_height,xe_fw)-obs_dist[0])/drho;
                xe_fw[j]=xe[0][j];
            }

            for(size_t j=4; j<nb_kin_DOF; j++)
                grad_f[idx_t[0]+j]-=dot(gradXe, grad.getCol(j).subVector(0,2));
        }*/

        for(size_t i=1; i<nb_total_points; i++)
        {
            xe1=xe[i];
            diff=xe1-xe2;

            for(size_t j=4; j<nb_kin_DOF; j++)
            {
                grad_f[idx_t[i]+j]+=2*dot(grad.getCol(j).subVector(3*i,3*i+2), diff);
                grad_f[idx_t[i-1]+j]+=-2*dot(grad.getCol(j).subVector(3*(i-1),3*(i-1)+2), diff);
            }
            xe2=xe1;

            //g[3] table avoidance
            /*if(domain_constr)
            {
                Vector xe_fw=xe[i];
                Vector gradXe(3,0.0);
                for (size_t j=0; j<3; j++)
                {
                    xe_fw[j]=xe[i][j]+drho;
                    gradXe[j]=(distanceFromOutDomainObstacle(domain_poly,out_domain_height,xe_fw)-obs_dist[i])/drho;
                    xe_fw[j]=xe[i][j];
                }

                for(size_t j=4; j<nb_kin_DOF; j++)
                    grad_f[idx_t[i]+j]-=dot(gradXe, grad.getCol(j).subVector(3*i,3*i+2));
            }*/
        }


        for (size_t i=0; i<nb_targets; i++)
        {
            for (size_t j=4; j<nb_targets; j++)
            {
                grad_f[idx_t[target_idx[i]]+j]+=2*dot(grad.getCol(j).subVector(3*target_idx[i],3*target_idx[i]+2), xd[i]-xe[target_idx[i]]);
            }
        }


yDebug() << "d gf e";
        return true;
    }

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g)
    {yDebug() << "d g s";
        computeQuantities(x,new_x);

        int nb_constr=3;
        if(domain_constr)
            nb_constr++;

        for (Ipopt::Index i=0; i<nb_total_points; i++)
        {
            // g[0,1] (lower_arm)
            double e2=hd2-din2[i].p[2];
            g[nb_constr*i+0]=e2*e2;
            g[nb_constr*i+1]=din2[i].n[2];

            // g[2] (cover constraints)
            g[nb_constr*i+2]=-cover_shoulder_avoidance[0]*x[idx_ua[i]+1]+x[idx_ua[i]+2];

            //g[3] table avoidance
            if(domain_constr)
                g[nb_constr*i+3]=obs_dist[i];
        }

        Ipopt::Index off=nb_constr*nb_total_points;
        for (Ipopt::Index i=0; i<nb_targets; i++)
        {
            // g[4] (reaching position)
            for(Ipopt::Index j=0; j<3 ;j++)
                g[off+6*i+j]=s_pos*(xd[i][j]-xe[target_idx[i]][j]);

            // g[5] (reaching orientation)
            Vector e=dcm2axis(Rd[i]*(Rb*T[target_idx[i]]).transposed());
            e*=s_ang*e[3];
            for(Ipopt::Index j=0; j<3 ;j++)
                g[off+6*i+3+j]=e[j];
        }


        /*if(nb_total_points>1)
        {
            // g[6] (equidistant checkpoints)
            Vector xe1=Hb*T0.getCol(3);
            yDebug() << "X0\n" << xe1.toString();
            xe1.pop_back();
            Vector xe2=xe1;

            Vector dist(nb_total_points);
            for(size_t i=0; i<nb_total_points; i++)
            {
                xe1=xe[i];
                dist[i]=norm2(xe1-xe2);
                yDebug() << "dist" << i << dist[i];
                xe2=xe1;
            }

            off=nb_constr*nb_total_points+6*nb_targets;
            for(size_t i=0; i<nb_total_points-1; i++)
            {
                g[off+i]=s_pos*(dist[i+1]-dist[i]);
            }
        }*/

        latch_x_verifying_alpha(n,x,g);

yDebug() << "d g e";
        return true;
    }

    /****************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                    Ipopt::Index *jCol, Ipopt::Number *values)
    {
        if (values==NULL)
        {yDebug() << "d jg s1";
            Ipopt::Index idx=0;

            int nb_constr=3;
            if(domain_constr)
                nb_constr++;

            for (size_t i=0; i<nb_total_points; i++)
            {
                // g[0,1] (lower_arm)
                iRow[idx]=nb_constr*i+0; jCol[idx]=idx_la[i]+0;idx++;
                iRow[idx]=nb_constr*i+1; jCol[idx]=idx_la[i]+0;idx++;
                iRow[idx]=nb_constr*i+0; jCol[idx]=idx_la[i]+1;idx++;
                iRow[idx]=nb_constr*i+1; jCol[idx]=idx_la[i]+1;idx++;
                iRow[idx]=nb_constr*i+0; jCol[idx]=idx_la[i]+2;idx++;
                iRow[idx]=nb_constr*i+1; jCol[idx]=idx_la[i]+2;idx++;

                // g[2] (cover constraints)
                iRow[idx]=nb_constr*i+2; jCol[idx]=idx_ua[i]+1;idx++;
                iRow[idx]=nb_constr*i+2; jCol[idx]=idx_ua[i]+2;idx++;

                //g[3] table avoidance
                if(domain_constr)
                {
                    for (Ipopt::Index col=idx_ua[i]+1; col<idx_la[i]+3; col++)
                    {
                        iRow[idx]=nb_constr*i+3; jCol[idx]=col;
                        idx++;
                    }
                }
            }

            Ipopt::Index off=nb_constr*nb_total_points;
            for (size_t i=0; i<nb_targets; i++)
            {
                // g[4] (reaching position)
                for (Ipopt::Index col=idx_ua[target_idx[i]]+1; col<idx_la[target_idx[i]]+3; col++)
                {
                    for (Ipopt::Index j=0; j<3; j++)
                    {
                        iRow[idx]=off+6*i+j; jCol[idx]=col;
                        idx++;
                    }
                }

                // g[5] (reaching orientation)
                for (Ipopt::Index col=idx_ua[target_idx[i]]+1; col<idx_la[target_idx[i]]+3; col++)
                {
                    for (Ipopt::Index j=0; j<3; j++)
                    {
                        iRow[idx]=off+6*i+3+j; jCol[idx]=col;
                        idx++;
                    }
                }
            }

            // g[6] (equidistant checkpoints)
            /*if(nb_total_points>1)
            {
                off=nb_constr*nb_total_points+6*nb_targets;
                for (size_t j=0; j<2; j++)
                {
                    for (Ipopt::Index col=idx_ua[j]+1; col<idx_la[j]+3; col++)
                    {
                        iRow[idx]=off; jCol[idx]=col;
                        idx++;
                    }
                }

                for(size_t i=1; i<nb_total_points-1; i++)
                {
                    for (int j=-1; j<2; j++)
                    {
                        for (Ipopt::Index col=idx_ua[i+j]+1; col<idx_la[i+j]+3; col++)
                        {
                            iRow[idx]=off+i; jCol[idx]=col;
                            idx++;
                        }
                    }
                }
            }*/
yDebug() << "d jg e1";
        }
        else
        {yDebug() << "d jg s2";
            computeQuantities(x,new_x);

            Ipopt::Number x_dx[n];
            for (Ipopt::Index i=0; i<n; i++)
                x_dx[i]=x[i];

            TripodState d_fw;

            Ipopt::Index idx=0;
            for (size_t i=0; i<nb_total_points; i++)
            {
                // g[0,1] (lower_arm)
                double e2=hd2-din2[i].p[2];

                for (size_t j=0; j<3; j++)
                {
                    x_dx[idx_la[i]+j]=x[idx_la[i]+j]+drho;
                    tripod_fkin(2,x_dx,&d_fw,i);
                    values[idx]=-2.0*e2*(d_fw.p[2]-din2[i].p[2])/drho;idx++;
                    values[idx]=(d_fw.n[2]-din2[i].n[2])/drho;idx++;
                    x_dx[idx_la[i]+j]=x[idx_la[i]+j];
                }

                // g[2] (cover constraints)
                values[idx]=-cover_shoulder_avoidance[0];idx++;
                values[idx]=1.0;idx++;

                //g[3] table avoidance
                if(domain_constr)
                {
                    // g[3] (upper_arm)
                    Vector xe_fw=xe[i];
                    Vector gradXe(3,0.0);
                    for (size_t j=0; j<3; j++)
                    {
                        xe_fw[j]=xe[i][j]+drho;
                        gradXe[j]=(distanceFromOutDomainObstacle(domain_poly,out_domain_height,xe_fw)-obs_dist[i])/drho;
                        xe_fw[j]=xe[i][j];
                    }

                    for (Ipopt::Index j=1; j<upper_arm.getDOF(); j++)
                    {
                        values[idx]=dot(gradXe, Hb.submatrix(0,2,0,2)*J_[i].getCol(j).subVector(0,2));
                        idx++;
                    }

                    // g[3] (lower_arm)
                    Matrix M=Hb*d1[i].T*H[i];
                    for (size_t j=0; j<3; j++)
                    {
                        x_dx[idx_la[i]+j]=x[idx_la[i]+j]+drho;
                        d_fw=tripod_fkin(2,x_dx,nullptr,i);
                        xe_fw=(M*d_fw.T*TN).getCol(3).subVector(0,2);
                        values[idx]=(distanceFromOutDomainObstacle(domain_poly,out_domain_height,xe_fw)-obs_dist[i])/drho;idx++;
                        x_dx[idx_la[i]+j]=x[idx_la[i]+j];
                    }
                }
            }

            for (size_t i=0; i<nb_targets; i++)
            {
                // g[4] (upper_arm)
                Matrix grad=Hb.submatrix(0,2,0,2)*J_[target_idx[i]].submatrix(0,2,0,upper_arm.getDOF()-1);
                for (size_t j=1; j<grad.cols(); j++)
                {
                    for (size_t k=0; k<3; k++)
                    {
                        values[idx]=-s_pos*grad[k][j];
                        idx++;
                    }
                }

                // g[4] (lower_arm)
                Vector e=xd[i]-xe[target_idx[i]];
                Vector e_fw;
                Matrix M=Hb*d1[target_idx[i]].T*H[target_idx[i]];
                for (size_t j=0; j<3; j++)
                {
                    x_dx[idx_la[target_idx[i]]+j]=x[idx_la[target_idx[i]]+j]+drho;
                    d_fw=tripod_fkin(2,x_dx,nullptr,target_idx[i]);
                    e_fw=xd[i]-(M*d_fw.T*TN).getCol(3).subVector(0,2);
                    values[idx]=s_pos*(e_fw[0]-e[0])/drho;idx++;
                    values[idx]=s_pos*(e_fw[1]-e[1])/drho;idx++;
                    values[idx]=s_pos*(e_fw[2]-e[2])/drho;idx++;
                    x_dx[idx_la[target_idx[i]]+j]=x[idx_la[target_idx[i]]+j];
                }

                // g[5] init
                Vector eo=dcm2axis(Rd[i]*(Rb*T[target_idx[i]]).transposed());
                double theta=eo[3];
                double coef=0.5*theta*cos(theta/2)/sin(theta/2);
                Vector eo_u=eo.subVector(0,2);
                eo*=theta;eo.pop_back();

                // g[5] (upper_arm)
                Matrix grado_ua=Rb.submatrix(0,2,0,2)*J_[target_idx[i]].submatrix(3,5,0,upper_arm.getDOF()-1);
                for (size_t j=1; j<grado_ua.cols(); j++)
                {
                    Vector w=grado_ua.getCol(j);
                    Vector eo_d=dot(w,eo_u)*eo_u;
                    Vector grado=s_ang*(-1*eo_d+coef*(eo_d-w)+0.5*theta*cross(w,eo_u));
                    values[idx]=grado[0];idx++;
                    values[idx]=grado[1];idx++;
                    values[idx]=grado[2];idx++;
                }

                // g[5] (lower_arm)
                Vector e_fwo;
                for (size_t j=0; j<3; j++)
                {
                    x_dx[idx_la[target_idx[i]]+j]=x[idx_la[target_idx[i]]+j]+drho;
                    d_fw=tripod_fkin(2,x_dx,nullptr,target_idx[i]);
                    e_fwo=dcm2axis(Rd[i]*(M*d_fw.T*TN).transposed()); e_fwo*=e_fwo[3]; e_fwo.pop_back();
                    values[idx]=s_ang*(e_fwo[0]-eo[0])/drho;idx++;
                    values[idx]=s_ang*(e_fwo[1]-eo[1])/drho;idx++;
                    values[idx]=s_ang*(e_fwo[2]-eo[2])/drho;idx++;
                    x_dx[idx_la[target_idx[i]]+j]=x[idx_la[target_idx[i]]+j];
                }
            }

            // g[6] (equidistant checkpoints)
            /*if(nb_total_points>1)
            {
                // compute position gradient of each point
                Matrix grad(3*nb_total_points, nb_kin_DOF);
                for(size_t i=0; i<nb_total_points; i++)
                {
                    // base

                    // torso

                    // upper_arm
                    grad.setSubmatrix(Hb.submatrix(0,2,0,2)*J_[i].submatrix(0,2,1,upper_arm.getDOF()-1), 3*i, 4);

                    // lower_arm
                    Vector fw;
                    Matrix M=Hb*d1[i].T*H[i];

                    for(size_t j=0; j<3; j++)
                    {
                        x_dx[idx_la[i]+j]=x[idx_la[i]+j]+drho;
                        d_fw=tripod_fkin(2,x_dx,nullptr,i);
                        fw=(M*d_fw.T*TN).getCol(3).subVector(0,2);
                        grad.setSubcol(1.0/drho*(fw-xe[i]), 3*i, idx_la[0]-3+j);
                        x_dx[idx_la[i]+j]=x[idx_la[i]+j];
                    }
                }

                // compute diffs
                Vector xe1=xe[0];
                Vector xe2=Hb*T0.getCol(3);
                xe2.pop_back();
                vector<Vector> diff(nb_total_points);
                diff[0]=xe1-xe2;
                xe2=xe1;
                for(size_t i=1; i<nb_total_points; i++)
                {
                    xe1=xe[i];
                    diff[i]=xe1-xe2;
                    xe2=xe1;
                }

                // compute grad
                for(size_t j=4; j<nb_kin_DOF; j++)
                {
                    values[idx]=-2*s_pos*dot(grad.getCol(j).subVector(0,2), diff[1]+diff[0]);
                    idx++;
                }

                for(size_t j=4; j<nb_kin_DOF; j++)
                {
                    values[idx]=2*s_pos*dot(grad.getCol(j).subVector(3,5), diff[1]);
                    idx++;
                }

                for(size_t i=1; i<nb_total_points-1; i++)
                {
                    for(size_t j=4; j<nb_kin_DOF; j++)
                    {
                        values[idx]=2*s_pos*dot(grad.getCol(j).subVector(3*(i-1),3*(i-1)+2), diff[i]);
                        idx++;
                    }

                    for(size_t j=4; j<nb_kin_DOF; j++)
                    {
                        values[idx]=-2*s_pos*dot(grad.getCol(j).subVector(3*i,3*i+2), diff[i+1]+diff[i]);
                        idx++;
                    }

                    for(size_t j=4; j<nb_kin_DOF; j++)
                    {
                        values[idx]=2*s_pos*dot(grad.getCol(j).subVector(3*(i+1),3*(i+1)+2), diff[i+1]);
                        idx++;
                    }
                }
            }*/

            yDebug() << "d jg e2";
        }

        return true;
    }
};


/****************************************************************/
class MobileArmPlanFullNoBaseNoTorsoNoHeaveNLP_CentralDiff : public MobileArmPlanFullNoBaseNoTorsoNoHeaveNLP_ForwardDiff
{
public:
    /****************************************************************/
    MobileArmPlanFullNoBaseNoTorsoNoHeaveNLP_CentralDiff(MobileArmPlanSolver &slv_, int nb_targets=1, int nb_checkPoints=0) :
        MobileArmPlanFullNoBaseNoTorsoNoHeaveNLP_ForwardDiff(slv_, nb_targets, nb_checkPoints)
    {
    }

    /****************************************************************/
    string get_mode() const
    {
        return "full_pose+plan+no_base+no_torso_no_heave+central_diff";
    }

    /****************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                    Ipopt::Index *jCol, Ipopt::Number *values)
    {
        if (values==NULL)
        {
            Ipopt::Index idx=0;
            for (size_t i=0; i<nb_targets; i++)
            {
                // g[0,1] (lower_arm)
                iRow[idx]=10*i+0; jCol[idx]=idx_la[i]+0;idx++;
                iRow[idx]=10*i+1; jCol[idx]=idx_la[i]+0;idx++;
                iRow[idx]=10*i+0; jCol[idx]=idx_la[i]+1;idx++;
                iRow[idx]=10*i+1; jCol[idx]=idx_la[i]+1;idx++;
                iRow[idx]=10*i+0; jCol[idx]=idx_la[i]+2;idx++;
                iRow[idx]=10*i+1; jCol[idx]=idx_la[i]+2;idx++;

                // g[2] (reaching position)
                for (Ipopt::Index col=0; col<3; col++)
                {
                    for (Ipopt::Index j=0; j<3; j++)
                    {
                        iRow[idx]=10*i+2+j; jCol[idx]=col;
                        idx++;
                    }
                }
                for (Ipopt::Index col=idx_ua[i]+1; col<idx_la[i]+3; col++)
                {
                    for (Ipopt::Index j=0; j<3; j++)
                    {
                        iRow[idx]=10*i+2+j; jCol[idx]=col;
                        idx++;
                    }
                }

                // g[3] (cover constraints)
                iRow[idx]=10*i+5; jCol[idx]=idx_ua[i]+1;idx++;
                iRow[idx]=10*i+5; jCol[idx]=idx_ua[i]+2;idx++;

                // g[4] (reaching orientation)
                for (Ipopt::Index j=0; j<3; j++)
                {
                    iRow[idx]=10*i+6+j; jCol[idx]=2;
                    idx++;
                }
                for (Ipopt::Index col=idx_ua[i]+1; col<idx_la[i]+3; col++)
                {
                    for (Ipopt::Index j=0; j<3; j++)
                    {
                        iRow[idx]=10*i+6+j; jCol[idx]=col;
                        idx++;
                    }
                }
            }

            // g[5] (domain boundaries constraints)
            if(domain_constr)
            {
                iRow[idx]=10*nb_targets; jCol[idx]=0;idx++;
                iRow[idx]=10*nb_targets; jCol[idx]=1;
            }
        }
        else
        {
            computeQuantities(x,new_x);

            Ipopt::Number x_dx[n];
            for (Ipopt::Index i=0; i<n; i++)
                x_dx[i]=x[i];

            TripodState d_fw,d_bw;

            Ipopt::Index idx=0;
            for (size_t i=0; i<nb_targets; i++)
            {
                // g[0,1] (lower_arm)
                double e2=hd2-din2[i].p[2];

                x_dx[idx_la[i]+0]=x[idx_la[i]+0]+drho;
                tripod_fkin(2,x_dx,&d_fw,i);
                x_dx[idx_la[i]+0]=x[idx_la[i]+0]-drho;
                tripod_fkin(2,x_dx,&d_bw,i);
                values[idx]=-e2*(d_fw.p[2]-d_bw.p[2])/drho;idx++;
                values[idx]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);idx++;
                x_dx[idx_la[i]+0]=x[idx_la[i]+0];

                x_dx[idx_la[i]+1]=x[idx_la[i]+1]+drho;
                tripod_fkin(2,x_dx,&d_fw,i);
                x_dx[idx_la[i]+1]=x[idx_la[i]+1]-drho;
                tripod_fkin(2,x_dx,&d_bw,i);
                values[idx]=-e2*(d_fw.p[2]-d_bw.p[2])/drho;idx++;
                values[idx]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);idx++;
                x_dx[idx_la[i]+1]=x[idx_la[i]+1];

                x_dx[idx_la[i]+2]=x[idx_la[i]+2]+drho;
                tripod_fkin(2,x_dx,&d_fw,i);
                x_dx[idx_la[i]+2]=x[idx_la[i]+2]-drho;
                tripod_fkin(2,x_dx,&d_bw,i);
                values[idx]=-e2*(d_fw.p[2]-d_bw.p[2])/drho;idx++;
                values[idx]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);idx++;
                x_dx[idx_la[i]+2]=x[idx_la[i]+2];

                // g[2] (base)
                values[idx]=-s_pos;idx++;
                values[idx]=0.0;idx++;
                values[idx]=0.0;idx++;

                values[idx]=0.0;idx++;
                values[idx]=-s_pos;idx++;
                values[idx]=0.0;idx++;

                Vector o(4,0.0);
                o[2] = 1.0;
                o[3] = M_PI/2.0+x[idx_b+2];
                Matrix Ro=axis2dcm(o);
                Vector v=Ro*T[i].getCol(3).subVector(0,3);

                values[idx]=-s_pos*v[0];idx++;
                values[idx]=-s_pos*v[1];idx++;
                values[idx]=0.0;idx++;

                // g[2] (upper_arm)
                Matrix grad=Hb.submatrix(0,2,0,2)*J_[i].submatrix(0,2,0,upper_arm.getDOF()-1);
                for (size_t j=1; j<grad.cols(); j++)
                {
                    for (size_t k=0; k<3; k++)
                    {
                        values[idx]=-s_pos*grad[k][j];
                        idx++;
                    }
                }

                // g[2] (lower_arm)
                Vector xe=Hb*T[i].getCol(3).subVector(0,3);
                xe.pop_back();
                Vector e=xd[i]-xe;
                Vector e_fw,e_bw;
                Matrix M=d1[i].T*H[i];

                x_dx[idx_la[i]+0]=x[idx_la[i]+0]+drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_fw=xd[i]-(Hb*M*d_fw.T*TN).getCol(3).subVector(0,2);
                x_dx[idx_la[i]+0]=x[idx_la[i]+0]-drho;
                d_bw=tripod_fkin(2,x_dx,nullptr,i);
                e_bw=xd[i]-(Hb*M*d_bw.T*TN).getCol(3).subVector(0,2);
                values[idx]=0.5*s_pos*(e_fw[0]-e_bw[0])/drho;idx++;
                values[idx]=0.5*s_pos*(e_fw[1]-e_bw[1])/drho;idx++;
                values[idx]=0.5*s_pos*(e_fw[2]-e_bw[2])/drho;idx++;
                x_dx[idx_la[i]+0]=x[idx_la[i]+0];

                x_dx[idx_la[i]+1]=x[idx_la[i]+1]+drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_fw=xd[i]-(Hb*M*d_fw.T*TN).getCol(3).subVector(0,2);
                x_dx[idx_la[i]+1]=x[idx_la[i]+1]-drho;
                d_bw=tripod_fkin(2,x_dx,nullptr,i);
                e_bw=xd[i]-(Hb*M*d_bw.T*TN).getCol(3).subVector(0,2);
                values[idx]=0.5*s_pos*(e_fw[0]-e_bw[0])/drho;idx++;
                values[idx]=0.5*s_pos*(e_fw[1]-e_bw[1])/drho;idx++;
                values[idx]=0.5*s_pos*(e_fw[2]-e_bw[2])/drho;idx++;
                x_dx[idx_la[i]+1]=x[idx_la[i]+1];

                x_dx[idx_la[i]+2]=x[idx_la[i]+2]+drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_fw=xd[i]-(Hb*M*d_fw.T*TN).getCol(3).subVector(0,2);
                x_dx[idx_la[i]+2]=x[idx_la[i]+2]-drho;
                d_bw=tripod_fkin(2,x_dx,nullptr,i);
                e_bw=xd[i]-(Hb*M*d_bw.T*TN).getCol(3).subVector(0,2);
                values[idx]=0.5*s_pos*(e_fw[0]-e_bw[0])/drho;idx++;
                values[idx]=0.5*s_pos*(e_fw[1]-e_bw[1])/drho;idx++;
                values[idx]=0.5*s_pos*(e_fw[2]-e_bw[2])/drho;idx++;
                x_dx[idx_la[i]+2]=x[idx_la[i]+2];

                // g[3]
                values[idx]=-cover_shoulder_avoidance[0];idx++;
                values[idx]=1.0;idx++;

                // g[4] init
                Vector eo=dcm2axis(Rd[i]*(Rb*T[i]).transposed());
                double theta=eo[3];
                double coef=0.5*theta*cos(theta/2)/sin(theta/2);
                Vector eo_u=eo.subVector(0,2);
                eo*=theta;eo.pop_back();

                // g[4] base
                Vector w(3,0.0);
                w[2]=1.0;
                Vector eo_d=dot(w,eo_u)*eo_u;
                Vector grado=s_ang*(-1*eo_d+coef*(eo_d-w)+0.5*theta*cross(w,eo_u));
                values[idx]=grado[0];idx++;
                values[idx]=grado[1];idx++;
                values[idx]=grado[2];idx++;

                // g[4] (upper_arm)
                Matrix grado_ua=Rb.submatrix(0,2,0,2)*J_[i].submatrix(3,5,0,upper_arm.getDOF()-1);
                for (size_t j=1; j<grado_ua.cols(); j++)
                {
                    w=grado_ua.getCol(j);
                    eo_d=dot(w,eo_u)*eo_u;
                    grado=s_ang*(-1*eo_d+coef*(eo_d-w)+0.5*theta*cross(w,eo_u));
                    values[idx]=grado[0];idx++;
                    values[idx]=grado[1];idx++;
                    values[idx]=grado[2];idx++;
                }

                // g[4] (lower_arm)
                Vector e_fwo, e_bwo;

                x_dx[idx_la[i]+0]=x[idx_la[i]+0]+drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_fwo=dcm2axis(Rd[i]*(Rb*M*d_fw.T*TN).transposed()); e_fwo*=e_fwo[3]; e_fwo.pop_back();
                x_dx[idx_la[i]+0]=x[idx_la[i]+0]-drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_bwo=dcm2axis(Rd[i]*(Rb*M*d_fw.T*TN).transposed()); e_bwo*=e_bwo[3]; e_bwo.pop_back();
                values[idx]=s_ang*0.5*(e_fwo[0]-e_bwo[0])/drho;idx++;
                values[idx]=s_ang*0.5*(e_fwo[1]-e_bwo[1])/drho;idx++;
                values[idx]=s_ang*0.5*(e_fwo[2]-e_bwo[2])/drho;idx++;
                x_dx[idx_la[i]+0]=x[idx_la[i]+0];

                x_dx[idx_la[i]+1]=x[idx_la[i]+1]+drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_fwo=dcm2axis(Rd[i]*(Rb*M*d_fw.T*TN).transposed()); e_fwo*=e_fwo[3]; e_fwo.pop_back();
                x_dx[idx_la[i]+1]=x[idx_la[i]+1]-drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_bwo=dcm2axis(Rd[i]*(Rb*M*d_fw.T*TN).transposed()); e_bwo*=e_bwo[3]; e_bwo.pop_back();
                values[idx]=s_ang*0.5*(e_fwo[0]-e_bwo[0])/drho;idx++;
                values[idx]=s_ang*0.5*(e_fwo[1]-e_bwo[1])/drho;idx++;
                values[idx]=s_ang*0.5*(e_fwo[2]-e_bwo[2])/drho;idx++;
                x_dx[idx_la[i]+1]=x[idx_la[i]+1];

                x_dx[idx_la[i]+2]=x[idx_la[i]+2]+drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_fwo=dcm2axis(Rd[i]*(Rb*M*d_fw.T*TN).transposed()); e_fwo*=e_fwo[3]; e_fwo.pop_back();
                x_dx[idx_la[i]+2]=x[idx_la[i]+2]-drho;
                d_fw=tripod_fkin(2,x_dx,nullptr,i);
                e_bwo=dcm2axis(Rd[i]*(Rb*M*d_fw.T*TN).transposed()); e_bwo*=e_bwo[3]; e_bwo.pop_back();
                values[idx]=s_ang*0.5*(e_fwo[0]-e_bwo[0])/drho;idx++;
                values[idx]=s_ang*0.5*(e_fwo[1]-e_bwo[1])/drho;idx++;
                values[idx]=s_ang*0.5*(e_fwo[2]-e_bwo[2])/drho;idx++;
                x_dx[idx_la[i]+2]=x[idx_la[i]+2];
            }

            // g[5] (domain boundaries constraints)
            if(domain_constr)
            {
                Vector v(2);
                v[0]=x[idx_b+0]+drho;
                v[1]=x[idx_b+1];
                double df = distanceFromDomain(domain_poly, v);
                v[0]=x[idx_b+0]-drho;
                double db = distanceFromDomain(domain_poly, v);
                values[idx]=0.5*(df-db)/drho;idx++;

                v[0]=x[idx_b+0];
                v[1]=x[idx_b+1]+drho;
                df = distanceFromDomain(domain_poly, v);
                v[1]=x[idx_b+1]-drho;
                db = distanceFromDomain(domain_poly, v);
                values[idx]=0.5*(df-db)/drho;
            }
        }

        return true;
    }
};


