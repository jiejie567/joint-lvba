#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <set>
#include <iostream>
#include "joint_costs.hpp"
using namespace lvba;

bool check(ceres::CostFunction*cost,std::vector<std::vector<double>> p){
 std::vector<double*> ptrs;std::vector<std::vector<double>> jac;std::vector<double*> jp;
 int n=cost->num_residuals();for(size_t j=0;j<p.size();++j){ptrs.push_back(p[j].data());jac.emplace_back(n*p[j].size());}for(auto&j:jac)jp.push_back(j.data());
 std::vector<double> r(n),plus(n),minus(n);cost->Evaluate(ptrs.data(),r.data(),jp.data());double worst=0;
 for(size_t j=0;j<p.size();++j)for(size_t k=0;k<p[j].size();++k){double old=p[j][k],h=1e-6;p[j][k]=old+h;cost->Evaluate(ptrs.data(),plus.data(),nullptr);p[j][k]=old-h;cost->Evaluate(ptrs.data(),minus.data(),nullptr);p[j][k]=old;for(int z=0;z<n;++z){double numeric=(plus[z]-minus[z])/(2*h),analytic=jac[j][z*p[j].size()+k];worst=std::max(worst,std::abs(numeric-analytic)/(1+std::abs(numeric)));}}
 delete cost;std::cout<<"Jacobian relative error "<<worst<<'\n';return worst<3e-5;
}
int main(){
 double angle=.08;std::vector<double> q={std::cos(angle/2),0,std::sin(angle/2),0},q2={1,0,0,0},c={.1,.2,.3},c2={.2,.3,.4},x={1,.5,3},n={0,0,1},d={-3};
 bool good=check(new ceres::AutoDiffCostFunction<JointReprojection,2,4,3,3>(new JointReprojection{500,500,406,406,686,597}),{q,c,x});
 good &= check(new ceres::AutoDiffCostFunction<JointLaserPlane,1,4,3,4,3,3,1>(new JointLaserPlane{Eigen::Vector3d(1,.2,2),.37,.03}),{q,c,q2,c2,n,d});
 good &= check(new ceres::AutoDiffCostFunction<JointLandmarkPlane,1,3,3,1>(new JointLandmarkPlane),{x,n,d});
 JointReprojection reproj{500,500,406,406,686,597};double before[2],after[2];reproj(q.data(),c.data(),x.data(),before);
 Eigen::Quaterniond G(Eigen::AngleAxisd(.32,Eigen::Vector3d(1,2,3).normalized())),Q(q[0],q[1],q[2],q[3]);Eigen::Vector3d t(.4,.8,-.2),C(c[0],c[1],c[2]),X(x[0],x[1],x[2]);Q=G*Q;C=G*C+t;X=G*X+t;std::array<double,4> nq={Q.w(),Q.x(),Q.y(),Q.z()};reproj(nq.data(),C.data(),X.data(),after);good &= std::hypot(before[0]-after[0],before[1]-after[1])<1e-8;
 JointLaserPlane laser{Eigen::Vector3d(1,.2,2),.37,.03};double rb,ra;laser(q.data(),c.data(),q2.data(),c2.data(),n.data(),d.data(),&rb);Eigen::Quaterniond Q2=G*Eigen::Quaterniond(q2[0],q2[1],q2[2],q2[3]);Eigen::Vector3d C2=G*Eigen::Vector3d(c2[0],c2[1],c2[2])+t,N=G*Eigen::Vector3d(0,0,1);double D=d[0]-N.dot(t);std::array<double,4> nq2={Q2.w(),Q2.x(),Q2.y(),Q2.z()};laser(nq.data(),C.data(),nq2.data(),C2.data(),N.data(),&D,&ra);good &= std::abs(rb-ra)<1e-8;
 std::cout<<(good?"PASS Jacobians and global-gauge invariance":"FAIL")<<std::endl;return good?0:1;
}
