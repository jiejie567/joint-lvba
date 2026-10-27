#pragma once
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>
#include <array>
#include <set>

namespace lvba {
// Experimental shared-trajectory LiDAR/visual BA, followed by fixed-structure BA.
// No SDK pose or relative-motion residuals. SDK supplies initialization only.
struct JointObservation { int camera, point; double u,v; };
struct JointLaser { int a,b,plane,scan; double alpha; Eigen::Vector3d p; };
struct JointPlane { std::array<double,3> n; double d; std::set<int> scans; };
struct JointReprojection {
 double u,v,fx,fy,cx,cy;
 template<class T> bool operator()(const T* q,const T* c,const T* X,T* r)const {
  T qi[4]={q[0],-q[1],-q[2],-q[3]},x[3]={X[0]-c[0],X[1]-c[1],X[2]-c[2]},p[3];ceres::QuaternionRotatePoint(qi,x,p);
  // Input observations are prefiltered for positive depth; a smooth floor
  // keeps a trial step finite without turning points behind the camera free.
  T z=T(.5)*(p[2]+ceres::sqrt(p[2]*p[2]+T(1e-8)))+T(1e-4);
  r[0]=T(fx)*p[0]/z+T(cx)-T(u);r[1]=T(fy)*p[1]/z+T(cy)-T(v);return true;
 }
};
struct JointLaserPlane {
 Eigen::Vector3d p;double alpha,sigma;
 template<class T> bool operator()(const T* qa,const T* ca,const T* qb,const T* cb,const T* n,const T* d,T* r)const {
  T q[4],norm=T(0);for(int j=0;j<4;++j){q[j]=T(1-alpha)*qa[j]+T(alpha)*qb[j];norm+=q[j]*q[j];}
  norm=ceres::sqrt(norm);for(int j=0;j<4;++j)q[j]/=norm;
  T x[3]={T(p.x()),T(p.y()),T(p.z())},w[3];ceres::QuaternionRotatePoint(q,x,w);
  r[0]=d[0];for(int j=0;j<3;++j)r[0]+=n[j]*(w[j]+T(1-alpha)*ca[j]+T(alpha)*cb[j]);r[0]/=T(sigma);return true;
 }
};
struct JointLandmarkPlane {
 template<class T>bool operator()(const T* x,const T* n,const T* d,T* r)const {r[0]=(n[0]*x[0]+n[1]*x[1]+n[2]*x[2]+d[0])/T(.04);return true;}
};


} // namespace lvba
