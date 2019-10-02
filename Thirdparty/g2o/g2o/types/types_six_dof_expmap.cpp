// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "types_six_dof_expmap.h"

#include "../core/factory.h"
#include "../stuff/macros.h"
#include <iostream>

namespace g2o {

using namespace std;


Vector2d project2d(const Vector3d& v)  {
    Vector2d res;
    res(0) = v(0)/v(2);
    res(1) = v(1)/v(2);
    return res;
}

Vector3d unproject2d(const Vector2d& v)  {
    Vector3d res;
    res(0) = v(0);
    res(1) = v(1);
    res(2) = 1;
    return res;
}

VertexSE3Expmap::VertexSE3Expmap() : BaseVertex<6, SE3Quat>() {
}

bool VertexSE3Expmap::read(std::istream& is) {
    Vector7d est;
    for (int i=0; i<7; i++)
        is  >> est[i];
    SE3Quat cam2world;
    cam2world.fromVector(est);
    setEstimate(cam2world.inverse());
    return true;
}

bool VertexSE3Expmap::write(std::ostream& os) const {
    SE3Quat cam2world(estimate().inverse());
    for (int i=0; i<7; i++)
        os << cam2world[i] << " ";
    return os.good();
}


EdgeSE3ProjectXYZ::EdgeSE3ProjectXYZ() : BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>() {
}

bool EdgeSE3ProjectXYZ::read(std::istream& is){
    for (int i=0; i<2; i++){
        is >> _measurement[i];
    }
    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++) {
            is >> information()(i,j);
            if (i!=j)
                information()(j,i)=information()(i,j);
        }
    return true;
}

bool EdgeSE3ProjectXYZ::write(std::ostream& os) const {

    for (int i=0; i<2; i++){
        os << measurement()[i] << " ";
    }

    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++){
            os << " " <<  information()(i,j);
        }
    return os.good();
}

void EdgeSE3ProjectXYZ::setExtrinsic(SE3Quat mVertexSE3, Matrix<double,6,6> adj)
{
    mVertexSE3CamExtAdj =  adj;
    mVertexSE3CamExt = mVertexSE3;
}

void EdgeSE3ProjectXYZ::computeError(){
    const VertexSBAPointXYZ* pt3 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]); //World Coordinate Points
    const VertexSE3Expmap* TMCS = static_cast<const VertexSE3Expmap*>(_vertices[1]); // Multi Camera System Pose
    Vector2d obs(_measurement);
    _error = obs-cam_project(mVertexSE3CamExt.map( TMCS->estimate().map(pt3->estimate())));
}

bool EdgeSE3ProjectXYZ::isDepthPositive()
{
    const VertexSE3Expmap* TMCS = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* pt3 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    return mVertexSE3CamExt.map( TMCS->estimate().map( pt3->estimate())) (2)>0.0;
}

void EdgeSE3ProjectXYZ::linearizeOplus() {
    const  VertexSBAPointXYZ* pt3 = static_cast<VertexSBAPointXYZ*>(_vertices[0]);   //World Coordinate Points
    const  VertexSE3Expmap * TMCS = static_cast<VertexSE3Expmap *>(_vertices[1]);    // Multi Camera System Pose

    SE3Quat Tmcs(TMCS->estimate());
    Vector3d xyz = pt3->estimate();
    Vector3d xyz_trans = mVertexSE3CamExt.map(Tmcs.map(xyz));

    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];


    Matrix<double,2,3> tmp;
    tmp(0,0) = fx;
    tmp(0,1) = 0;
    tmp(0,2) = -x/z*fx;

    tmp(1,0) = 0;
    tmp(1,1) = fy;
    tmp(1,2) = -y/z*fy;


    Matrix<double,3,6> J3;
    J3(0,0) = 0 ; J3(0,1)  = z ; J3(0,2) =  -y ; J3(0,3) =  1 ;  J3(0,4) =  0  ;  J3(0,5) =  0;
    J3(1,0) = -z ; J3(1,1) = 0 ; J3(1,2) = x ; J3(1,3) =  0 ;  J3(1,4) =  1  ;  J3(1,5) =  0;
    J3(2,0) = y ; J3(2,1)  = -x ; J3(2,2) =  0 ; J3(2,3) =  0 ;  J3(2,4) =  0  ;  J3(2,5) =  1;

    Matrix<double,2,6> jacobianMCS_;
    jacobianMCS_ = -1./z*tmp*J3*mVertexSE3CamExtAdj;
    _jacobianOplusXj = jacobianMCS_ ;


    SE3Quat T = mVertexSE3CamExt*Tmcs ;
    Matrix<double,2,3> Jpoints;
    Jpoints = -1./z*tmp * T.rotation().toRotationMatrix();
    _jacobianOplusXi  = Jpoints;

}

Vector2d EdgeSE3ProjectXYZ::cam_project(const Vector3d & trans_xyz) const{
    Vector2d proj = project2d(trans_xyz);
    Vector2d res;
    res[0] = proj[0]*fx + cx;
    res[1] = proj[1]*fy + cy;
    return res;
}




bool EdgeSE3ProjectXYZOnlyPose::read(std::istream& is){
    for (int i=0; i<2; i++){
        is >> _measurement[i];
    }
    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++) {
            is >> information()(i,j);
            if (i!=j)
                information()(j,i)=information()(i,j);
        }
    return true;
}

bool EdgeSE3ProjectXYZOnlyPose::write(std::ostream& os) const {

    for (int i=0; i<2; i++){
        os << measurement()[i] << " ";
    }

    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++){
            os << " " <<  information()(i,j);
        }
    return os.good();
}

void EdgeSE3ProjectXYZOnlyPose::setExtrinsic(SE3Quat mVertexSE3, Matrix<double,6,6> adj)
{
    mVertexSE3CamExtAdj =  adj;
    mVertexSE3CamExt = mVertexSE3;
}

void EdgeSE3ProjectXYZOnlyPose::computeError()  {
    const VertexSE3Expmap* TMCS = static_cast<const VertexSE3Expmap*>(_vertices[0]); // Multi Camera System Pose
    Vector2d obs(_measurement);
    _error = obs-cam_project(mVertexSE3CamExt.map( TMCS->estimate().map(Xw)));
}

bool EdgeSE3ProjectXYZOnlyPose::isDepthPositive() {
    const VertexSE3Expmap* TMCS = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    return mVertexSE3CamExt.map( TMCS->estimate().map( Xw)) (2)>0.0;
}



void EdgeSE3ProjectXYZOnlyPose::linearizeOplus()
{
    const  VertexSE3Expmap * TMCS = static_cast<VertexSE3Expmap *>(_vertices[0]);    // Multi Camera System Pose
    SE3Quat Tmcs(TMCS->estimate());
    Vector3d xyz_trans = mVertexSE3CamExt.map(Tmcs.map(Xw));

    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];

    Matrix<double,2,3> tmp;
    tmp(0,0) = fx;
    tmp(0,1) = 0;
    tmp(0,2) = -x/z*fx;

    tmp(1,0) = 0;
    tmp(1,1) = fy;
    tmp(1,2) = -y/z*fy;

    Matrix<double,3,6> J3;
    J3(0,0) = 0 ; J3(0,1)  = z ; J3(0,2) =  -y ; J3(0,3) =  1 ;  J3(0,4) =  0  ;  J3(0,5) =  0;
    J3(1,0) = -z ; J3(1,1) = 0 ; J3(1,2) = x ; J3(1,3) =  0 ;  J3(1,4) =  1  ;  J3(1,5) =  0;
    J3(2,0) = y ; J3(2,1)  = -x ; J3(2,2) =  0 ; J3(2,3) =  0 ;  J3(2,4) =  0  ;  J3(2,5) =  1;

    Matrix<double,2,6> jacobianMCS_;
    jacobianMCS_ = -1./z*tmp*J3*mVertexSE3CamExtAdj;
    _jacobianOplusXi = jacobianMCS_;

}

Vector2d EdgeSE3ProjectXYZOnlyPose::cam_project(const Vector3d & trans_xyz) const{
    Vector2d proj = project2d(trans_xyz);
    Vector2d res;
    res[0] = proj[0]*fx + cx;
    res[1] = proj[1]*fy + cy;
    return res;
}





} // end namespace
