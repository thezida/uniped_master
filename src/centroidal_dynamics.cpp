/*
 * centroidal_dynamics.cpp
 *
 *  Created on: Mar 6, 2019
 *      Author: Ivo Vatavuk
 */

#include "centroidal_dynamics.h"


void CentroidalDynamics::CalcCMM (	Model *rbdlModel, MatNd H, SpatialTransform X01,
                                          VecNd Cdq, MatNd &AG, MatNd &dAGdq ) {

  MatNd I1c = H.block(0,0,6,6);
  double mass = I1c(5,5);
  Vec3d pG1(1/mass*I1c(2,4), 1/mass*I1c(0,5), 1/mass*I1c(1,3));
  //SpatialTransform XG1(X01.E.transpose(), pG1);
  SpatialTransform XG1(X01.E, -X01.E.transpose()*pG1);

  AG = XG1.toMatrix().transpose() * H.block(0,0,6,rbdlModel->dof_count);
  dAGdq = XG1.toMatrix().transpose() * Cdq.segment(0,6);
}

void CentroidalDynamics::CalcCOMfromCMM (	Model *rbdlModel, MatNd AG, MatNd dAGdq,
                                                 SpatialTransform X01, VecNd Q,
                                                 SpatialVector v0, VecNd QDot,
                                                 MatNd H,
                                                 SpatialVector &h, Vec3d &drG, Vec3d &rG) {

  VecNd Q_modified = VecNd::Zero (rbdlModel->dof_count);
  VecNd QDot_modified = VecNd::Zero (rbdlModel->dof_count);
  SpatialVector zero_6d = SpatialVector::Zero();
  QDot_modified << v0, QDot;
  Q_modified << zero_6d, Q;


  Matrix3d R01 = X01.E;

  h = AG*QDot_modified;


  MatNd I1c = H.block(0,0,6,6);
  double mass = I1c(5,5);
  Vec3d pG1 (1/mass*I1c(2,4), 1/mass*I1c(0,5), 1/mass*I1c(1,3));

  SpatialTransform XG1(X01.E, -X01.E.transpose()*pG1);

  MatNd IG = XG1.toMatrix().transpose() * I1c * XG1.toMatrix();

  SpatialVector vG = IG.inverse()*h;
  drG = vG.segment(3,3);

  SpatialTransform X10 = X01.inverse();
  MatNd T10(4,4);
  T10 << X10.E, -X10.E*X10.r, 0, 0, 0, 1;
  Vector4d pG1_4d;
  pG1_4d << pG1, 1;

  rG = (T10*pG1_4d).segment(0,3);
}




