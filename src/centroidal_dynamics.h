//
// Created by martindzida on 21.01.20..
//

#ifndef SRC_CENTROIDAL_DYNAMICS_H
#define SRC_CENTROIDAL_DYNAMICS_H

#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

typedef MatrixNd MatNd;
typedef VectorNd VecNd;
typedef Vector3d Vec3d;

class CentroidalDynamics {
public:
  static void CalcCMM (	Model *rbdlModel, MatNd H, SpatialTransform X01,
                               VecNd Cdq, MatNd &AG, MatNd &dAGdq );

  static void CalcCOMfromCMM (	Model *rbdlModel, MatNd AG, MatNd dAGdq,
                                      SpatialTransform X01, VecNd Q,
                                      SpatialVector v0, VecNd QDot,
                                      MatNd H,
                                      SpatialVector &h, Vec3d &drG, Vec3d &rG);
};

#endif //SRC_CENTROIDAL_DYNAMICS_H
