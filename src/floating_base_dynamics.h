//
// Created by martindzida on 21.01.20..
//

#ifndef SRC_FLOATING_BASE_DYNAMICS_H
#define SRC_FLOATING_BASE_DYNAMICS_H

#include <rbdl/rbdl.h>
#include <vector>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

typedef MatrixNd MatNd;
typedef VectorNd VecNd;

class FloatingBaseDynamics {
public:
    static void FloatingBaseCompositeRigidBodyAlgorithm ( Model *rbdlModel,
                                                          VecNd Q, MatNd &H );

    static void FloatingBaseNonlinearEffects ( Model *rbdlModel,
                                               SpatialTransform X01, VecNd Q,
                                               SpatialVector v0, VecNd QDot,
                                               VecNd &Cdq,
                                               std::vector<SpatialVector> *f_ext );

    static void FloatingBaseNonlinearEffectsOneForce (	Model *rbdlModel,
                                                          SpatialTransform X01,
                                                          VecNd Q, SpatialVector v0,
                                                          VecNd QDot, SpatialVector f,
                                                          int body_number, VecNd &Cdq );

    static void FloatingBaseInverseDynamics ( Model *rbdlModel,
                                              SpatialTransform X01, VecNd Q,
                                              SpatialVector v0, VecNd QDot,
                                              VecNd QDDot, VecNd &Tau, VecNd &a0,
                                              std::vector<SpatialVector> *f_ext );

    static void FloatingBaseInverseDynamicsOneForce ( Model *rbdlModel,
                                                      SpatialTransform X01,
                                                      VecNd Q, SpatialVector v0,
                                                      VecNd QDot, VecNd QDDot,
                                                      SpatialVector f,
                                                      int body_number,
                                                      VecNd &Tau, VecNd &a0 );
};

#endif //SRC_FLOATING_BASE_DYNAMICS_H
