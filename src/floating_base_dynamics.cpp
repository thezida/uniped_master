/*
 * floating_base_dynamics.cpp
 *
 *  Created on: Mar 5, 2019
 *      Author: Ivo Vatavuk
 */
#include "floating_base_dynamics.h"

void FloatingBaseDynamics::FloatingBaseCompositeRigidBodyAlgorithm ( Model *rbdlModel,
                                                                     VecNd Q, MatNd &H )
{
    VecNd Q_modified = VecNd::Zero (rbdlModel->dof_count);
    VecNd QDot_modified = VecNd::Zero (rbdlModel->dof_count);
    VecNd QDDot_modified = VecNd::Zero (rbdlModel->dof_count);

    SpatialVector zero_6d = SpatialVector::Zero();
    Q_modified << zero_6d, Q;

    UpdateKinematics(*rbdlModel, Q_modified, QDot_modified, QDDot_modified);

    CompositeRigidBodyAlgorithm (*rbdlModel, Q_modified, H, false);
}
/*
void FloatingBaseDynamics::UpdateKinematics(Model *rbdlModel, VecNd Q, VecNd v0, VecNd QDot) {
	VecNd Q_modified = VecNd::Zero (rbdlModel->dof_count);
	VecNd QDot_modified = VecNd::Zero (rbdlModel->dof_count);
	VecNd QDDot_modified = VecNd::Zero (rbdlModel->dof_count);

	SpatialVector zero_6d = SpatialVector::Zero();
	Q_modified << zero_6d, Q;
	QDot_modified << v0, QDot;

	UpdateKinematics(*rbdlModel, Q_modified, QDot_modified, QDDot_modified);
}
*/
void FloatingBaseDynamics::FloatingBaseNonlinearEffects ( Model *rbdlModel,
                                                          SpatialTransform X01, VecNd Q,
                                                          SpatialVector v0, VecNd QDot,
                                                          VecNd &Cdq,
                                                          std::vector<SpatialVector> *f_ext )
{

    VecNd Q_modified = VecNd::Zero (rbdlModel->dof_count);
    VecNd QDot_modified = VecNd::Zero (rbdlModel->dof_count);
    VecNd QDDot_modified = VecNd::Zero (rbdlModel->dof_count);

    SpatialVector zero_6d = SpatialVector::Zero();
    QDot_modified << v0, QDot;
    Q_modified << zero_6d, Q;

    UpdateKinematics(*rbdlModel, Q_modified, QDot_modified, QDDot_modified);

    SpatialVector a0_spatial (	0.,
                                  0.,
                                  0.,
                                  rbdlModel->gravity[0],
                                  rbdlModel->gravity[1],
                                  rbdlModel->gravity[2] );

    SpatialVector ag0 = X01.apply(a0_spatial);

    // Reset the velocity of the root body
    rbdlModel->v[0].setZero();
    rbdlModel->a[0] = -ag0;

    for (unsigned int i = 1; i < rbdlModel->mJointUpdateOrder.size(); i++) {
        jcalc (*rbdlModel, rbdlModel->mJointUpdateOrder[i], Q_modified, QDot_modified);
    }

    for (unsigned int i = 1; i < rbdlModel->mBodies.size(); i++) {
        if (rbdlModel->lambda[i] == 0) {
            rbdlModel->v[i] = rbdlModel->v_J[i];
            rbdlModel->a[i] = rbdlModel->X_lambda[i].apply(-ag0);
        }
        else {
            rbdlModel->v[i] = rbdlModel->X_lambda[i].apply(rbdlModel->v[rbdlModel->lambda[i]]) + rbdlModel->v_J[i];
            if(i>6){
                rbdlModel->c[i] = rbdlModel->c_J[i] + crossm(rbdlModel->v[i],rbdlModel->v_J[i]);
            }
            else {
                rbdlModel->c[i] = rbdlModel->c_J[i];
            }
            rbdlModel->a[i] = rbdlModel->X_lambda[i].apply(rbdlModel->a[rbdlModel->lambda[i]]) + rbdlModel->c[i];
        }

        if (!rbdlModel->mBodies[i].mIsVirtual) {
            rbdlModel->f[i] = rbdlModel->I[i] * rbdlModel->a[i] + crossf(rbdlModel->v[i],rbdlModel->I[i] * rbdlModel->v[i]);
            if (f_ext != NULL && (*f_ext)[i] != SpatialVector::Zero()) {
                //rbdlModel->f[i] -= rbdlModel->X_base[i].toMatrixAdjoint() * (*f_ext)[i];
                rbdlModel->f[i] -= (*f_ext)[i];
            }
        }
        else {
            rbdlModel->f[i].setZero();
        }
    }

    for (unsigned int i = rbdlModel->mBodies.size() - 1; i > 0; i--) {
        if(rbdlModel->mJoints[i].mJointType != JointTypeCustom){
            if (rbdlModel->mJoints[i].mDoFCount == 1) {
                Cdq[rbdlModel->mJoints[i].q_index] = rbdlModel->S[i].dot(rbdlModel->f[i]);
            }
            else if (rbdlModel->mJoints[i].mDoFCount == 3) {
                Cdq.block<3,1>(rbdlModel->mJoints[i].q_index, 0) = rbdlModel->multdof3_S[i].transpose() * rbdlModel->f[i];
            }
        } else if(rbdlModel->mJoints[i].mJointType == JointTypeCustom) {
            unsigned int k = rbdlModel->mJoints[i].custom_joint_index;
            Cdq.block(rbdlModel->mJoints[i].q_index,0,rbdlModel->mCustomJoints[k]->mDoFCount, 1) = rbdlModel->mCustomJoints[k]->S.transpose() * rbdlModel->f[i];
        }

        if (rbdlModel->lambda[i] != 0) {
            rbdlModel->f[rbdlModel->lambda[i]] = rbdlModel->f[rbdlModel->lambda[i]] + rbdlModel->X_lambda[i].applyTranspose(rbdlModel->f[i]);
        }
    }
}

void FloatingBaseDynamics::FloatingBaseNonlinearEffectsOneForce (	Model *rbdlModel,
                                                                     SpatialTransform X01,
                                                                     VecNd Q, SpatialVector v0,
                                                                     VecNd QDot, SpatialVector f,
                                                                     int body_number, VecNd &Cdq )
{
    std::vector<SpatialVector> f_vector;

    for(int i = 0; i < rbdlModel->mBodies.size(); i++) {
        if (i == body_number)
            f_vector.push_back(f);
        else
            f_vector.push_back(VecNd::Zero (6));
    }

    std::vector<SpatialVector> *f_ext = &f_vector;


    FloatingBaseNonlinearEffects(rbdlModel, X01, Q, v0, QDot, Cdq, f_ext);
}

void FloatingBaseDynamics::FloatingBaseInverseDynamics ( Model *rbdlModel,
                                                         SpatialTransform X01, VecNd Q,
                                                         SpatialVector v0, VecNd QDot,
                                                         VecNd QDDot, VecNd &Tau, VecNd &a0,
                                                         std::vector<SpatialVector> *f_ext )
{

    VecNd QDDot_modified = VecNd::Zero (rbdlModel->dof_count);
    SpatialVector zero_6d = SpatialVector::Zero();
    QDDot_modified << zero_6d, QDDot;

    MatNd H = MatNd::Zero(rbdlModel->dof_count, rbdlModel->dof_count);
    VecNd Cdq = VecNd::Zero(rbdlModel->dof_count);

    //calculate H via CRBA
    FloatingBaseCompositeRigidBodyAlgorithm(rbdlModel, Q, H);

    //calculate Cdq via RNEA
    FloatingBaseNonlinearEffects(rbdlModel, X01, Q, v0, QDot, Cdq, f_ext);


    MatNd I0c = H.block(0,0,6,6);
    MatNd F = H.block(0,6,6,rbdlModel->dof_count - 6);
    VecNd p0c = Cdq.segment(0,6);

    //calculate a0 from the fact that first 6 values of Tau are equal to zero
    a0 = -I0c.inverse()*(p0c + F*QDDot);

    for(int i = 0; i < 6; i++) {
        QDDot_modified[i] = a0[i];
    }
    //calculate Tau
    Tau = H*QDDot_modified + Cdq;
}

void FloatingBaseDynamics::FloatingBaseInverseDynamicsOneForce ( Model *rbdlModel,
                                                                 SpatialTransform X01,
                                                                 VecNd Q, SpatialVector v0,
                                                                 VecNd QDot, VecNd QDDot,
                                                                 SpatialVector f,
                                                                 int body_number,
                                                                 VecNd &Tau, VecNd &a0 )
{
    std::vector<SpatialVector> f_vector;

    for(int i = 0; i < rbdlModel->mBodies.size(); i++) {
        if (i == body_number)
            f_vector.push_back(f);
        else
            f_vector.push_back(VecNd::Zero (6));
    }

    std::vector<SpatialVector> *f_ext = &f_vector;


    FloatingBaseInverseDynamics(rbdlModel, X01, Q, v0, QDot, QDDot, Tau, a0, f_ext);
}
