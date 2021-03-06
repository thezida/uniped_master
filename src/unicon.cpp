#include "gazebo_msgs/ApplyJointEffort.h"
#include "gazebo_msgs/GetJointProperties.h"
#include "gazebo_msgs/LinkStates.h"

/*
#include <sdformat-6.2/sdf/system_util.hh>
#include <sdformat-6.2/sdf/Exception.hh>
#include <sdformat-6.2/sdf/Assert.hh>
#include <sdformat-6.2/sdf/sdf.hh>
*/

//#include "gazebo-9/gazebo/transport/transport.hh"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <rbdl/rbdl.h>

#include "centroidal_dynamics.h"
#include "floating_base_dynamics.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

void readLinkStates(const gazebo_msgs::LinkStates::ConstPtr &msg) {
  std::cout << "New message!!" << std::endl;
  for (const std::string &name : msg->name) {
    std::cout << name << std::endl;
  }

  for (const geometry_msgs::Pose pose : msg->pose) {
    std::cout << pose.position.x << " " << pose.position.y << " "
              << pose.position.z << std::endl;
  }
}

Model *initModel();

int main(int argc, char **argv) {

  // init node
  ros::init(argc, argv, "unicon");

  ros::NodeHandle n;

  // init model
  Model *uniped_model = initModel();
  std::string joints[] = {"hip", "knee", "ankle"};

  // Suscribe and publish
  //gazebo::transport::Publisher gazebo_world_pub =
  //    n.advertise<gazebo::msgs::WorldControl>("~/world_control");

  // ros::Subscriber sub = n.subscribe("gazebo/link_states", 1000,
  // readLinkStates);
  ros::ServiceClient control_client =
      n.serviceClient<gazebo_msgs::ApplyJointEffort>(
          "/gazebo/apply_joint_effort");
  ros::ServiceClient read_client =
      n.serviceClient<gazebo_msgs::GetJointProperties>(
          "/gazebo/get_joint_properties");

  // init joint variables
  std::map<std::string, double> qs;   // last joint position
  std::map<std::string, double> dqs;  // last joint speed
  std::map<std::string, double> ddqs; // last joint acceleration
  for (std::string j : joints)
    qs[j] = 0;
  for (std::string j : joints)
    dqs[j] = 0;
  for (std::string j : joints)
    ddqs[j] = 0;

  // init joint dynamics vectors
  VectorNd Q = VectorNd::Zero(3);   // vector of joint variables 3
  VectorNd dQ = VectorNd::Zero(3);  // vector of joint speed 3
  VectorNd ddQ = VectorNd::Zero(3); // acc calculated using derivative
  VectorNd tau;                     // tau calculated using inverse dynmics
  VectorNd a0;

  // setup simulation parameters
  double t_max = 10*60;          // secs
  double N = t_max * 100;      // total inputs = t_max * inputs per sec
  double dt = t_max / (N - 1); // give input every dt
  long long iter = 0;
  double freq = 1 / dt;
  double acc_amp = 10;
  double delta_t = dt;
  double omega = 0.1;

  // time management
  ros::Time start = ros::Time::now();
  ros::Rate loop_rate(freq);

  // loop
  while (ros::ok()) {
    std::cout<<"NEW LOOP"<<std::endl;
    // if simulation still runs do...
    if (iter * dt < t_max) {
      // calculate elapsed seconds
      ros::Time end = ros::Time::now();
      ros::Duration elapsed_seconds = end - start;
      start = end;
      delta_t = elapsed_seconds.toSec();

      for (int i = 0; i < (uniped_model->dof_count - 6); i++) {
        // read joints positions and rates
        gazebo_msgs::GetJointProperties read_joints_srv;
        read_joints_srv.request.joint_name = joints[i];
        read_client.call(read_joints_srv);

        if (read_joints_srv.response.position.size() == 0)
          break;
        double pose = read_joints_srv.response.position[0];
        double rate = read_joints_srv.response.rate[0];

        // calculate derivative of rate = acceleration
        ddqs[joints[i]] = (rate - dqs[joints[i]]) / dt;
        dqs[joints[i]] = rate;
        qs[joints[i]] = pose;

        // set vectors for inverse dynamics
        Q[i] = pose;
        dQ[i] = rate;
        ddQ[i] = acc_amp * sin(omega * iter * dt);
      }
      // calculate desired torques
      // RigidBodyDynamics::InverseDynamics(*uniped_model, Q, dQ, ddQ, tau);

      // calculate desired torques for floating base robot
      SpatialTransform X01 = SpatialTransform(
          Matrix3d(1, 0, 0, 0, 1, 0, 0, 0, 1), Vector3d(0, 0, 0));
      SpatialVector v0 = SpatialVector(0, 0, 0, 0, 0, 0);
      std::vector<SpatialVector> *f_ext;
      f_ext = NULL;
      FloatingBaseDynamics::FloatingBaseInverseDynamics(
          uniped_model, X01, Q, v0, dQ, ddQ, tau, a0, f_ext);

      // set desired torques
      for (int i = 0; i < (uniped_model->dof_count - 6); i++) {
        gazebo_msgs::ApplyJointEffort set_torque_srv;
        set_torque_srv.request.joint_name = joints[i];
        set_torque_srv.request.start_time.nsec = 0;
        set_torque_srv.request.start_time.sec = 0;
        set_torque_srv.request.effort = tau[6 + i];
        set_torque_srv.request.duration.nsec = 1000000;
        set_torque_srv.request.duration.sec = 0;
        if (control_client.call(set_torque_srv)) {
          std::cout<<"TORQUE SENT : "<<tau[6+i]<<" "<<joints[i]<<std::endl;
        } else {
          std::cout<<"TORQUE WAS NOT SENT!!!"<<std::endl;
        }
      }

      // print current state
      std::cout << "qs : " << std::endl;
      for (int i = 0; i < 3; i++) {
        std::cout << Q[i] << std::endl;
        // ss << Q[6+i] << ",";
      }
      std::cout << std::endl;
      std::cout << "dqs : " << std::endl;
      for (int i = 0; i < 3; i++) {
        std::cout << dQ[i] << std::endl;
        // ss << dQ[6+i] << ",";
      }
      std::cout << std::endl;
      std::cout << "ddqs : " << std::endl;
      for (int i = 0; i < 3; i++) {
        std::cout << ddQ[i] << " " << ddqs[joints[i]] << std::endl;
        // ss << ddQ_set[6+i] << "," << ddQ_get[6+i] << ",";
      }
      std::cout << std::endl;
      std::cout << "taus : " << std::endl;
      for (int i = 0; i < 3; i++) {
        std::cout << tau[6 + i] << std::endl;
        // ss << tau_set[6+i] << "," << tau_get[6+i] << ",";
      }
      std::cout << std::endl;
      std::cout << delta_t << " " << dt << std::endl;

      iter++;
    }
    // wait ---> roscore!!!!
    loop_rate.sleep();
    // listen to callbacks
    //ros::spinOnce();
  }

  return 0;
}

Model *initModel() {
  double mass = 1;
  double torso_width = 0.7;
  double leg_width = 0.2;
  double torso_thickness = 0.3;
  double leg_thickness = 0.2;
  double torso_length = 0.7;
  double tigh_length = 0.5;
  double calf_length = 0.5;
  double foot_height = 0.2;
  double foot_width = 0.3;
  double foot_length = 0.3;

  double ixx = 0.0;
  double iyy = 0.0;
  double izz = 0.0;

  ixx = mass / 12.0 *
        (torso_thickness * torso_thickness + torso_length * torso_length);
  iyy = mass / 12.0 * (torso_length * torso_length + torso_width * torso_width);
  izz = mass / 12.0 *
        (torso_width * torso_width + torso_thickness * torso_thickness);

  Matrix3d torso_inertia = Matrix3d(ixx, 0, 0, 0, iyy, 0, 0, 0, izz);

  ixx =
      mass / 12.0 * (leg_thickness * leg_thickness + tigh_length * tigh_length);
  iyy = mass / 12.0 * (tigh_length * tigh_length + leg_width * leg_width);
  izz = mass / 12.0 * (leg_width * leg_width + leg_thickness * leg_thickness);

  Matrix3d tigh_inertia = Matrix3d(ixx, 0, 0, 0, iyy, 0, 0, 0, izz);

  ixx =
      mass / 12.0 * (leg_thickness * leg_thickness + calf_length * calf_length);
  iyy = mass / 12.0 * (calf_length * calf_length + leg_width * leg_width);
  izz = mass / 12.0 * (leg_width * leg_width + leg_thickness * leg_thickness);

  Matrix3d calf_inertia = Matrix3d(ixx, 0, 0, 0, iyy, 0, 0, 0, izz);

  ixx = mass / 12.0 * (foot_length * foot_length + foot_height * foot_height);
  iyy = mass / 12.0 * (foot_height * foot_height + foot_width * foot_width);
  izz = mass / 12.0 * (foot_width * foot_width + foot_length * foot_length);

  Matrix3d foot_inertia = Matrix3d(ixx, 0, 0, 0, iyy, 0, 0, 0, izz);

  rbdl_check_api_version(RBDL_API_VERSION);

  Model *model = NULL;

  unsigned int torso_id, tigh_id, calf_id, foot_id;
  Body torso, tigh, calf, foot;
  Joint soul, hip, knee, ankle;

  model = new Model();

  model->gravity = Vector3d(0., 0., 0);

  torso = Body(mass, Vector3d(0, 0, 0), torso_inertia);
  tigh = Body(mass, Vector3d(0, 0, 0), tigh_inertia);
  calf = Body(mass, Vector3d(0, 0, 0), calf_inertia);
  foot = Body(mass, Vector3d(0, 0, 0), foot_inertia);

  hip = Joint(JointTypeRevolute, Vector3d(0, 0, 1.0));
  knee = Joint(JointTypeRevolute, Vector3d(0, 0, 1.0));
  ankle = Joint(JointTypeRevolute, Vector3d(0, 0, 1.0));
  soul = Joint(SpatialVector(1., 0., 0., 0., 0., 0.),
               SpatialVector(0., 1., 0., 0., 0., 0.),
               SpatialVector(0., 0., 1., 0., 0., 0.),
               SpatialVector(0., 0., 0., 1., 0., 0.),
               SpatialVector(0., 0., 0., 0., 1., 0.),
               SpatialVector(0., 0., 0., 0., 0., 1.));

  Eigen::Matrix3d tempE;
  Vector3d tempr;

  torso_id = model->AddBody(0, SpatialTransform(), soul, torso);

  tempE << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  tempr << 0., 0., -torso_length / 2;
  tigh_id = model->AddBody(torso_id, SpatialTransform(tempE, tempr), hip, tigh);

  tempr << 0., 0., -tigh_length / 2;
  calf_id = model->AddBody(tigh_id, SpatialTransform(tempE, tempr), knee, calf);

  tempr << 0., 0., -calf_length / 2;
  foot_id =
      model->AddBody(calf_id, SpatialTransform(tempE, tempr), ankle, foot);

  return model;
}
