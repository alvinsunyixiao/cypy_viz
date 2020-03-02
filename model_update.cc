#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/comm/pose3d_t.hpp>

namespace gazebo
{

class ModelUpdate : public ModelPlugin
{
 public: 
  void Load(physics::ModelPtr parent, sdf::ElementPtr) {
    // Store the pointer to the model
    this->model_ = parent;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ModelUpdate::OnUpdate, this));

    // initialize pose
    pose_.Set(model_->WorldPose().Pos(), model_->WorldPose().Rot());
    // initialize velocity
    model_->SetLinearVel(ignition::math::Vector3d(0, 0, 0));

    common::Console::msg << "Model Name: " << model_->GetName() << std::endl;

    lcm_.subscribe(model_->GetName(), &ModelUpdate::MessageHandler, this);
  }

  // Called by the world update start event
  void OnUpdate() {
    // update location;
    lcm_.handleTimeout(0);
    model_->SetWorldPose(pose_);
  }

 private: 
  void MessageHandler(const lcm::ReceiveBuffer *rbuf, 
                      const std::string &channel,
                      const comm::pose3d_t *msg) {
    pose_.Pos().Set(msg->position.x, msg->position.y, msg->position.z);
    common::Console::msg << "Pose Updated: " << pose_ << std::endl;
  }

  // Pointer to the model
  physics::ModelPtr model_;

  // Pointer to the update event connection
  event::ConnectionPtr update_connection_;

  // global pose
  ignition::math::Pose3d pose_;

  // LCM handle
  lcm::LCM lcm_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelUpdate)

} /* namespace gazebo */
