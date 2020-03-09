#include <ignition/math/Pose3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/comm/model_t.hpp>

namespace gazebo {

class WorldUpdate : public WorldPlugin {
 public:
  void Load(physics::WorldPtr parent, sdf::ElementPtr) {
    this->world_ = parent;

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&WorldUpdate::OnUpdate, this));

    lcm_.subscribe("SPAWN_MODEL", &WorldUpdate::SpawnModelHandler, this);
  }

 private:
  void SpawnModelHandler(const lcm::ReceiveBuffer *rbuf,
                         const std::string &channel,
                         const comm::model_t *msg) {
    sdf::SDFPtr sdf_ptr = sdf::readFile(msg->path);
    sdf::ElementPtr model = sdf_ptr->Root()->GetElement("model");
    model->GetAttribute("name")->SetFromString(msg->name);
    world_->InsertModelSDF(*sdf_ptr);
    gzmsg << "Inserted Model: " << msg->name << std::endl;
  }

  void OnUpdate() {
    lcm_.handleTimeout(0);
  }

  // Pointer to the world
  physics::WorldPtr world_;

  // LCM handle
  lcm::LCM lcm_;

  // Pointer to the update event connection
  event::ConnectionPtr update_connection_;
};

GZ_REGISTER_WORLD_PLUGIN(WorldUpdate)

} /* namespace gazebo */
