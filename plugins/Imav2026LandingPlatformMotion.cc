#include <algorithm>
#include <chrono>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/plugin/Register.hh>

#include <sdf/Element.hh>

namespace imav2026
{

/// \brief A Gazebo Sim System plugin that moves the IMAV2026 landing
///        platform back and forth along the X axis between two configurable
///        boundaries at a fixed speed.
///
/// This is a World plugin that finds the target model by name in the
/// EntityComponentManager and updates its pose command every PreUpdate tick.
class LandingPlatformMotion
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
public:
  LandingPlatformMotion() = default;

  // Documentation inherited
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager & /*_eventMgr*/) override
  {
    // For a world plugin, _entity is the world entity.
    this->worldEntity = _entity;

    // Read optional parameters from SDF, falling back to defaults
    if (_sdf->HasElement("model_name"))
      this->modelName = _sdf->Get<std::string>("model_name");

    if (_sdf->HasElement("min_x"))
      this->minX = _sdf->Get<double>("min_x");

    if (_sdf->HasElement("max_x"))
      this->maxX = _sdf->Get<double>("max_x");

    if (_sdf->HasElement("speed_mps"))
      this->speed = _sdf->Get<double>("speed_mps");

    // Sanity: enforce min <= max
    if (this->minX > this->maxX)
      std::swap(this->minX, this->maxX);

    // Clamp speed to non-negative
    this->speed = std::max(0.0, this->speed);

    // Determine initial travel direction from the model's current pose,
    // matching the behaviour of the original Gazebo Classic plugin.
    gz::sim::World world(this->worldEntity);
    auto modelEntity = world.ModelByName(_ecm, this->modelName);
    if (modelEntity != gz::sim::kNullEntity)
    {
      gz::sim::Model model(modelEntity);
      auto pose = gz::sim::worldPose(modelEntity, _ecm);
      this->direction = (pose.Pos().X() <= this->minX) ? 1.0 : -1.0;
    }

    gzmsg << "Imav2026LandingPlatformMotion configured for model ["
          << this->modelName << "] "
          << "min_x=" << this->minX << ", "
          << "max_x=" << this->maxX << ", "
          << "speed_mps=" << this->speed << ", "
          << "initial_direction=" << this->direction << std::endl;
  }

  // Documentation inherited
  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    // Guard: valid speed
    if (this->speed <= 0.0)
      return;

    // Find the target model by name via the world.
    gz::sim::World world(this->worldEntity);
    auto modelEntity = world.ModelByName(_ecm, this->modelName);
    if (modelEntity == gz::sim::kNullEntity)
      return;

    gz::sim::Model model(modelEntity);

    // On the very first call, stash the initial sim time and skip the
    // first step so we have a non-zero dt on the next tick.
    if (this->lastTime.count() == 0)
    {
      this->lastTime = _info.simTime;
      return;
    }

    // Compute elapsed wall-clock time since the last tick.
    const double dt =
        std::chrono::duration<double>(_info.simTime - this->lastTime).count();
    this->lastTime = _info.simTime;

    if (dt <= 0.0)
      return;

    // Current pose
    auto pose = gz::sim::worldPose(modelEntity, _ecm);

    // Compute next X position, bouncing at the boundaries.
    double nextX = pose.Pos().X() + this->direction * this->speed * dt;

    if (nextX >= this->maxX)
    {
      nextX = this->maxX;
      this->direction = -1.0;
    }
    else if (nextX <= this->minX)
    {
      nextX = this->minX;
      this->direction = 1.0;
    }

    // Apply the new pose via the command interface so the physics system
    // picks it up during the update step.
    pose.Pos().X(nextX);
    model.SetWorldPoseCmd(_ecm, pose);
  }

private:
  /// World entity (set in Configure).
  gz::sim::Entity worldEntity{gz::sim::kNullEntity};
  /// Name of the model to move.
  std::string modelName = "imav2026_landing_platform";

  /// Leftmost X coordinate (world frame).
  double minX = 1.569980;

  /// Rightmost X coordinate (world frame).
  double maxX = 2.727315;

  /// Constant speed in m/s.
  double speed = 0.2;

  /// Current travel direction (+1 = toward maxX, -1 = toward minX).
  double direction = -1.0;

  /// Simulation time of the previous PreUpdate call.
  std::chrono::steady_clock::duration lastTime{0};
};

}  // namespace imav2026

GZ_ADD_PLUGIN(imav2026::LandingPlatformMotion,
              gz::sim::System,
              imav2026::LandingPlatformMotion::ISystemConfigure,
              imav2026::LandingPlatformMotion::ISystemPreUpdate)
