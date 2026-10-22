// StudentAnimator: poses the links of static student models from a topic.
//
// Subscribes to gz.msgs.Pose_V messages whose pose names are "<model>::<link>", with
// poses relative to the model frame, and writes them straight into the links' Pose
// components. Student models are static and jointless, so physics never simulates
// them; the student_behaviors node does the animation and forward kinematics.
//
// Driving ~400 physics joints instead (position resets or velocity commands) pulled
// the simulation well below real time.

#include <gz/msgs/pose_v.pb.h>

#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/msgs/convert/Pose.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>

namespace exam_proctor
{

class StudentAnimator : public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> & sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    const auto topic = sdf->Get<std::string>("topic", "/classroom/student_link_poses").first;
    if (this->node.Subscribe(topic, &StudentAnimator::OnPoses, this)) {
      gzmsg << "StudentAnimator listening on [" << topic << "]\n";
    } else {
      gzerr << "StudentAnimator failed to subscribe to [" << topic << "]\n";
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager & ecm) override
  {
    std::vector<std::pair<std::string, gz::math::Pose3d>> poses;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      poses.swap(this->pending);
    }
    for (const auto & [name, pose] : poses) {
      const auto link = this->Resolve(name, ecm);
      if (link != gz::sim::kNullEntity) {
        ecm.SetComponentData<gz::sim::components::Pose>(link, pose);
        ecm.SetChanged(link, gz::sim::components::Pose::typeId,
                       gz::sim::ComponentState::PeriodicChange);
      }
    }
  }

private:
  void OnPoses(const gz::msgs::Pose_V & msg)
  {
    std::vector<std::pair<std::string, gz::math::Pose3d>> poses;
    poses.reserve(msg.pose_size());
    for (const auto & pose : msg.pose()) {
      poses.emplace_back(pose.name(), gz::msgs::Convert(pose));
    }
    std::lock_guard<std::mutex> lock(this->mutex);
    this->pending.swap(poses);  // only the latest poses matter
  }

  gz::sim::Entity Resolve(const std::string & scoped, gz::sim::EntityComponentManager & ecm)
  {
    const auto cached = this->links.find(scoped);
    if (cached != this->links.end()) {
      return cached->second;
    }
    gz::sim::Entity link = gz::sim::kNullEntity;
    const auto sep = scoped.find("::");
    if (sep != std::string::npos) {
      const auto model = ecm.EntityByComponents(
        gz::sim::components::Name(scoped.substr(0, sep)), gz::sim::components::Model());
      if (model != gz::sim::kNullEntity) {
        link = gz::sim::Model(model).LinkByName(ecm, scoped.substr(sep + 2));
      }
    }
    if (link == gz::sim::kNullEntity) {
      if (this->missing.insert(scoped).second) {
        gzwarn << "StudentAnimator: no link named [" << scoped << "]\n";
      }
      return link;  // not cached: the model may appear later
    }
    this->links.emplace(scoped, link);
    return link;
  }

  gz::transport::Node node;
  std::mutex mutex;
  std::vector<std::pair<std::string, gz::math::Pose3d>> pending;
  std::unordered_map<std::string, gz::sim::Entity> links;
  std::unordered_set<std::string> missing;
};

}  // namespace exam_proctor

GZ_ADD_PLUGIN(
  exam_proctor::StudentAnimator,
  gz::sim::System,
  exam_proctor::StudentAnimator::ISystemConfigure,
  exam_proctor::StudentAnimator::ISystemPreUpdate)
