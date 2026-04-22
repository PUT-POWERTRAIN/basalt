#pragma once

#include <string>
#include <memory>
#include <vector>

#ifdef BASALT_WITH_RERUN
#include <rerun.hpp>
#include <sophus/se3.hpp>
#include <Eigen/Dense>

namespace basalt {

class RerunHelper {
 public:
  static void initialize(const std::string& app_name) {
    if (!instance().rec) {
      instance().rec = std::make_unique<rerun::RecordingStream>(app_name);
      instance().rec->spawn().exit_on_failure();
    }
  }

  static void set_recording_stream(rerun::RecordingStream* rec) {
    instance().external_rec_ptr = rec;
  }

  static rerun::RecordingStream* stream() {
    if (instance().external_rec_ptr) return instance().external_rec_ptr;
    return instance().rec.get();
  }

  static bool enabled() {
    return stream() != nullptr;
  }

  static void log_pose(const std::string& path, const Sophus::SE3d& T_w_i, int64_t t_ns) {
    auto s = stream();
    if (!s) return;

    s->set_time_timestamp_nanos_since_epoch("sensor_time", t_ns);

    const auto& rot = T_w_i.unit_quaternion();
    const auto& trans = T_w_i.translation();

    s->log(path, rerun::Transform3D(
        rerun::components::Translation3D{static_cast<float>(trans.x()), static_cast<float>(trans.y()), static_cast<float>(trans.z())},
        rerun::Quaternion::from_xyzw(static_cast<float>(rot.x()), static_cast<float>(rot.y()),
                                     static_cast<float>(rot.z()), static_cast<float>(rot.w()))
    ));
  }

  template<typename PointContainer>
  static void log_points(const std::string& path, const PointContainer& points, int64_t t_ns) {
    auto s = stream();
    if (!s) return;

    s->set_time_timestamp_nanos_since_epoch("sensor_time", t_ns);

    std::vector<rerun::Position3D> rerun_points;
    rerun_points.reserve(points.size());
    for (const auto& p : points) {
      rerun_points.push_back({static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())});
    }

    s->log(path, rerun::Points3D(rerun_points).with_radii({0.01f}));
  }

 private:
  RerunHelper() = default;
  static RerunHelper& instance() {
    static RerunHelper helper;
    return helper;
  }

  std::unique_ptr<rerun::RecordingStream> rec;
  rerun::RecordingStream* external_rec_ptr{nullptr};
};

} // namespace basalt

#else

namespace basalt {
class RerunHelper {
 public:
  static bool enabled() { return false; }
  static void initialize(const std::string&) {}
  static void set_recording_stream(void* rec) { (void)rec; }

  template<typename T>
  static void log_pose(const std::string&, const T&, int64_t) {}

  template<typename T>
  static void log_points(const std::string&, const std::vector<T>&, int64_t) {}
};
}

#endif
