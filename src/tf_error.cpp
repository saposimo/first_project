#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <first_project/msg/tf_error_msg.hpp>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// Nodo: tf_error
//
// Legge dal TF tree:
//   odom → base_link   (ground truth dal bag GPS)
//   odom → base_link2  (odometria calcolata da noi)
//
// Pubblicazione: /tf_error_msg  (first_project/msg/TfErrorMsg)
//   - tf_error          : distanza euclidea tra le due pose [m]
//   - time_from_start   : secondi dall'avvio del confronto [s]
//   - travelled_distance: distanza percorsa da base_link2  [m]
// ─────────────────────────────────────────────────────────────────────────────

class TfError : public rclcpp::Node
{
public:
  TfError()
  : Node("tf_error"),
    first_call_(true),
    start_time_(0, 0, RCL_ROS_TIME),
    last_eval_time_(0, 0, RCL_ROS_TIME),
    travelled_distance_(0.0f),
    prev_x_(0.0), prev_y_(0.0),
    first_pose_(true)
  {
    // Buffer e listener TF
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publisher messaggio custom
    pub_ = this->create_publisher<first_project::msg::TfErrorMsg>("/tf_error_msg", 10);

    // Timer: eseguito a 10 Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TfError::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "TF Error node started.");
  }

private:
  builtin_interfaces::msg::Time toBuiltinTime(const rclcpp::Time & time) const
  {
    builtin_interfaces::msg::Time stamp;
    const int64_t nanoseconds = time.nanoseconds();
    stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    return stamp;
  }

  void resetMetrics(const rclcpp::Time & now)
  {
    first_call_ = true;
    start_time_ = now;
    last_eval_time_ = now;
    travelled_distance_ = 0.0f;
    prev_x_ = 0.0;
    prev_y_ = 0.0;
    first_pose_ = true;
  }

  void timerCallback()
  {
    const rclcpp::Time now = this->get_clock()->now();

    if (now.nanoseconds() == 0) {
      return;
    }

    if (last_eval_time_.nanoseconds() != 0 && now < last_eval_time_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Detected jump back in time. Resetting TF error metrics.");
      resetMetrics(now);
    }

    last_eval_time_ = now;

    geometry_msgs::msg::TransformStamped t_gt, t_our;

    // Prova a leggere entrambi i TF (ultima disponibile)
    try {
      t_gt  = tf_buffer_->lookupTransform("odom", "base_link",  tf2::TimePointZero);
      t_our = tf_buffer_->lookupTransform("odom", "base_link2", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for TF odom->base_link and odom->base_link2: %s", ex.what());
      return;
    }

    // Inizializza il tempo di partenza al primo match valido
    if (first_call_) {
      start_time_ = now;
      first_call_ = false;
    }

    // ── Errore: distanza euclidea tra le due pose (XY) ────────────────────
    const double dx = t_gt.transform.translation.x - t_our.transform.translation.x;
    const double dy = t_gt.transform.translation.y - t_our.transform.translation.y;
    const float  tf_error = static_cast<float>(std::sqrt(dx * dx + dy * dy));

    // ── Distanza percorsa (integrazione su base_link2) ─────────────────────
    const double cx = t_our.transform.translation.x;
    const double cy = t_our.transform.translation.y;

    if (!first_pose_) {
      const double step_x = cx - prev_x_;
      const double step_y = cy - prev_y_;
      travelled_distance_ += static_cast<float>(std::sqrt(step_x * step_x + step_y * step_y));
    }
    prev_x_     = cx;
    prev_y_     = cy;
    first_pose_ = false;

    // ── Tempo dall'avvio ───────────────────────────────────────────────────
    const int32_t time_from_start =
      static_cast<int32_t>((now - start_time_).seconds());

    // ── Pubblica messaggio custom ──────────────────────────────────────────
    first_project::msg::TfErrorMsg out;
    out.header.stamp        = toBuiltinTime(now);
    out.header.frame_id     = "odom";
    out.tf_error            = tf_error;
    out.time_from_start     = time_from_start;
    out.travelled_distance  = travelled_distance_;

    pub_->publish(out);
  }

  // ── Membri ────────────────────────────────────────────────────────────────
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<first_project::msg::TfErrorMsg>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr                timer_;

  bool            first_call_;
  rclcpp::Time    start_time_;
  rclcpp::Time    last_eval_time_;
  float           travelled_distance_;
  double          prev_x_, prev_y_;
  bool            first_pose_;
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfError>());
  rclcpp::shutdown();
  return 0;
}
