#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <bunker_msgs/msg/bunker_status.hpp>
#include <std_srvs/srv/empty.hpp>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// Nodo: odometer
//
// Sottoscrizione:  /bunker_status  (bunker_msgs/msg/BunkerStatus)
// Pubblicazione:   /project_odom   (nav_msgs/msg/Odometry)
//                  TF  odom → base_link2
// Servizio:        reset           (std_srvs/srv/Empty)  →  azzera posizione
// ─────────────────────────────────────────────────────────────────────────────

class Odometer : public rclcpp::Node
{
public:
  Odometer()
  : Node("odometer"),
    x_(0.0), y_(0.0), theta_(0.0),
    first_msg_(true),
    last_stamp_(0, 0, RCL_ROS_TIME)
  {
    // Sottoscrizione ai dati del robot
    sub_ = this->create_subscription<bunker_msgs::msg::BunkerStatus>(
      "/bunker_status", 10,
      std::bind(&Odometer::statusCallback, this, std::placeholders::_1));

    // Publisher odometria
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/project_odom", 10);

    // TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Servizio reset
    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "reset",
      std::bind(&Odometer::resetCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Odometer node started.");
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

  rclcpp::Time resolveSampleTime(const builtin_interfaces::msg::Time & stamp)
  {
    const rclcpp::Time msg_time(stamp);

    if (msg_time.nanoseconds() != 0) {
      return msg_time;
    }

    // When replaying the provided bag, /bunker_status has an empty header.stamp.
    // With use_sim_time enabled, now() follows /clock and stays aligned to the bag.
    return this->get_clock()->now();
  }

  void resetOdometryState()
  {
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    first_msg_ = true;
    last_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  // ── Callback dati robot ───────────────────────────────────────────────────
  void statusCallback(const bunker_msgs::msg::BunkerStatus::SharedPtr msg)
  {
    const rclcpp::Time msg_stamp(msg->header.stamp);
    const rclcpp::Time current_stamp = resolveSampleTime(msg->header.stamp);

    if (current_stamp.nanoseconds() == 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for a valid timestamp from /bunker_status or /clock.");
      return;
    }

    if (msg_stamp.nanoseconds() == 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Using /clock as timestamp because /bunker_status.header.stamp is empty.");
    }

    if (first_msg_) {
      last_stamp_ = current_stamp;
      first_msg_ = false;
      return;
    }

    const double dt = (current_stamp - last_stamp_).seconds();

    if (dt <= 0.0) {
      last_stamp_ = current_stamp;
      if (dt < 0.0) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Skipping /bunker_status sample with non-increasing dt=%.3f s.", dt);
      }
      return;
    }

    if (dt > 1.0) {
      last_stamp_ = current_stamp;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Skipping /bunker_status sample with invalid dt=%.3f s.", dt);
      return;
    }

    last_stamp_ = current_stamp;

    const double v = msg->linear_velocity;   // [m/s]
    const double w = msg->angular_velocity;  // [rad/s]

    // ── Integrazione odometria con metodo di Eulero ────────────────────────
    // Per un robot con cingoli (skid-steering) la cinematica è:
    //   x    += v * cos(θ) * dt
    //   y    += v * sin(θ) * dt
    //   theta += ω * dt
    x_     += v * std::cos(theta_) * dt;
    y_     += v * std::sin(theta_) * dt;
    theta_ += w * dt;

    // Normalizza theta in [-π, π]
    while (theta_ >  M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

    // Converti yaw → quaternione
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);

    // ── Pubblica nav_msgs/Odometry ─────────────────────────────────────────
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp    = toBuiltinTime(current_stamp);
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id  = "base_link2";

    odom_msg.pose.pose.position.x  = x_;
    odom_msg.pose.pose.position.y  = y_;
    odom_msg.pose.pose.position.z  = 0.0;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x  = v;
    odom_msg.twist.twist.angular.z = w;

    odom_pub_->publish(odom_msg);

    // ── Pubblica TF  odom → base_link2 ────────────────────────────────────
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp    = toBuiltinTime(current_stamp);
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id  = "base_link2";

    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
  }

  // ── Callback servizio reset ───────────────────────────────────────────────
  void resetCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    const std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    resetOdometryState();
    RCLCPP_INFO(this->get_logger(), "Odometry reset to zero.");
  }

  // ── Membri ────────────────────────────────────────────────────────────────
  rclcpp::Subscription<bunker_msgs::msg::BunkerStatus>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr           odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>                  tf_broadcaster_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                reset_srv_;

  double x_, y_, theta_;
  bool   first_msg_;
  rclcpp::Time last_stamp_;
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometer>());
  rclcpp::shutdown();
  return 0;
}
