#include <rclcpp/rclcpp.hpp>
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
    first_msg_(true)
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
  // ── Callback dati robot ───────────────────────────────────────────────────
  void statusCallback(const bunker_msgs::msg::BunkerStatus::SharedPtr msg)
  {
    if (first_msg_) {
      last_time_ = msg->header.stamp;
      first_msg_ = false;
      return;
    }

    // Calcola dt
    double dt = (rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_time_)).seconds();
    last_time_ = msg->header.stamp;

    // Scarta dt nulli o anomali (es. bag loop o msg fuori ordine)
    if (dt <= 0.0 || dt > 1.0) {
      return;
    }

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
    odom_msg.header.stamp    = msg->header.stamp;
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
    tf_msg.header.stamp    = msg->header.stamp;
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
    x_         = 0.0;
    y_         = 0.0;
    theta_     = 0.0;
    first_msg_ = true;
    RCLCPP_INFO(this->get_logger(), "Odometry reset to zero.");
  }

  // ── Membri ────────────────────────────────────────────────────────────────
  rclcpp::Subscription<bunker_msgs::msg::BunkerStatus>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr           odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>                  tf_broadcaster_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                reset_srv_;

  double x_, y_, theta_;
  bool   first_msg_;
  builtin_interfaces::msg::Time last_time_;
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometer>());
  rclcpp::shutdown();
  return 0;
}
