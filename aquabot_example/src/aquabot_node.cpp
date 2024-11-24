#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

#define MAX_THRUSTER_POS 0.8
#define Kp 0.5
#define Ki 0.1

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
class AquabotNode : public rclcpp::Node
{
  public:
    AquabotNode() : Node("aquabot_node_cpp"), 
    m_target_pos(0.0), 
    m_yaw_obj(0.0),
    err_sum(0.0)
    {
      // Log that the node has succesfully started
      RCLCPP_INFO(this->get_logger(), "Hello world from aquabot_node_cpp!");

      // Create a publisher on the thusters position topics
      m_thrustersLeftPos_pub = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/left/pos", 10);
      m_thrustersRightPos_pub = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/right/pos", 10);

      imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/aquabot/sensors/imu/imu/data", 10, std::bind(&AquabotNode::imu_callback, this, std::placeholders::_1));

      //Subscriber qui recupere le cap desire
      yaw_obj_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/yaw_hat", 10, std::bind(&AquabotNode::objectif_callback, this, std::placeholders::_1));

      // Create a timer that will call the timer_callback function every 500ms
      //m_tf2_timer = this->create_wall_timer(100ms, std::bind(&AquabotNode::update_engine_position, this));
      m_timer = this->create_wall_timer(500ms, std::bind(&AquabotNode::timer_callback, this));

      // Create tf2 buffer and listener
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::Float64();
      message.data = m_target_pos;
      m_thrustersLeftPos_pub->publish(message);
      m_thrustersRightPos_pub->publish(message);
      RCLCPP_INFO(this->get_logger(), "New target pos: '%f'", m_target_pos);
      //m_target_pos = m_target_pos - 0.1;
    }

    void objectif_callback(std_msgs::msg::Float64::SharedPtr msg){
      m_yaw_obj = msg->data;
      RCLCPP_INFO(this->get_logger(), "Objectif : '%f'", m_yaw_obj);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extraire la quaternion de l'IMU
        tf2::Quaternion quat(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

        // Convertir la quaternion en angles roll, pitch, yaw
        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        RCLCPP_INFO(this->get_logger(), "Cap Actuelle: '%f'", yaw);

        // Calcul de l'erreur normalisée
        double errp = sawtooth(m_yaw_obj - yaw);
        RCLCPP_INFO(this->get_logger(), "Erreur normalisée (errp): '%f'", errp);
        RCLCPP_INFO(this->get_logger(), "Angle objecti: '%f'", m_yaw_obj);
        

        // Accumulation de l'erreur avec saturation
        double dt = 0.1; // Fréquence de la boucle : 10 Hz
        err_sum += errp * dt;
        err_sum = std::min(std::max(err_sum, -10.0), 10.0); // Saturation

        // Calcul de la commande PI
        double err = errp * Kp + err_sum * Ki;
        m_yaw_dot = std::min(std::max(err, -0.5), 0.5); // Saturation de la commande
        RCLCPP_INFO(this->get_logger(), "yaw_dot: '%f'", m_yaw_dot);


        // Calcul de l'angle cible des propulseurs
        m_target_pos = (-1.6)*m_yaw_dot;
    
        RCLCPP_INFO(this->get_logger(), "Commande Envoye: '%f'", m_target_pos);



  
    }

    double sawtooth(double erreur)
    {
        return fmod(erreur + M_PI, 2 * M_PI) - M_PI;
    }
 
    // Position des thruster a commander pour le Cap
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_thrustersLeftPos_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_thrustersRightPos_pub;

    // Cap objectif et Cap pour la regulation
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_yaw_pub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_obj_sub_;

    rclcpp::TimerBase::SharedPtr m_timer;
    //rclcpp::TimerBase::SharedPtr m_tf2_timer;

    float m_target_pos;
    double m_yaw_dot;
    double m_yaw_obj;
    double err_sum;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AquabotNode>());
  rclcpp::shutdown();
  return 0;
}

