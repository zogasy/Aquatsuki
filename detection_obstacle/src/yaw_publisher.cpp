#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class YawExtractorNode : public rclcpp::Node
{
public:
    YawExtractorNode()
        : Node("yaw_extractor_node")
    {
        // Créer le subscriber pour le topic IMU
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/aquabot/sensors/imu/imu/data", 10, std::bind(&YawExtractorNode::imu_callback, this, std::placeholders::_1));

        // Créer le publisher pour publier le cap (yaw)
        yaw_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/boat_cap", 10);
        speed_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/boat_speed",10);
        ang_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/yaw_dot",10);

        RCLCPP_INFO(this->get_logger(), "YawExtractorNode is running...");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {


        // Extraire la norme de la vitesse 
        double ax = msg->linear_acceleration.x;
        double ay = msg->linear_acceleration.y;
        // Calcul de la norme de l'accélération (sans gravité si déjà compensée)
        double linear_speed = sqrt(ax * ax + ay * ay );

        double yaw_dot = msg->angular_velocity.z;
        


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

        //conversion en message ROS
        auto yaw_msg = std_msgs::msg::Float64();
        auto speed_msg = std_msgs::msg::Float64();
        auto yaw_dot_msg = std_msgs::msg::Float64();

        yaw_dot_msg.data = yaw_dot;
        yaw_msg.data = yaw;
        speed_msg.data = linear_speed;

        yaw_publisher_->publish(yaw_msg);
        speed_publisher_->publish(speed_msg);
        ang_vel_publisher_->publish(yaw_dot_msg);

        RCLCPP_INFO(this->get_logger(), "Published yaw: %f", yaw);
        RCLCPP_INFO(this->get_logger(), "Linear speed: %f", linear_speed);
        RCLCPP_INFO(this->get_logger(), "Vitesse angulaire cap: %f", yaw_dot);


    }

    // Déclaration des membres
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ang_vel_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YawExtractorNode>());
    rclcpp::shutdown();
    return 0;
}
