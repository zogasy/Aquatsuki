#include "projection.hpp"
#include "tf2/utils.h"

Projection::Projection(): Node("projection")
{
    initProj();
    gnss_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/gnss_pose", 10, std::bind(&Control::pose_callback, this, std::placeholders::_1));
    coord_pub = this->create_publisher<geometry_msgs::msg::Vector3>("/projected_gnss_pose", 10);
}

Projection::~Projection():
{
    proj_destroy(P);
    proj_context_destroy(ctx);
}

void Projection::initProj(double &x, double &y, double &z) // Convert the coordinates from the GNSS frame to a cartesian frame using proj4 library
{
    P = proj_create(nullptr, projectionDefinition);
    // Créer un contexte de transformation
    ctx = proj_context_create();
    if (P == NULL){
        RCLCPP_ERROR(this->get_logger(), "Error: Projection failed");
        rclcpp::shutdown();
    }
}

// This callback receives the gnss position, and publishes it back as converted coordinate
void Projection::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg):
{
    // Saving the coordinates in the coords variable
    coords.lpzt.lam = proj_torad(msg->latitude);
    coords.lpzt.phi = proj_torad(msg->longitude);
    coords.lpzt.z = msg->altitude;
    // RCLCPP_INFO(this->get_logger(), "In GNSS frame: X: %f, Y: %f, Z: %f", coords.lpzt.lam, coords.lpzt.phi, coords.lpzt.z);
    // Converting the coordinates to the cartesian frame
    projected_coords = proj_trans(P, PJ_FWD, coords);
    geometry_msgs::msg::Vector3 coords_msg
    coords_msg.x = projected_coords.xyz.x
    coords_msg.y = projected_coords.xyz.y
    coords_msg.z = projected_coords.xyz.z
    coord_pub->publish(coords_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Projection>());
    rclcpp::shutdown();
    return 0;
}