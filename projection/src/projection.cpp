#include "projection/projection.hpp"

Projection::Projection(): Node("projection")
{
    initProj();
    gnss_pose_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>("/gnss_pose", 10, std::bind(&Projection::pose_callback, this, std::placeholders::_1));
    coord_pub = this->create_publisher<geometry_msgs::msg::Vector3>("/projected_gnss_pose", 10);
}

Projection::~Projection()
{
    proj_destroy(P);
    proj_context_destroy(ctx);
}

void Projection::initProj() // Initialisation of the Proj projection variables
{
    // Créer un contexte de transformation
    ctx = proj_context_create();
    P = proj_create(ctx, projectionDefinition);
    if (P == NULL){
        RCLCPP_ERROR(this->get_logger(), "Error: Projection failed");
        rclcpp::shutdown();
    }
}

// This callback receives the gnss position, and publishes it back as converted coordinate
void Projection::pose_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    // Saving the coordinates in the coords variable
    coords.lpzt.lam = proj_torad(msg->latitude);
    coords.lpzt.phi = proj_torad(msg->longitude);
    coords.lpzt.z = msg->altitude;
    // RCLCPP_INFO(this->get_logger(), "In GNSS frame: X: %f, Y: %f, Z: %f", coords.lpzt.lam, coords.lpzt.phi, coords.lpzt.z);
    // Converting the coordinates to the cartesian frame
    projected_coords = proj_trans(P, PJ_FWD, coords);
    geometry_msgs::msg::Vector3 coords_msg;
    coords_msg.x = projected_coords.xyz.x;
    coords_msg.y = projected_coords.xyz.y;
    coords_msg.z = projected_coords.xyz.z;
    coord_pub->publish(coords_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Projection>());
    rclcpp::shutdown();
    return 0;
}