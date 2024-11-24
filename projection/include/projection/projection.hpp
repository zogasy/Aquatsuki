#ifndef __PTU_CONTROL_HPP
#define __PTU_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <proj.h>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


class Projection : public rclcpp::Node
{
public:
    Projection();
    ~Projection();
    void pose_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

private:
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_pose_sub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr coord_pub;


    PJ* P;
    PJ_CONTEXT* ctx;
    PJ_COORD coords;
    PJ_COORD projected_coords;
    const char* projectionDefinition = "+proj=tmerc +lat_0=48.4 +lon_0=-4.4833 +ellps=WGS84 +x_0=0 +y_0=0";
    void initProj();
};

#endif