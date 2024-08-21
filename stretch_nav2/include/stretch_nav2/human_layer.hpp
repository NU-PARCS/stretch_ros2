#ifndef HUMAN_PATH_LAYER_HPP
#define HUMAN_PATH_LAYER_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <vector>

#include "laser_geometry/laser_geometry.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreorder"
#include "tf2_ros/message_filter.h"
#pragma GCC diagnostic pop
#include "message_filters/subscriber.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/obstacle_layer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav2_costmap_2d/observation_buffer.hpp>
#include <std_srvs/srv/empty.hpp>

namespace stretch_nav2
{
class HumanPathLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
    // HumanPathLayer();
    void onInitialize() override;


    void pointCloud2Callback(
        sensor_msgs::msg::PointCloud2::SharedPtr message,
        const std::shared_ptr<nav2_costmap_2d::ObservationBuffer> & buffer);

};

} // namespace stretch_nav2
#endif