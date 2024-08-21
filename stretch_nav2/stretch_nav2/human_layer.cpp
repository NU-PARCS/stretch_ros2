

#include <stretch_nav2/human_layer.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/obstacle_layer.hpp>
#include <std_srvs/srv/empty.hpp>

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

using nav2_costmap_2d::ObservationBuffer;
using nav2_costmap_2d::Observation;
using rcl_interfaces::msg::ParameterType;
namespace stretch_nav2
{

void HumanPathLayer::onInitialize()
{
    ObstacleLayer::onInitialize();

    // auto node = node_.lock();
    // std::string topic = "/human_path";
    // rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    // custom_qos_profile.depth = 50;
    // double transform_tolerance = 0.3;

    

    // auto sub_opt = rclcpp::SubscriptionOptions();
    // sub_opt.callback_group = callback_group_;

    // auto sub2 = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2,
    //       rclcpp_lifecycle::LifecycleNode>>(node, topic, custom_qos_profile, sub_opt);
    // sub2->unsubscribe();


    // auto filter2 = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
    //     *sub2, *tf_, global_frame_, 50,
    //     node->get_node_logging_interface(),
    //     node->get_node_clock_interface(),
    //     tf2::durationFromSec(transform_tolerance));

    // filter2->registerCallback(
    // std::bind(
    //     &HumanPathLayer::pointCloud2Callback, this, std::placeholders::_1,
    //     observation_buffers_.back()));

    // observation_subscribers_.push_back(sub2);
    // observation_notifiers_.push_back(filter2);
}


void HumanPathLayer::pointCloud2Callback(
    sensor_msgs::msg::PointCloud2::SharedPtr message,
    const std::shared_ptr<nav2_costmap_2d::ObservationBuffer> & buffer)
    {
        this->reset();
        buffer->lock();
        buffer->bufferCloud(*message);
        buffer->unlock();
    }    



} // namespace stretch_nav2   
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(stretch_nav2::HumanPathLayer, nav2_costmap_2d::Layer)
