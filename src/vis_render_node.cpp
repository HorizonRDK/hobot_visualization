// Copyright (c) 2023，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "include/vis_render_node.h"

VisNode::VisNode(const std::string &node_name, const NodeOptions &options)
    : rclcpp::Node(node_name, options) {

  this->declare_parameter<std::string>("msg_pub_topic_name", msg_pub_topic_name_);
  this->declare_parameter<std::string>("smart_msg_sub_topic_name", smart_msg_sub_topic_name_);

  this->get_parameter<std::string>("msg_pub_topic_name", msg_pub_topic_name_);
  this->get_parameter<std::string>("smart_msg_sub_topic_name", smart_msg_sub_topic_name_);

  {
    std::stringstream ss;
    ss << "Parameter:"
       << "\n msg_pub_topic_name: " << msg_pub_topic_name_
       << "\n smart_msg_sub_topic_name: " << smart_msg_sub_topic_name_;
    RCLCPP_WARN(rclcpp::get_logger("hobot_trigger"), "%s", ss.str().c_str());
  }

  msg_publisher_ = this->create_publisher<visualization_msgs::msg::ImageMarker>(
    msg_pub_topic_name_, 10);

  smart_msg_subscription_ =
    this->create_subscription<ai_msgs::msg::PerceptionTargets>(
        smart_msg_sub_topic_name_,
        10,
        std::bind(&VisNode::SmartTopicCallback,
                  this,
                  std::placeholders::_1));
  
  RCLCPP_INFO(rclcpp::get_logger("VisNode"), "VisNode start."); 
}

VisNode::~VisNode() {}

void VisNode::SmartTopicCallback(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg) {
  std::stringstream ss;
  ss << "Recved msg"
     << ", frame_id: " << msg->header.frame_id
     << ", stamp: " << msg->header.stamp.sec << "_" << msg->header.stamp.nanosec
     << ", targets size: " << msg->targets.size();

  // 发布 image_marker
  visualization_msgs::msg::ImageMarker image_marker;
  image_marker.header = msg->header;
  image_marker.type = visualization_msgs::msg::ImageMarker::LINE_LIST;

  // 设置缩放因子
  image_marker.scale = 10;
  image_marker.filled = 0;
  image_marker.action = visualization_msgs::msg::ImageMarker::ADD;  

  for (const auto& tar : msg->targets) {
    ss << " has roi num: " << tar.rois.size();
    ss << " has capture num: " << tar.captures.size();
    
    for (const auto& roi : tar.rois) {
      ss << ", roi type: " << roi.type;
      ss << ", roi x1: " << roi.rect.x_offset;
      ss << ", roi y1: " << roi.rect.y_offset;
      ss << ", roi x2: " << roi.rect.x_offset + roi.rect.width;
      ss << ", roi y2: " << roi.rect.y_offset + roi.rect.height;

      geometry_msgs::msg::Point p1, p2, p3, p4;
      p1.x = roi.rect.x_offset;
      p1.y = roi.rect.y_offset;
      p1.z = 0.0;

      p2.x = roi.rect.x_offset + roi.rect.width;
      p2.y = roi.rect.y_offset;
      p2.z = 0.0;

      p3.x = roi.rect.x_offset + roi.rect.width;
      p3.y = roi.rect.y_offset + roi.rect.height;
      p3.z = 0.0;

      p4.x = roi.rect.x_offset;
      p4.y = roi.rect.y_offset + roi.rect.height;
      p4.z = 0.0;

      // 添加矩形框1的四个边
      image_marker.points.push_back(p1);
      image_marker.points.push_back(p2);

      image_marker.points.push_back(p2);
      image_marker.points.push_back(p3);

      image_marker.points.push_back(p3);
      image_marker.points.push_back(p4);

      image_marker.points.push_back(p4);
      image_marker.points.push_back(p1);

      // 设置外边框颜色
      std_msgs::msg::ColorRGBA outline_color;
      outline_color.r = 1.0;
      outline_color.g = 0.0;
      outline_color.b = 0.0;
      outline_color.a = 1.0;

      for(int i = 0; i < 4; i++){
        image_marker.outline_colors.push_back(outline_color);
      }
    }

    for (const auto& capture : tar.captures) {
      ss << ", capture features size: " << capture.features.size();
      ss << ", img.width: " << capture.img.width;
      ss << ", img.height: " << capture.img.height;
    }

    ss << ", has attr num: " << tar.attributes.size();
    for (const auto& attr : tar.attributes) {
      ss << ", attr type: " << attr.type << ", val: " << attr.value;
    }
    ss << "\n";
  }

  msg_publisher_->publish(image_marker);

  RCLCPP_INFO(rclcpp::get_logger("VisNode"), "smart msg: %s", ss.str().c_str());

}