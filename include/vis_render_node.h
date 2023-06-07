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

#ifndef HOBOT_VISUALIZATION_H_
#define HOBOT_VISUALIZATION_H_

#include <cstdlib>
#include <ctime>

#include "ai_msgs/msg/perception_targets.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/image_marker.hpp>

using rclcpp::NodeOptions;
using std::placeholders::_1;

class VisNode : public rclcpp::Node
{
public:
  VisNode(const std::string &node_name,
        const NodeOptions &options = NodeOptions());

  virtual ~VisNode();

  void SmartTopicCallback(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);

private:

  std::string msg_pub_topic_name_ = "/hobot_visualization";
  rclcpp::Publisher<visualization_msgs::msg::ImageMarker>::SharedPtr msg_publisher_;

  std::string smart_msg_sub_topic_name_ = "/hobot_dnn_detection";
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr
      smart_msg_subscription_ = nullptr;
};

#endif  // HOBOT_VISUALIZATION_H_