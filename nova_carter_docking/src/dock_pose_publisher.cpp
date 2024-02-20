// Copyright (c) 2024 Open Navigation LLC
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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

class DockPosePublisher : public rclcpp::Node
{
  public:
    DockPosePublisher()
    : Node("dock_pose_publisher")
    {
      subscription_ = this->create_subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
      "tag_detections", 10, std::bind(&DockPosePublisher::detectionCallback, this, _1));
      publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("detected_dock_pose", 10);
      
      // If you don't expect multiple detections in a scene, use the only apriltag
      use_first_detection_ = this->declare_parameter("use_first_detection", true);

      // If you expect multiple detections in a scene, specify which you want to use
      // (default here is included in media/ directory)
      dock_tag_family_ = this->declare_parameter("dock_tag_family", "tag36h11");
      dock_tag_id_ = this->declare_parameter("dock_tag_id", 585);
    }

  private:
    void detectionCallback(const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr msg)
    {
      geometry_msgs::msg::PoseStamped p;
      for (unsigned int i = 0; i != msg->detections.size(); i++) {
        if (!use_first_detection_) {
          if (msg->detections[i].family == dock_tag_family_ && msg->detections[i].id == dock_tag_id_) {
            p.header = msg->header;
            p.pose = msg->detections[i].pose.pose.pose;
            publisher_->publish(p);
            return;
          }
        } else {
          // Use the first detection found
          p.header = msg->header;
          p.pose = msg->detections[0].pose.pose.pose;
          publisher_->publish(p);
          return;
        }
      }
    }

    std::string dock_tag_family_;
    int dock_tag_id_;
    bool use_first_detection_;
    rclcpp::Subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
