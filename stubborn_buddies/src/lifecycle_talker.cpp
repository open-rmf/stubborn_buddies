// Copyright (c) 2020 Mapless AI, Inc.
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

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "std_msgs/msg/string.hpp"

#include "stubborn_buddies_msgs/msg/status.hpp"

using namespace std::chrono_literals;

constexpr char DEFAULT_STATUS_NAME[] = "status";

namespace lifecycle_talker
{
  
class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode
{
public:

  explicit LifecycleTalker(const rclcpp::NodeOptions& options):
    LifecycleNode("lifecycle_talker", options),
    active_node_(true), count_(0), talker_period_(1000ms),
    active_status_topic_(DEFAULT_STATUS_NAME)
  {
    //Declare parameters
    this->declare_parameter<bool>("active_node", true);
    this->declare_parameter<int>("talker_period", 1000);
    this->declare_parameter<std::string>("buddy_subns", "yang");
    this->declare_parameter<std::string>("namespace", std::string());
    
    configure();

    if(active_node_)
      activate();
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &)
  {
    //Retrieve parameters values
    this->get_parameter("active_node", active_node_);
    talker_publisher_ = this->create_publisher<std_msgs::msg::String>("/chatter", 10);
    this->get_parameter("buddy_subns", buddy_subns_);
    this->get_parameter("namespace", namespace_);

    active_status_topic_ = "/" + namespace_ + "/" + buddy_subns_ + "/" + std::string(DEFAULT_STATUS_NAME);
    
    //only suscribe for eventual activation if we are the inactive node
    if(!active_node_)
    {
      status_sub_ = this->create_subscription<stubborn_buddies_msgs::msg::Status>(
        active_status_topic_,
        10,
        [this](const typename stubborn_buddies_msgs::msg::Status::SharedPtr msg) -> void {
                RCLCPP_WARN(get_logger(), "Watchdog rised at %s, "
                                          "self activation triggered",
                            active_status_topic_.c_str(),
                            msg->stamp.sec);
                this->set_parameter(rclcpp::Parameter("active_node", true));
                active_node_=true;
                activate();
        });
    }

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &)
  {
    //Retrieve parameters values
    talker_period_ = std::chrono::milliseconds(this->get_parameter("talker_period").as_int());
    
    timer_ = this->create_wall_timer(talker_period_, 
                                     std::bind(&LifecycleTalker::talker_timer_callback, this));
    talker_publisher_->on_activate();
    
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &)
  {
    talker_publisher_->on_deactivate();
    if(status_sub_)
    {
      status_sub_.reset(); // there does not seem to be a 'deactivate' for subscribers.
      status_sub_ = nullptr;
    }
    
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    on_cleanup(const rclcpp_lifecycle::State &)
  {
    timer_.reset();
    if(status_sub_)
    {
      status_sub_.reset(); // there does not seem to be a 'deactivate' for subscribers.
      status_sub_ = nullptr;
    }

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_cleanup() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
    on_shutdown(const rclcpp_lifecycle::State &)
  {
    timer_.reset();
    if(status_sub_)
    {
      status_sub_.reset(); // there does not seem to be a 'deactivate' for subscribers.
      status_sub_ = nullptr;
    }

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  
  void talker_timer_callback()
  {
    auto message = std::make_unique<std_msgs::msg::String>();
    message->data = "Hello World: " + std::to_string(++count_);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message->data.c_str());
    std::flush(std::cout);

    talker_publisher_->publish(std::move(message));
    
  }
  
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr talker_publisher_ = nullptr;
  rclcpp::Subscription<stubborn_buddies_msgs::msg::Status>::SharedPtr status_sub_ = nullptr;
  
  bool active_node_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  std::chrono::milliseconds talker_period_;
  std::string active_status_topic_;
  std::string buddy_subns_;
  std::string namespace_;

};

}// namespace lifecycle_talker

RCLCPP_COMPONENTS_REGISTER_NODE(lifecycle_talker::LifecycleTalker)
