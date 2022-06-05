# ROS1 and ROS2 Changes
### High Level Architecture Changes
![alt text](./images/ROS1_ROS2_Arch_diff.png "ROS1 and ROS2 HLD")

### ROS2 Architecture
![alt text](./images/ros2_Arch.jpg  "ROS2 HLD")

### ROS2 Component flow
![alt text](./images/Ros2Component.png  "ROS2 Component Flow")

### ROS2 MW Sequence
![alt text](./images/Ros2Sequence.png  "ROS2 Component Flow")

### ROS2 Publisher Sequence
```c++
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```
![alt text](./images/Ros2pub.png  "ROS2 Component Flow")

## ROS1 and ROS2 changes
![alt text](./images/ROS1_vs_ROS2/2.png "ROS1 vs ROS2 Architecture difference")
![alt text](./images/ROS1_vs_ROS2/3.png "ROS1 vs ROS2 Language")
![alt text](./images/ROS1_vs_ROS2/4.png "ROS1 vs ROS2 client lib")
![alt text](./images/ROS1_vs_ROS2/5.png "ROS1 vs ROS2 simple node")
![alt text](./images/ROS1_vs_ROS2/6.png "ROS1 vs ROS2 Launch File")
![alt text](./images/ROS1_vs_ROS2/7.png "ROS1 vs ROS2 Publisher Node")
![alt text](./images/ROS1_vs_ROS2/8.png "ROS1 vs ROS2 Subscriber Node")