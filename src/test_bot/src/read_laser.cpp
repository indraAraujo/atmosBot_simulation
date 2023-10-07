#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"  // sensor_msgs/msg/LaserScan

using namespace std::chrono_literals;

class ReadLaser : public rclcpp::Node{
    public: 
        ReadLaser() : Node("read_laser"){ 
            subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&ReadLaser::topic_callback, this, std::placeholders::_1));      
        }
    
    private:
        void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr _msg){
             RCLCPP_INFO(this->get_logger(), "OUVINDO: '%f' e %f", _msg->ranges[0], _msg->ranges[100]);
        }
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char *argv[]){
     rclcpp::init(argc, argv);
     auto node = std::make_shared<ReadLaser>();
     RCLCPP_INFO(node->get_logger(), "INSCREVENDO");
     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;
}

