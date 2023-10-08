#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"  // sensor_msgs/msg/LaserScan
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class AvoidObstacles : public rclcpp::Node{
    public: 
        AvoidObstacles() : Node("avoid_obstacles"){ 
            subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&AvoidObstacles::topic_callback, this, std::placeholders::_1));      
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        }
    
    private:
        void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr _scan){
            float min = 10;
            long unsigned int scanRangeLength = sizeof(_scan->ranges);

            for(long unsigned int i=0; i<scanRangeLength;i++){
                float current = _scan->ranges[i];
                if(current<min){
                    min=current;
                }
            }

            auto command = this->getCommand(min);
            publisher_->publish(command);
        }

        geometry_msgs::msg::Twist getCommand(float distance){
            auto command = geometry_msgs::msg::Twist();

            if(distance<1){
                command.linear.x=0;
                command.angular.z = 0.3;
            }else{
                command.linear.x=0.3;
                command.angular.z = 0;
            }
            
            return command;
        }

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

};

int main(int argc, char *argv[]){
     rclcpp::init(argc, argv);
     auto node = std::make_shared<AvoidObstacles>();
     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;
}

