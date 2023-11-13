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
        /*
            indica se há obstáculos no seu raio,
            dando maior importância para os obstáculos a frente

            olha os raios frontais e depois os laterais
        */
        void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr _scan){
            //  RCLCPP_INFO(this->get_logger(), "Intervalo do scanner %ld", sizeof(_scan->ranges)); // identificar a quantidade de lasers
            float middleLaser =  _scan->ranges[0];
            float leftLaser = _scan->ranges[12];
            float rightLaser = _scan->ranges[23];

            float minimumDistance = 0.5;
            
            RCLCPP_INFO(this->get_logger(), "Extremidade 1: %f \n Meio: %f \n Extremidade 2: %f", leftLaser, middleLaser, rightLaser);
            
            if(middleLaser<minimumDistance){ //verifica obstáculo na frente
             RCLCPP_INFO(this->get_logger(), "OBSTÁCULO A FRENTE ");
             check_sides(leftLaser, rightLaser, minimumDistance); //verifica obstáculos laterais
            }else if(leftLaser<minimumDistance){
             RCLCPP_INFO(this->get_logger(), "OBSTÁCULO NO LADO ESQUERDO ");
            }else if(rightLaser<minimumDistance){
             RCLCPP_INFO(this->get_logger(), "OBSTÁCULO NO LADO DIREITO ");
            }
        }

        /*
            verifica obstáculos pelos raios extremos 
            de acordo com a distância mínima definida
        */
        void check_sides(float leftLaser, float rightLaser, float minimumDistance){
            if(leftLaser>minimumDistance){
                RCLCPP_INFO(this->get_logger(), "LIVRE PARA A ESQUERDA ");
             } else if(rightLaser > minimumDistance){
                RCLCPP_INFO(this->get_logger(), "LIVRE PARA A DIREITA ");
             }
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

