#include <chrono>
#include <functional>
#include <memory>
#include <algorithm> 
#include <iostream>
#include <ctime>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"  // sensor_msgs/msg/LaserScan
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class AvoidObstacles : public rclcpp::Node{
    public: 
        AvoidObstacles() : Node("avoid_obstacles"){ 
            std::this_thread::sleep_for(std::chrono::seconds(4));
            subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&AvoidObstacles::laser_callback, this, std::placeholders::_1));      
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            timer_ = this->create_wall_timer(1ms, std::bind(&AvoidObstacles::avoid_obstacle_callback, this));
            laser_ranges_.fill(0.0);
            scan_available_ = false;
        }
        
    private:
        //Armazenamento das leituras do laser na variável compartilhada entre callbacks
        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr _scan){
            for (int i = 0; i < 24; i++) {
                laser_ranges_[i] = _scan->ranges[i];
            }
            scan_available_ = true;

        }

        //Controle do robô para evitar os obstáculos
        void avoid_obstacle_callback(){
            //Valor mínimo dentre os 45 graus a extrema esquerda
            float left_min_range = std::numeric_limits<float>::max();
            for(int i=20; i>17; i--){
                if((!std::isinf(laser_ranges_[i]))&&(laser_ranges_[i]<left_min_range)){
                    left_min_range=laser_ranges_[i];
                }
            }
          
            //Valor mínimo dentre os 45 graus a extrema direita
            float right_min_range = std::numeric_limits<float>::max();
            for(int i=4; i<7; i++){
                if((!std::isinf(laser_ranges_[i]))&&(laser_ranges_[i]<right_min_range)){
                    right_min_range=laser_ranges_[i];
                }
            }

            //Valor do meio
            float middle_range= laser_ranges_[0];

            //Mensagem a ser publicada no tópico de velocidade
            auto message = geometry_msgs::msg::Twist();

            //Valores a serem publicados pela mensagem de velocidade
            float linear_velocity = 0.0; //m/s
            float angular_velocity = 0.0; //rad/s

                std::cout << "NOVA LEITURA"  << std::endl;
                std::cout << "Left range: " << left_min_range << std::endl;
                std::cout << "Right range: " << right_min_range << std::endl;
                std::cout << "Middle range: " << middle_range << std::endl;
                std::cout << "---------------"  << std::endl;

            
            if(scan_available_){ //Valida se as leituras do scan já estao disponíveis
                if(middle_range<=0.9){ // Identifica se tem um obstáculo logo a frente
                    if(left_min_range<=0.5 && right_min_range>=0.5){ //Identifica se pode virar para a direita
                        linear_velocity = 0.0;
                        angular_velocity = -0.5; //vira para a direita
                    }else if(left_min_range>=0.5 && right_min_range<=0.5){ //Identifica se pode virar para a esquerda
                        linear_velocity = 0.0; 
                        angular_velocity = 0.5; //vira para a esquerda
                    }else if(left_min_range<=0.5 && right_min_range<=0.5){ 
                        linear_velocity = -0.2; //robô dá ré pois não consegue virar para lugar nenhum
                        angular_velocity = 0.0;
                    }else{ //ambos os lados do robô estão sem obstáculos muito próximos
                        linear_velocity=0.0;
                        angular_velocity=0.5; //vira a esquerda
                    }
                   
                }else{ // não existe nenhum obstáculo na frente
                    if(left_min_range<=0.5 && right_min_range>=0.5){ //Identifica se não está batendo em nada do lado esquerdo
                        linear_velocity = 0.0;
                        angular_velocity = -0.2; //vira para a direita
                    }else if(left_min_range>=0.5 && right_min_range<=0.5){ //Identifica se não está batendo em nada do lado direito
                        linear_velocity = 0.0; 
                        angular_velocity = 0.2; //vira para a esquerda
                    }else{ 
                        linear_velocity = 0.2; // robô segue em frente
                        angular_velocity = 0.0;
                    }
                 }
            }

            //Insere os comandos na mensagem a ser publicada no tópico de velocidade
            message.linear.x = linear_velocity;
            message.angular.z = angular_velocity;

            //Publica o comando da velocidade
            publisher_->publish(message);
        }



        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::array<float, 24> laser_ranges_;
        bool scan_available_;
};

int main(int argc, char *argv[]){
     rclcpp::init(argc, argv);
     auto node = std::make_shared<AvoidObstacles>();
     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;
}

//  if((right_min_range - left_min_range)>0.9){ //valida se tem um obstáculo do lado esquerdo extremo
//                         if(left_min_range>=0.9){  //objeto não tá tão próximo 
//                             linear_velocity = 0.2; // robô continua para frente
//                         }else{ // objeto está próximo
//                             linear_velocity = 0.0;
//                             angular_velocity = -0.5; // robô vira para a direita
//                         }
//                     } else if((left_min_range - right_min_range)>0.9){ //valida se tem um obstáculo do lado direito extremo
//                         if(right_min_range>=0.9){  //objeto não tá tão próximo 
//                             linear_velocity = 0.2; // robô continua para frente
//                         }else{ // objeto está próximo
//                             linear_velocity = 0.0;
//                             angular_velocity = 0.5; // robô vira para a esquerda
//                         }
//                     } else { //tem um obstáculo muito próximo de um dos extremos ou não tem obstáculo algum próximo
//                         if((left_min_range<=0.9)||(right_min_range<=0.9)){ //valida se o obstáculo está muito próximo
//                             linear_velocity = 0.0;
//                             if(right_min_range<=left_min_range){ // valida se o obstáculo está do lado direito
//                                 angular_velocity = 0.5; // vira o robô para a esquerda
//                             }else{
//                                 angular_velocity = -0.5; // vira o robô para a direita já que o obstáculo está do lado esquerdo
//                             }
//                         }else{ // não existe nenhum obstáculo próximo aos lados
//                             linear_velocity = 0.0; 
//                             angular_velocity = 0.5; //robô vira para a esquerda
//                         }
//                     }