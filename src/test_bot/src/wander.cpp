#include <chrono>
#include <functional>
#include <memory>
#include <algorithm> 
#include <iostream>
#include <ctime>
#include <thread>
#include <cmath>

#include "rclcpp_action/rclcpp_action.hpp"
#include "test_bot/action/wander.hpp"  
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"  // sensor_msgs/msg/LaserScan
#include "geometry_msgs/msg/twist.hpp"


using namespace std::chrono_literals;


class Wander : public rclcpp::Node{
    public: 
        Wander() : Node("wander"){ 
            action_server_ = rclcpp_action::create_server<test_bot::action::Wander>(
                this, 
                "wandering",
                std::bind(&Wander::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&Wander::handle_cancel, this, std::placeholders::_1),
                std::bind(&Wander::handle_accepted, this, std::placeholders::_1));

            auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));     
            qos_profile.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE);                           // FIFO de até 10 mensagens
            

            //aguarda 1 minuto para todos os componentes iniciarem na simulação
            std::this_thread::sleep_for(std::chrono::seconds(60)); 
            
            //cria a inscrição pro tópico do laser para conseguir suas informações
            subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", qos_profile, std::bind(&Wander::laser_callback, this, std::placeholders::_1));      
            //cria um publicador pro tópico da velocidade para controlar a locomoção do robo
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos_profile);
            //define um período de 1 ms para executar o algoritmo de evitar obstáculos
            timer_ = this->create_wall_timer(1ms, std::bind(&Wander::avoid_obstacle_callback, this));
            
            //definição de variáveis comuns
            laser_ranges_.fill(0.0);
            scan_available_ = false;
            angle_min_ = 0.0;
            angle_increment_=0.0;
            ranges_size_=0;
            start_wandering_=false;
        }
        
    private:

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const test_bot::action::Wander::Goal> goal) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<test_bot::action::Wander>> goal_handle) {
            start_wandering_=false;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<test_bot::action::Wander>> goal_handle) {
            start_wandering_=true;
        }
        
        //Armazenamento das leituras do laser na variável compartilhada entre callbacks
        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr _scan){
            ranges_size_ = sizeof(_scan->ranges);
            for (int i = 0; i < ranges_size_; i++) {
                laser_ranges_[i] = _scan->ranges[i];
            }
            angle_increment_=_scan->angle_increment;
            angle_min_=_scan->angle_min;

            scan_available_ = true;

        }

        //Controle do robô para evitar os obstáculos
        void avoid_obstacle_callback(){

            //Mensagem a ser publicada no tópico de velocidade
            auto message = geometry_msgs::msg::Twist();

            //Valores a serem publicados pela mensagem de velocidade
            float linear_velocity = 0.0; //m/s
            float angular_velocity = 0.0; //rad/s

            if(scan_available_ && start_wandering_){ //Valida se as leituras do scan já estao disponíveis

                //Valor mínimo dentre os 90 graus a esquerda
                float right_min_range = std::numeric_limits<float>::max();
                for(int i=18; i<23; i++){
                    if((!std::isinf(laser_ranges_[i]))&&(laser_ranges_[i]<right_min_range)){
                        right_min_range=laser_ranges_[i];
                    }
                }
                
                //Valor mínimo dentre os 90 graus a direita
                float left_min_range = std::numeric_limits<float>::max();
                for(int i=1; i<7; i++){
                    if((!std::isinf(laser_ranges_[i]))&&(laser_ranges_[i]<left_min_range)){
                        left_min_range=laser_ranges_[i];
                    }
                }

                //Valor do meio
                float middle_range= laser_ranges_[0];

                std::cout << "NOVA LEITURA"  << std::endl;
                std::cout << "Left range: " << left_min_range << std::endl;
                std::cout << "Right range: " << right_min_range << std::endl;
                std::cout << "Middle range: " << middle_range << std::endl;
                std::cout << "---------------"  << std::endl;
          
                if(middle_range<=0.9){ // Identifica se tem um obstáculo logo a frente
                    if(left_min_range<=0.5 && right_min_range>0.5){ //Identifica se pode virar para a direita
                        linear_velocity = 0.0;
                        angular_velocity = -0.5; //vira para a direita

                    }else if(right_min_range<=0.5 && left_min_range>0.5){ //Identifica se pode virar para a esquerda
                        linear_velocity = 0.0; 
                        angular_velocity = 0.5; //vira para a esquerda
                    }else if(left_min_range<=0.5 && right_min_range<=0.5){ 
                        angular_velocity = 0.0;
                        linear_velocity = -0.2; //robô dá ré pois não consegue virar para lugar nenhum
                    }else{ //ambos os lados do robô estão sem obstáculos muito próximos
                        linear_velocity=0.0;
                        angular_velocity=0.5; //vira a esquerda
                    }
                }else{ // não existe nenhum obstáculo na frente
                    if(left_min_range<=0.5 && right_min_range>0.5){ //Identifica se pode virar para a direita
                        linear_velocity = 0.0;
                        angular_velocity = -0.5; //vira para a direita

                    }else if(right_min_range<=0.5 && left_min_range>0.5){ //Identifica se pode virar para a esquerda
                        linear_velocity = 0.0; 
                        angular_velocity = 0.5; //vira para a esquerda
                    }else{ 
                        angular_velocity = 0.0;
                        linear_velocity = 0.2; // robô segue em frente
                    }
                 }
            }

            //Insere os comandos na mensagem a ser publicada no tópico de velocidade
            message.linear.x = linear_velocity;
            message.angular.z = angular_velocity;

            //Publica o comando da velocidade
            publisher_->publish(message);
        }
         

        rclcpp_action::Server<test_bot::action::Wander>::SharedPtr action_server_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::array<float, 360> laser_ranges_;
        bool scan_available_;
        bool start_wandering_;
        float angle_min_;
        float angle_increment_;
        int ranges_size_;
};

int main(int argc, char *argv[]){
     rclcpp::init(argc, argv);
     auto node = std::make_shared<Wander>();
     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;
}
