#include <chrono>
#include <functional>
#include <memory>
#include <algorithm> 
#include <iostream>
#include <ctime>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"  // sensor_msgs/msg/LaserScan
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class AvoidObstacles : public rclcpp::Node{
    public: 
        AvoidObstacles() : Node("avoid_obstacles"){ 
            //definição do perfil de QoS para melhorar a comunicação entre os nós
            auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));     
            qos_profile.reliability(rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE);                           // FIFO de até 10 mensagens
            

            //aguarda 4 segundos para todos os componentes iniciarem na simulação
            std::this_thread::sleep_for(std::chrono::seconds(4)); 
            
            //cria a inscrição pro tópico do laser para conseguir suas informações
            subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", qos_profile, std::bind(&AvoidObstacles::laser_callback, this, std::placeholders::_1));      
            //cria um publicador pro tópico da velocidade para controlar a locomoção do robo
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos_profile);
            //define um período de 1 ms para executar o algoritmo de evitar obstáculos
            timer_ = this->create_wall_timer(1ms, std::bind(&AvoidObstacles::avoid_obstacle_callback, this));
            
            //definição de variáveis comuns
            laser_ranges_.fill(0.0);
            scan_available_ = false;
            angle_min_ = 0.0;
            angle_increment_=0.0;
            ranges_size_=0;
        }
        
    private:
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

        //Resgata a leitura de um raio de acordo com o ângulo proporcionado em graus
        int get_range(float degrees){
            float radians = degrees * (M_PI / 180.0); //transforma o parâmetro recebido em radianos
            float index = ((radians-angle_min_)/angle_increment_);//calcula o indice do vetor de leituras do sensor em que está o angulo recebido
            int roundedIndex = static_cast<int>(round(index)); //arrendonda o valor do indice para o numero inteiro mais próximo
            float range = laser_ranges_[roundedIndex]; //resgata a leitura de acordo com o indice calculado

            return range; //retorna a leitura
        }

        //Consegue o angulo ao qual se refere um indice do vetor de leituras do sensor
        float get_angle(int index){
            float radians = (index*angle_increment_)+angle_min_; //calcula o angulo em radianos a partir do indice recebido
            float degrees = radians * (180.0 / M_PI); //calcula o equivalente em graus

            return degrees; //retorna o angulo em graus
        }

        //Controle do robô para evitar os obstáculos
        void avoid_obstacle_callback(){

            //Mensagem a ser publicada no tópico de velocidade
            auto message = geometry_msgs::msg::Twist();

            //Valores a serem publicados pela mensagem de velocidade
            float linear_velocity = 0.0; //m/s
            float angular_velocity = 0.0; //rad/s

            if(scan_available_){ //Valida se as leituras do scan já estao disponíveis

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

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::array<float, 360> laser_ranges_;
        bool scan_available_;
        float angle_min_;
        float angle_increment_;
        int ranges_size_;
};

int main(int argc, char *argv[]){
     rclcpp::init(argc, argv);
     auto node = std::make_shared<AvoidObstacles>();
     rclcpp::spin(node);
     rclcpp::shutdown();
     return 0;
}

