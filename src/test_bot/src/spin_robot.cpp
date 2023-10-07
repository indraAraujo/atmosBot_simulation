#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp" //tipagem do tópico no qual vai ser publicado o comando

using namespace std::chrono_literals;

//Classe que executa a publicação no tópico
class SpinRobot : public rclcpp::Node{
    public: //construtor da classe
        /*
            Node("") -> nome do nó que será criado que vai realizar as publicações
            count_() -> variável contador para monitorar quantidade de publicações realizadas pelo nó 
        */
        SpinRobot() : Node("spin_robot"), count_(0){ 
            /*
                 < > -> tipagem do tópico no qual será publicado
                 ("", 0) -> nome do tópico e quantidade máxima de publicações na "fila"
            */
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10); //cria um novo publicador
            timer_ = this -> create_wall_timer( //definição de um timer para ficar publicando o comando (dado pelo ::timer_callback) no tópico
                500ms, std::bind(&SpinRobot::timer_callback, this)
            );
        }
    private: //métodos da classe
        void timer_callback(){ //método que será executado quando o timer (definido acima) for acionado
            auto message = geometry_msgs::msg::Twist(); // criação do comando a ser publicado
            message.linear.x= 0.5; //definição do comando
            message.angular.z = 0.3; //continuação da definição do comando
            RCLCPP_INFO(this->get_logger(), "PUBLICANDO: '%f.2' and %f.2", message.linear.x, message.angular.z);
            publisher_ -> publish(message); //publicação do comando no tópico
        }

        //declarações
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        size_t count_;
};

//Main para executar as classes criadas
int main(int argc, char *argv[]){
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<SpinRobot>());
     rclcpp::shutdown();
     return 0;
}