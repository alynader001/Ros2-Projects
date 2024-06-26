#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "my_robot_interfaces/srv/kill_turtle.hpp"
#include "turtlesim/srv/kill.hpp"
#include "my_robot_interfaces/msg/alive_turtles.hpp"
#include "my_robot_interfaces/msg/turtle_object.hpp"
#include <random>
#include <string>

using std::placeholders::_1;
using std::placeholders::_2;

class TurtlesimSpawnerNode : public rclcpp::Node
{
  public:
    TurtlesimSpawnerNode() : Node("turtlesim_spawner")
    {
      //thread1_ = std::thread(std::bind(&TurtlesimSpawnerNode::call_spawn_service, this));
      //thread1_.join();
      threads_.push_back(std::thread(std::bind(&TurtlesimSpawnerNode::call_spawn_service, this)));
      server_ = this->create_service<my_robot_interfaces::srv::KillTurtle>(
        "kill_turtle",
        std::bind(&TurtlesimSpawnerNode::callbackKillTurtle, this, _1, _2));
      publisher_ = this->create_publisher<my_robot_interfaces::msg::AliveTurtles>("alive_turtles", 10);
      RCLCPP_INFO(this->get_logger(), "Service server has been started.");
    }

  private:
    void test_func(){
      RCLCPP_INFO(this->get_logger(), "Executing every 1 second.");
    }
    void spawn_service_caller(){
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&TurtlesimSpawnerNode::call_spawn_service, this));
    }

    void call_spawn_service(){
      auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
      while (!client->wait_for_service(std::chrono::seconds(1)))
      {
        RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
      }

      
      //To generate random numbers which will be used as parameters for the newly generated turtle.
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_int_distribution<int> dist1(0, 10);
      std::uniform_int_distribution<int> dist2(0, 360);
      
      while(true){
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = dist1(gen);
        request->y = dist1(gen);
        request->theta = dist2(gen);

        auto future = client->async_send_request(request);
        try
        {
          auto response = future.get();
          RCLCPP_INFO(this->get_logger(), "A new turtle named %s has spawned", response->name.c_str());
          auto turtle_ = my_robot_interfaces::msg::TurtleObject();
          turtle_.name = response->name;
          turtle_.x = request->x;
          turtle_.y = request->y;
          alive_turtles_array.push_back(turtle_);
          auto msg = my_robot_interfaces::msg::AliveTurtles();
          msg.turtle_array = alive_turtles_array;
          publisher_->publish(msg);
        }
          catch (const std::exception &e)
        {
          RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
      }      
    }

    void callbackKillTurtle(const my_robot_interfaces::srv::KillTurtle::Request::SharedPtr request,
      const my_robot_interfaces::srv::KillTurtle::Response::SharedPtr response){
        if (request->kill){
          response->killed = true;
          threads_.push_back(std::thread(std::bind(&TurtlesimSpawnerNode::call_kill_service,
            this, request->name, response)));
        }
    }

    void call_kill_service(std::string name,
      const my_robot_interfaces::srv::KillTurtle::Response::SharedPtr result){

      auto turtle_ = std::find_if(alive_turtles_array.begin(), alive_turtles_array.end(),
       [&name](const my_robot_interfaces::msg::TurtleObject search_obj){
        return search_obj.name == name;
       });
      if(turtle_ != alive_turtles_array.end()){
        alive_turtles_array.erase(turtle_);
      }
      auto msg = my_robot_interfaces::msg::AliveTurtles();
      msg.turtle_array = alive_turtles_array;
      publisher_->publish(msg);
      auto client = this->create_client<turtlesim::srv::Kill>("kill");
      while (!client->wait_for_service(std::chrono::seconds(1)))
      {
        RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
      }
      auto request = std::make_shared<turtlesim::srv::Kill::Request>();
      request->name = name;
      auto future = client->async_send_request(request);
      try
      {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "A turtle called %s was killed", name.c_str());
        result->killed = true;
        }
        catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Service call failed");
        result->killed = false;
      }
    } 

    std::vector<std::thread> threads_;
    //std::thread thread1_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<my_robot_interfaces::srv::KillTurtle>::SharedPtr server_;
    rclcpp::Publisher<my_robot_interfaces::msg::AliveTurtles>::SharedPtr publisher_;
    std::vector<my_robot_interfaces::msg::TurtleObject> alive_turtles_array;
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtlesimSpawnerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}