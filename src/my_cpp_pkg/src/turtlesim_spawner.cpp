#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include <random>
class TurtlesimSpawnerNode : public rclcpp::Node
{
  public:
    TurtlesimSpawnerNode() : Node("turtlesim_spawner")
    {
      //thread1_ = std::thread(std::bind(&TurtlesimSpawnerNode::call_spawn_service, this));
      //thread1_.join();
      threads_.push_back(std::thread(std::bind(&TurtlesimSpawnerNode::call_spawn_service, this)));
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
          RCLCPP_INFO(this->get_logger(), "A new turtle has spawned");
        }
          catch (const std::exception &e)
        {
          RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }      
    }
    std::vector<std::thread> threads_;
    //std::thread thread1_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtlesimSpawnerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}