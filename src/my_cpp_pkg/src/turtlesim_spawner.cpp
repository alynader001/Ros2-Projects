#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
class TurtlesimSpawnerNode : public rclcpp::Node
{
 public:
   TurtlesimSpawnerNode() : Node("turtlesim_spawner")
   {
     thread1_ = std::thread(std::bind(&TurtlesimSpawnerNode::spawn_service_caller, this));
   }
 private:
 void spawn_service_caller(){
   timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&TurtlesimSpawnerNode::call_spawn_service, this));
 }
 void call_spawn_service()
 std::thread thread1_;
};
int main(int argc, char **argv)
{
 rclcpp::init(argc, argv);
 auto node = std::make_shared<TurtlesimSpawnerNode>();
 rclcpp::spin(node);
 rclcpp::shutdown();
 return 0;
}