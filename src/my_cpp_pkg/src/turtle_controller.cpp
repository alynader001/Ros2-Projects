#include "rclcpp/rclcpp.hpp"
 
class TurtleControllerNode : public rclcpp::Node
{
public:
  TurtleControllerNode() : Node("turtle_controller")
  {
    
  }
 
private:
};
 
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
