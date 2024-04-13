#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/alive_turtles.hpp"
#include "my_robot_interfaces/srv/kill_turtle.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

 
class TurtleControllerNode : public rclcpp::Node
{
  public:
    TurtleControllerNode() : Node("turtle_controller")
    {
      alive_turtles_subscriber = this->create_subscription<my_robot_interfaces::msg::AliveTurtles>(
        "alive_turtles", 10,
        std::bind(&TurtleControllerNode::array_updated, this, std::placeholders::_1));
      
      location_subscriber = this->create_subscription<turtlesim::msg::Pose>(
        "turtle1/pose", 10,
        std::bind(&TurtleControllerNode::update_current_location, this, std::placeholders::_1));

        cmd_publisher = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    }
  
  private:
    void update_current_location(const turtlesim::msg::Pose msg){
      x_location = msg.x;
      y_location = msg.y;
      theta_angle = msg.theta;
    }
    void array_updated(const my_robot_interfaces::msg::AliveTurtles msg){
      if (msg.turtle_array.size() != 0 ){
        kill_turtle(msg.turtle_array[0].name, msg.turtle_array[0].x, msg.turtle_array[0].y);
      }
    }


    void kill_turtle(std::string name, float x, float y){
      bool turtle_caught = false;

      while(!turtle_caught){

        double dist_x = x - x_location;
        double dist_y = y - y_location;
        double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);

        auto msg = geometry_msgs::msg::Twist();

        if (distance > 0.5) {

          //position
          msg.linear.x = 2 * distance;

          //orientation
          double steering_angle = std::atan2(dist_y, dist_x);
          double angle_diff = steering_angle - theta_angle;
          
          if (angle_diff > M_PI)
          {
              angle_diff -= 2 * M_PI;
          }
          else if (angle_diff < -M_PI)
          {
              angle_diff += 2 * M_PI;
          }
          msg.angular.z = 6 * angle_diff;
        }
        else {
          //target reached!
          msg.linear.x = 0.0;
          msg.angular.z = 0.0;
          turtle_caught = true;
        }

        cmd_publisher->publish(msg);

      }

      threads_.push_back(std::thread(std::bind(&TurtleControllerNode::callKillService, this, name)));
    }


    void callKillService(std::string name){
      auto client = this->create_client<my_robot_interfaces::srv::KillTurtle>("kill_turtle");
      while (!client->wait_for_service(std::chrono::seconds(1)))
      {
          RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
      }

      auto request = std::make_shared<my_robot_interfaces::srv::KillTurtle::Request>();
      request->kill = true;
      request->name = name;

      auto future = client->async_send_request(request);

      try
      {
          auto response = future.get();
          if(response->killed){
            RCLCPP_INFO(this->get_logger(), "A turtle called %s was killed", name.c_str());
          }
      }
      catch (const std::exception &e)
      {
          RCLCPP_ERROR(this->get_logger(), "Service call failed");
      }
    }
    rclcpp::Subscription<my_robot_interfaces::msg::AliveTurtles>::SharedPtr alive_turtles_subscriber;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr location_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher;
    std::vector<std::thread> threads_;
    float x_location;
    float y_location;
    float theta_angle;
};
 
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
