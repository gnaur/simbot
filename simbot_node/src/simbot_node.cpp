#include <cstdio>

#include <rclcpp/rclcpp.hpp>

class SimbotNode : public rclcpp::Node 
{
public:

  SimbotNode() : Node("simbot_node")
  {
   
    printf("simbot node running\n");

  };

  ~SimbotNode() {

  };

private:

};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SimbotNode>());
 	rclcpp::shutdown();

	return 0;
}