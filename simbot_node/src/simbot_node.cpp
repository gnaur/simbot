#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "UDPSocket.hpp"
#include "NetThread.hpp"

#include "NetPackets.h"
#include <functional>
using namespace std::placeholders; // for _1, _2 etc.

static const  int MAIN_COMS_PORT=49152;
#define SERVER "10.2.99.201"

class SimbotNode : public rclcpp::Node 
{
public:

  SimbotNode() : Node("simbot_node"), comsSocket((const int)MAIN_COMS_PORT,(const char*)SERVER, [=](const MAIN_COMS_OUT_PACKET & data) { this->doMainComs(data); } )
  {
    netHandler.start();
    netHandler+comsSocket;

    motor_enable_sub = create_subscription<std_msgs::msg::String>("/motor_en", 10, std::bind(&SimbotNode::doMotorEn, this,_1));

    printf("simbot node running\n");

  };

  ~SimbotNode() {
    netHandler.stop();
  };

  void doMainComs(const MAIN_COMS_OUT_PACKET & data) {
    printf("%d",data.arg);
  }

  void doMotorEn(const std_msgs::msg::String::SharedPtr msg) {
      MAIN_COMS_IN_PACKET packet;

      memset(&packet,0,sizeof(packet));
      packet.header.seq_num++;
      packet.cmd = (MainComsInCmd)htonl(CMD_MTR_EN);
     
      bool en=msg->data=="enable";

      packet.arg = htonl(en?1:0);
      comsSocket.send(packet);

      RCLCPP_INFO_STREAM(rclcpp::get_logger("simbot_node"),"Motors " << (en?"Enabled":"Disabled"));
  }

private:

  UDPSocket<MAIN_COMS_OUT_PACKET,MAIN_COMS_IN_PACKET> comsSocket;
  NetThread netHandler;

  //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr read_publisher;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motor_enable_sub;


};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SimbotNode>());
 	rclcpp::shutdown();

	return 0;
}