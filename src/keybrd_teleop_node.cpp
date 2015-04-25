#include <keybrd_teleop/teleop_key.h>
#include "std_msgs/String.h"

std::string robot_name_string = "robot";
std::string cmd_vel_string = "cmd_vel";

std::string name_node = "keyboard_drive_bridge";

int main(int argc, char** argv)
{
	ros::init(argc, argv, name_node);
	ros::NodeHandle nh;

    //Load configuration
    if (nh.hasParam(name_node + "/cmd_vel")) {
            nh.getParam(name_node + "/cmd_vel", cmd_vel_string);
	} else {
            nh.setParam(name_node + "/cmd_vel", cmd_vel_string);
    }
    TeleopKeybrd teleop(nh, robot_name_string, cmd_vel_string );

    teleop.keyLoop();

  return(0);
}
