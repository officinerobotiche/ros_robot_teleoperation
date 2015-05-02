#include <keybrd_teleop/teleop_key.h>
#include "std_msgs/String.h"

std::string cmd_vel_string = "cmd_vel";

std::string name_node = "keyboard_drive_bridge";

int main(int argc, char** argv)
{
	ros::init(argc, argv, name_node);
	ros::NodeHandle nh;

    //Load configuration
    TeleopKeybrd teleop(nh, cmd_vel_string );

    teleop.keyLoop();

  return(0);
}
