#include <ros/ros.h>
#include "std_msgs/String.h"

#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

#include <stdio.h>

class TeleopKeybrd
{
public:
    TeleopKeybrd(const ros::NodeHandle& nh, std::string robot, std::string cmd_vel);
    void keyLoop();

protected:

private:
    ros::NodeHandle m_nh;

    double mLinear;
    double mAngular;
    double mMaxLin;
    double mMaxAng;

    double mLinStep;
    double mAngStep;

    double mSpeedRatio;

    int mKeyTimeout;

    ros::Publisher mVelPub;

    bool mLocked;

    ros::Publisher mPubVelControl, mPubEnableControl;
};


