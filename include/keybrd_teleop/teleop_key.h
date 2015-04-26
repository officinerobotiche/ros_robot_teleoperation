#include <ros/ros.h>
#include "std_msgs/String.h"

#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

#include <stdio.h>
#include <csignal>



class TeleopKeybrd
{
public:
    TeleopKeybrd(const ros::NodeHandle& nh, std::string cmd_vel);
    void keyLoop();

protected:
    // >>>>> Ctrl+C handler
    /*! Ctrl+C handler
     */
    static void sighandler(int signo)
    {
        TeleopKeybrd::_stopping = (signo == SIGINT);
    }
    // <<<<< Ctrl+C handler

private:
    static bool _stopping;

    ros::NodeHandle m_nh;

    double mLinear;
    double mAngular;
    double mMaxLin;
    double mMaxAng;

    double mLinStep;
    double mAngStep;

    double mSpeedRatio;

    int mKeyTimeout;

    bool mLocked;

    ros::Publisher mPubVelControl, mPubEnableControl;
};


