#include <keybrd_teleop/teleop_key.h>
#include <geometry_msgs/Twist.h>
#include <ncurses.h>
#include <math.h>
#include <string>

using namespace std;

bool TeleopKeybrd::_stopping = false;

TeleopKeybrd::TeleopKeybrd(const ros::NodeHandle& nh, std::string cmd_vel ):
    mLinear(0.0),
    mAngular(0.0),
    mMaxLin(1.5),
    mMaxAng(6.28),
    mLinStep(0.05),
    mAngStep(0.157),
    mSpeedRatio(1.0),
    mKeyTimeout(50),
    mLocked(false)
{
    // >>>>> Ctrl+C handling
    struct sigaction sigAct;
    memset( &sigAct, 0, sizeof(sigAct) );
    sigAct.sa_handler = TeleopKeybrd::sighandler;
    sigaction(SIGINT, &sigAct, 0);
    // <<<<< Ctrl+C handling

    mPubVelControl = m_nh.advertise<geometry_msgs::Twist>( cmd_vel, 10);

    // >>>>> Parameters
    string nodeName = ros::this_node::getName();
    string nameSpace = ros::this_node::getNamespace();

    string paramStr;

    paramStr = ( nameSpace + nodeName + "/teleop_key/Max_linear");
    if(m_nh.hasParam( paramStr ))
        m_nh.getParam(paramStr, mMaxLin);
    else
        m_nh.setParam(paramStr, mMaxLin );

    paramStr = ( nameSpace + nodeName + "/teleop_key/Max_angular");
    if(m_nh.hasParam( paramStr ))
        m_nh.getParam(paramStr, mMaxAng);
    else
        m_nh.setParam(paramStr, mMaxAng );

    paramStr = ( nameSpace + nodeName + "/teleop_key/Lin_step");
    if(m_nh.hasParam( paramStr ))
        m_nh.getParam(paramStr, mMaxAng);
    else
        m_nh.setParam(paramStr, mMaxAng );

    paramStr = ( nameSpace + nodeName + "/teleop_key/Ang_step");
    if(m_nh.hasParam( paramStr ))
        m_nh.getParam(paramStr, mAngStep);
    else
        m_nh.setParam(paramStr, mAngStep );

    paramStr = ( nameSpace + nodeName + "/teleop_key/Key_timeout");
    if(m_nh.hasParam( paramStr ))
        m_nh.getParam(paramStr, mKeyTimeout);
    else
        m_nh.setParam(paramStr, mKeyTimeout );
    // <<<<< Parameters

}

void TeleopKeybrd::keyLoop()
{
    // >>>>> nCurses initization
    initscr();
    keypad(stdscr, TRUE);
    cbreak();
    noecho();
    timeout(mKeyTimeout);
    // <<<<< nCurses initization

    int c;
    bool dirty=false;

    c = getch();

    ROS_INFO_STREAM("-----------------------------------\r");
    ROS_INFO_STREAM("      Keyboard teleoperation       \r");
    ROS_INFO_STREAM("-----------------------------------\r");
    ROS_INFO_STREAM("- Use arrow keys to move the robot.\r");
    ROS_INFO_STREAM("- Press SPACEBAR to stop the robot.\r");
    ROS_INFO_STREAM("- Press Q to exit.\r");
    ROS_INFO_STREAM("- Press 1 for Max_speed.\r");
    ROS_INFO_STREAM("- Press 2 for Max speed/2.\r");
    ROS_INFO_STREAM("- Press 3 for Max speed/3.\r");
    ROS_INFO_STREAM("- Press L to toggle lock speeds\r");
    ROS_INFO_STREAM("-----------------------------------\r");

    bool stop = false;

    while(1)
    {
        if( stop || TeleopKeybrd::_stopping )
        {
            ROS_INFO_STREAM( "... exiting ... Robot stopped!");
            
            break;
        }

        dirty = false;

        c = getch();

        double linStep = mLinStep*mSpeedRatio;
        double angStep = mAngStep*mSpeedRatio;

        ROS_DEBUG_STREAM("Key pressed: " << c << "\r");

        switch(c)
        {
        case 'q':
        case 'Q':
        {
            ROS_DEBUG_STREAM("EXIT, 'Q' pressed \r");
            stop = true;
            continue;
        }
        case KEY_LEFT:
        {
            ROS_DEBUG_STREAM("LEFT\r");
            mAngular += angStep;
            dirty = true;
            break;
        }
        case KEY_RIGHT:
        {
            ROS_DEBUG_STREAM("RIGHT\r");
            mAngular -= angStep;
            dirty = true;
            break;
        }
        case KEY_UP:
        {
            ROS_DEBUG_STREAM("UP\r");
            mLinear += linStep;
            dirty = true;
            break;
        }
        case KEY_DOWN:
        {
            ROS_DEBUG_STREAM("DOWN\r");
            mLinear -= linStep;
            dirty = true;
            break;
        }
        case ' ':
        {
            ROS_DEBUG_STREAM("STOP\r");
            mLinear = 0.0;
            mAngular = 0.0;
            dirty = true;
            break;
        }
        case '1':
        {
            mSpeedRatio = 1.0;
            dirty = true;
            break;
        }
        case '2':
        {
            mSpeedRatio = 0.5;
            dirty = true;
            break;
        }
        case '3':
        {
            mSpeedRatio = 0.333333;
            dirty = true;
            break;
        }
        case 'l':
        case 'L':
        {
            ROS_DEBUG_STREAM("Speed lock toggled");

            mLocked = mLocked?false:true;
            break;
        }
        default:
        {
            if( mLinear > 0 )
            {
                if(!mLocked)
                {
                    mLinear -= linStep;
                    dirty = true;
                }
            }
            else if( mLinear < 0  )
            {
                if(!mLocked)
                {
                    mLinear += linStep;
                    dirty = true;
                }
            }


            if( mAngular > 0 )
            {
                if(!mLocked)
                {
                    mAngular -= angStep;
                    dirty = true;
                }
            }
            else if( mAngular < 0  )
            {
                if(!mLocked)
                {
                    mAngular += angStep;

                    dirty = true;
                }
            }
        }
        }

        geometry_msgs::Twist vel;

        // >>>>> Saturations
        if( mLinear > mMaxLin*mSpeedRatio )
            mLinear = mMaxLin*mSpeedRatio;
        else if( mLinear < -mMaxLin*mSpeedRatio )
            mLinear = -mMaxLin*mSpeedRatio;

        if( mAngular > mMaxAng*mSpeedRatio )
            mAngular = mMaxAng*mSpeedRatio;
        else if( mAngular < -mMaxAng*mSpeedRatio )
            mAngular = -mMaxAng*mSpeedRatio;

        if( fabs(mLinear) < 0.01 )
            mLinear = 0.0;

        if( fabs(mAngular) < 0.01 )
            mAngular = 0.0;

        vel.linear.x = mLinear;
        vel.angular.z = mAngular;
        // <<<<< Saturations

        //if(dirty==true)
        {
            ROS_INFO_STREAM( "Robot speed " << (mLocked?"locked ":"") << "- Linear: " << mLinear << " - Angular: " << mAngular <<  "\n\r" \
                             "[Q quit][1 Max_speed][2 for Max_speed/2][3 Max speed/3][L toggle lock]\r");

            mPubVelControl.publish(vel); // publish "cmd_vel" message
            dirty=false;
        }
    }
    
    // >>>>> Force motor stopping
    geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    vel.angular.z = 0.0;
    
    mPubVelControl.publish(vel); // publish "cmd_vel" message
    // <<<<< Force motor stopping

    endwin();

    ROS_INFO_STREAM("-----------------------------\r");
    ROS_INFO_STREAM("TELEOPERATION STOPPED BY USER\r");
    ROS_INFO_STREAM("-----------------------------\r");

    return;
}



