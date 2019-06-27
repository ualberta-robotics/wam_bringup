/*
    Copyright 2012 Barrett Technology <support@barrett.com>

    This file is part of barrett-ros-pkg.

    This version of barrett-ros-pkg is free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This version of barrett-ros-pkg is distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this version of barrett-ros-pkg.  If not, see
    <http://www.gnu.org/licenses/>.

    Barrett Technology holds all copyrights on barrett-ros-pkg. As the sole
    copyright holder, Barrett reserves the right to release future versions
    of barrett-ros-pkg under a different license.

    File: bhand_node.h
    Refactored by: Laura Petrich 17/06/19
 */

#include "ros/ros.h"
#include <string>
#include <sstream>
#include <exception>
//Boost libraries
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
//BHand Services / Joint Messages
#include "wam_srvs/BHandFingerPos.h"
#include "wam_srvs/BHandGraspPos.h"
#include "wam_srvs/BHandSpreadPos.h"
#include "wam_srvs/BHandFingerVel.h"
#include "wam_srvs/BHandGraspVel.h"
#include "wam_srvs/BHandSpreadVel.h"
#include "wam_srvs/BHandPinchPos.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/JointState.h"

// Publishing Frequency for the BarretHand
static const int PUB_FREQ = 10; 
// Serial port location - can be found with command line $ dmesg | grep tty
const std::string BHAND_PORT = "/dev/ttyS0"; 
/* Ratio from motor encoding the hand returns to radians.
 * Fingers move about 135 degrees max = 2.356 radians
 * Motor encoding goes from 0 to ~17000
 * Original was using 5729.578
 * use: 17000 / 2.1223
*/
const double FINGER_RATIO = 8010.17763;
/* Ratio from motor encoding the hand returns to radians.
 * Fingers move 180 degrees, pi radians
 * Motor encoding goes from 0 to ~3146
 * Original was using 1002.68
 * use: 3146 / 3.1416
*/
const double SPREAD_RATIO = 1001.40056;
/* 0.9 / 2.1223 = 0.424068; Ratio of max finger bent. If
 * FingerOne/KnuckleTwoJoint is closed 50% of the way assume
 * FingerOne/KnuckleThreeJoint is closed that much
*/
const double SECOND_LINK_RATIO = 0.424068;

class BHandNode
{
    public:
        ros::NodeHandle nb_;
        // ros::Rate pub_rate; // Default 100Hz
        std::string bhand_command;
        sensor_msgs::JointState bhand_joint_state;
        ros::Publisher bhand_joint_state_pub;
        //Services to be made available
        ros::ServiceServer hand_initialize_srv;
        ros::ServiceServer hand_open_grsp_srv;
        ros::ServiceServer hand_close_grsp_srv;
        ros::ServiceServer hand_open_sprd_srv;
        ros::ServiceServer hand_close_sprd_srv;
        ros::ServiceServer hand_fngr_pos_srv;
        ros::ServiceServer hand_fngr_vel_srv;
        ros::ServiceServer hand_grsp_pos_srv;
        ros::ServiceServer hand_grsp_vel_srv;
        ros::ServiceServer hand_sprd_pos_srv;
        ros::ServiceServer hand_sprd_vel_srv;
        ros::ServiceServer hand_pinch_pos_srv;
        // Our serial port variables
        boost::asio::serial_port_base::baud_rate baud;
        boost::asio::serial_port_base::character_size char_size;
        boost::asio::serial_port_base::flow_control flow;
        boost::asio::serial_port_base::parity parity;
        boost::asio::serial_port_base::stop_bits stopbits;
        boost::asio::io_service io;
        boost::asio::serial_port port;

        BHandNode() :
            nb_("bhand"), 
            // pub_rate(PUB_FREQ), 
            baud(9600), 
            char_size(8),
            flow(boost::asio::serial_port_base::flow_control::none),
            parity(boost::asio::serial_port_base::parity::none),
            stopbits(boost::asio::serial_port_base::stop_bits::one), 
            port(io, BHAND_PORT) 
            {}
        
        void init();

        std::string read_line();

        void write(std::string buf) {
            //write command out to serial port
            boost::asio::write(port, boost::asio::buffer((buf+"\r").c_str(), buf.size() + 1));
        }

        ~BHandNode() {
        }

        bool handInitialize(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool handOpenGrasp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool handCloseGrasp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool handOpenSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool handCloseSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool handFingerPos(wam_srvs::BHandFingerPos::Request &req, wam_srvs::BHandFingerPos::Response &res);
        bool handGraspPos(wam_srvs::BHandGraspPos::Request &req, wam_srvs::BHandGraspPos::Response &res);
        bool handSpreadPos(wam_srvs::BHandSpreadPos::Request &req, wam_srvs::BHandSpreadPos::Response &res);
        bool handPinchPos(wam_srvs::BHandPinchPos::Request &req, wam_srvs::BHandPinchPos::Response &res);
        bool handFingerVel(wam_srvs::BHandFingerVel::Request &req, wam_srvs::BHandFingerVel::Response &res);
        bool handGraspVel(wam_srvs::BHandGraspVel::Request &req, wam_srvs::BHandGraspVel::Response &res);
        bool handSpreadVel(wam_srvs::BHandSpreadVel::Request &req, wam_srvs::BHandSpreadVel::Response &res);
        void publish();
};
