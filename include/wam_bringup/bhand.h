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

 File: bhand.cpp
 Date: 07/12/18
 Author: Laura Petrich
 */

#include "ros/ros.h"

#include <string>
#include <sstream>

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
const std::string bhand_port = "/dev/ttyS0"; //Serial port location - can be found with command line $ dmesg | grep tty
const double pub_freq = 10; //Hz

/* Ratio from motor encoding the hand returns to radians.
 * Fingers move about 135 degrees max = 2.356 radians
 * Motor encoding goes from 0 to ~17000
 *
 * Original was using 5729.578
 * use: 17000 / 2.1223
*/
const double finger_ratio = 8010.17763;
/* Ratio from motor encoding the hand returns to radians.
 * Fingers move 180 degrees, pi radians
 * Motor encoding goes from 0 to ~3146
 *
 * Original was using 1002.68
 * use: 3146 / 3.1416
*/
const double spread_ratio = 1001.40056;

/* 0.9 / 2.1223 = 0.424068; Ratio of max finger bent. If
 * FingerOne/KnuckleTwoJoint is closed 50% of the way assume
 * FingerOne/KnuckleThreeJoint is closed that much
*/
const double second_link_ratio = 0.424068;

//BH262 Class
class BHand
{
    public:
        ros::NodeHandle nb_;
        ros::Rate pub_rate; // Default 100Hz
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
        ros::ServiceServer pinch_pos_srv;

        // Our serial port variables
        boost::asio::serial_port_base::baud_rate baud;
        boost::asio::serial_port_base::character_size char_size;
        boost::asio::serial_port_base::flow_control flow;
        boost::asio::serial_port_base::parity parity;
        boost::asio::serial_port_base::stop_bits stopbits;
        boost::asio::io_service io;
        boost::asio::serial_port port;

        BHand() :
            nb_("bhand"), pub_rate(pub_freq), baud(9600), char_size(8),
            flow(boost::asio::serial_port_base::flow_control::none),
            parity(boost::asio::serial_port_base::parity::none),
            stopbits(boost::asio::serial_port_base::stop_bits::one), port(io, bhand_port) { }
        void
            init();

        std::string read_line();

        void write(std::string buf) {
            //write command out to serial port
            boost::asio::write(port, boost::asio::buffer((buf+"\r").c_str(), buf.size() + 1));
        }

        ~BHand() {}

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

void BHand::init() {
    ROS_INFO("Openning BHand on serial port %s", bhand_port.c_str());

    // Set the settings
    port.set_option(baud);
    port.set_option(char_size);
    port.set_option(flow);
    port.set_option(parity);
    port.set_option(stopbits);
    //Advertise the following services only if there is a BarrettHand present
    hand_initialize_srv = nb_.advertiseService("initialize",   &BHand::handInitialize, this);
    hand_open_grsp_srv  = nb_.advertiseService("open_grasp",   &BHand::handOpenGrasp, this);
    hand_close_grsp_srv = nb_.advertiseService("close_grasp",  &BHand::handCloseGrasp, this);
    hand_open_sprd_srv  = nb_.advertiseService("open_spread",  &BHand::handOpenSpread, this);
    hand_close_sprd_srv = nb_.advertiseService("close_spread", &BHand::handCloseSpread, this);
    hand_fngr_pos_srv   = nb_.advertiseService("finger_pos",   &BHand::handFingerPos, this);
    hand_grsp_pos_srv   = nb_.advertiseService("grasp_pos",    &BHand::handGraspPos, this);
    hand_sprd_pos_srv   = nb_.advertiseService("spread_pos",   &BHand::handSpreadPos, this);
    pinch_pos_srv       = nb_.advertiseService("pinch_pos",    &BHand::handPinchPos, this);
    hand_fngr_vel_srv   = nb_.advertiseService("finger_vel",   &BHand::handFingerVel, this);
    hand_grsp_vel_srv   = nb_.advertiseService("grasp_vel",    &BHand::handGraspVel, this);
    hand_sprd_vel_srv   = nb_.advertiseService("spread_vel",   &BHand::handSpreadVel, this);
    /* Setup Hand joint state message
     * Newer hands are already handled by the wam_node.
    */
    //const char* bhand_jnts[] = {"inner_f1", "inner_f2", "inner_f3", "spread"};//, "outer_f1", "outer_f2", "outer_f3"};
    const char* bhand_jnts[] = {"wam/BHand/FingerOne/KnuckleTwoJoint",
                                "wam/BHand/FingerTwo/KnuckleTwoJoint",
                                "wam/BHand/FingerThree/KnuckleTwoJoint",
                                "wam/BHand/FingerOne/KnuckleOneJoint",
                                "wam/BHand/FingerTwo/KnuckleOneJoint",
                                "wam/BHand/FingerOne/KnuckleThreeJoint",
                                "wam/BHand/FingerTwo/KnuckleThreeJoint",
                                "wam/BHand/FingerThree/KnuckleThreeJoint"};
    std::vector < std::string > bhand_joints(bhand_jnts, bhand_jnts + 8);
    bhand_joint_state.name.resize(8);
    bhand_joint_state.name = bhand_joints;
    bhand_joint_state.position.resize(8);
    bhand_joint_state_pub = nb_.advertise < sensor_msgs::JointState > ("joint_states", 1);

    ros::AsyncSpinner spinner(0);
    spinner.start();
}

bool BHand::handInitialize(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    ROS_INFO("BHand Initialize Request");
    write("HI");
    return true;
}

bool BHand::handOpenGrasp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    ROS_INFO("BHand Open Request");
    write("GO");
    return true;
}

bool BHand::handCloseGrasp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    ROS_INFO("BHand Close Request");
    write("GC");
    return true;
}

bool BHand::handOpenSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    ROS_INFO("Spread Open Request");
    write("SO");
    return true;
}

bool BHand::handCloseSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    ROS_INFO("Spread Close Request");
    write("SC");
    return true;
}

bool BHand::handFingerPos(wam_srvs::BHandFingerPos::Request &req, wam_srvs::BHandFingerPos::Response &res) {
    ROS_INFO("Finger Position Request");
    bhand_command = "1M " + boost::lexical_cast<std::string>(ceil(req.radians[0] * finger_ratio));
    write(bhand_command);
    bhand_command = "2M " + boost::lexical_cast<std::string>(ceil(req.radians[1] * finger_ratio));
    write(bhand_command);
    bhand_command = "3M " + boost::lexical_cast<std::string>(ceil(req.radians[2] * finger_ratio));
    write(bhand_command);
    return true;
}

bool BHand::handGraspPos(wam_srvs::BHandGraspPos::Request &req, wam_srvs::BHandGraspPos::Response &res) {
    ROS_INFO("Grasp Position Request");
    bhand_command = "GM " + boost::lexical_cast<std::string>(ceil(req.radians * finger_ratio));
    write(bhand_command);
    return true;
}

bool BHand::handSpreadPos(wam_srvs::BHandSpreadPos::Request &req, wam_srvs::BHandSpreadPos::Response &res) {
    ROS_INFO("Spread Position Request");
    bhand_command = "SM " + boost::lexical_cast<std::string>(ceil(req.radians * spread_ratio));
    write(bhand_command);
    return true;
}

bool BHand::handPinchPos(wam_srvs::BHandPinchPos::Request &req, wam_srvs::BHandPinchPos::Response &res) {
    ROS_INFO("Pinch Position Request");
    bhand_command = "12M " + boost::lexical_cast<std::string>(ceil(req.radians * finger_ratio));
    write(bhand_command);
    return true;
}

/* If finger velocity value is positive then closeing velocities are set for all fingers.
 * If all velocities are negative then finger opening velocities are set.
 *
 * Default is 100 for all finger opening and closing
 */
bool BHand::handFingerVel(wam_srvs::BHandFingerVel::Request &req, wam_srvs::BHandFingerVel::Response &res) {
    ROS_INFO("Finger Velocities Request");

    if (req.velocity[0] > 0 || req.velocity[1] > 0 || req.velocity[2] > 0) {
        bhand_command = "1FSET MCV " + boost::lexical_cast<std::string>(ceil(abs(req.velocity[0])));
        write(bhand_command);
        bhand_command = "2FSET MCV " + boost::lexical_cast<std::string>(ceil(abs(req.velocity[1])));
        write(bhand_command);
        bhand_command = "3FSET MCV " + boost::lexical_cast<std::string>(ceil(abs(req.velocity[2])));
    }
    else {
        bhand_command = "1FSET MOV " + boost::lexical_cast<std::string>(ceil(abs(req.velocity[0])));
        write(bhand_command);
        bhand_command = "2FSET MOV " + boost::lexical_cast<std::string>(ceil(abs(req.velocity[1])));
        write(bhand_command);
        bhand_command = "3FSET MOV " + boost::lexical_cast<std::string>(ceil(abs(req.velocity[2])));
    }
    return true;
}

/* Set grasp close velocity if requested velocity is positive, opening velocity if requested velocity
 * is negative.
 *
 * Default grasp velocities are 100 for MCV and MOV
 */
bool BHand::handGraspVel(wam_srvs::BHandGraspVel::Request &req, wam_srvs::BHandGraspVel::Response &res) {
    if (req.velocity > 0.0) {
        bhand_command = "GFSET MCV " + boost::lexical_cast<std::string>(ceil(abs(req.velocity)));
        write(bhand_command);
        bhand_command = "GC";
    }
    else {
        bhand_command = "GFSET MOV " + boost::lexical_cast<std::string>(ceil(abs(req.velocity)));
        write(bhand_command);
        bhand_command = "GO";
    }
    return true;
}

/* Set spread close velocity if requested velocity is positive, opening velocity if requested velocity
 * is negative.
 *
 * Default for MCV and MOV for spread is 60
 */
bool BHand::handSpreadVel(wam_srvs::BHandSpreadVel::Request &req, wam_srvs::BHandSpreadVel::Response &res) {
    if (req.velocity > 0.0) {
        bhand_command = "4FSET MCV " + boost::lexical_cast<std::string>(ceil(abs(req.velocity)));
        write(bhand_command);
    }
    else {
        bhand_command = "4FSET MOV " + boost::lexical_cast<std::string>(ceil(abs(req.velocity)));
        write(bhand_command);
    }
    return true;
}

std::string BHand::read_line() {
    char c;
    std::string result;
    for(;;)
    {
        boost::asio::read(port, boost::asio::buffer(&c, 1));
        switch(c)
        {
            case '\r':
                break;
            case '\n':
                return result;
            default:
                result+=c;
        }
    }
}

void BHand::publish() {
    ::tcflush(port.lowest_layer().native(), TCIFLUSH);
    write("FGET P");
    //Working on reading these positions and converting them to joint_state messages
    std::string result;

    /*
     * TODO: Should use locks and make this process robust.
     *       Consider using real time mode on the hand.
     * NOTE: Publishing of finger state will be interrupted while the hand moves
     */
    result = read_line();
    //ROS_INFO_STREAM("Cleaning read: " << result);
    std::size_t found = result.find("FGET P");
    if (found!=std::string::npos) {
        result = read_line();
        //ROS_INFO_STREAM("Publish read: " << result);
        std::stringstream stream(result);

        int i;
        for (i = 0; i < 4; ++i) {
            if (!(stream >> bhand_joint_state.position[i]))
                break;
            //ROS_INFO_STREAM("Found integer: " << bhand_joint_state.position[i]);
        }

        if (i == 4) {
            bhand_joint_state.position[0] /= finger_ratio;
            bhand_joint_state.position[1] /= finger_ratio;
            bhand_joint_state.position[2] /= finger_ratio;
            bhand_joint_state.position[3] /= spread_ratio;
            bhand_joint_state.position[4]  = bhand_joint_state.position[3];
            bhand_joint_state.position[5] = bhand_joint_state.position[0] * second_link_ratio;
            bhand_joint_state.position[6] = bhand_joint_state.position[1] * second_link_ratio;
            bhand_joint_state.position[7] = bhand_joint_state.position[2] * second_link_ratio;
            bhand_joint_state_pub.publish(bhand_joint_state);
        }
    }
}
