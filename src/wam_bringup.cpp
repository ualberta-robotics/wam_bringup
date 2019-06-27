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

File: wam_bringup.cpp
Date: 5 June, 2012
Author: Kyle Maroney
Refactored by: Laura Petrich 01/05/18
*/

#include "wam_bringup/wam_bringup.h"

// Templated Initialization Function
template<size_t DOF>
void WamBringup<DOF>::init(ProductManager& pm)
{
    // ros::NodeHandle n_("wam"); // WAM specific nodehandle
    // ros::NodeHandle nb_("bhand"); // BarrettHand specific nodehandle
    mm = NULL;
    vhp = NULL;
    //Setting up real-time command timeouts and initial values
    rt_msg_timeout.fromSec(0.3); //rt_status will be determined false if rt message is not received in specified time
    cart_vel_mag = SPEED; //Setting default cartesian velocity magnitude to SPEED
    ortn_vel_mag = SPEED;
    hand_available = false;
    cart_vel_status = false; //Bool for determining cartesian velocity real-time state
    ortn_vel_status = false; //Bool for determining orientation velocity real-time state
    new_rt_cmd = false; //Bool for determining if a new real-time message was received
    jnt_vel_status = false;
    jnt_pos_status = false;
    jnt_hand_tool_status = false;
    cart_pos_status = false;
    ortn_pos_status = false;
    fn = 0.0;
    systems_connected = false;

    // last_cart_vel_msg_time = ros::Time::now();
    // SETUP BHAND
    ROS_INFO("Openning BHand on serial port %s", BHAND_PORT.c_str());
    // Set the settings
    port.set_option(baud);
    port.set_option(char_size);
    port.set_option(flow);
    port.set_option(parity);
    port.set_option(stopbits);
    //Advertise the following services only if there is a BarrettHand present
    hand_initialize_srv = nb_.advertiseService("initialize", &WamBringup::handInitialize, this);
    hand_open_grsp_srv = nb_.advertiseService("open_grasp", &WamBringup::handOpenGrasp, this);
    hand_close_grsp_srv = nb_.advertiseService("close_grasp", &WamBringup::handCloseGrasp, this);
    hand_open_sprd_srv = nb_.advertiseService("open_spread", &WamBringup::handOpenSpread, this);
    hand_close_sprd_srv = nb_.advertiseService("close_spread", &WamBringup::handCloseSpread, this);
    hand_fngr_pos_srv = nb_.advertiseService("finger_pos", &WamBringup::handFingerPos, this);
    hand_grsp_pos_srv = nb_.advertiseService("grasp_pos", &WamBringup::handGraspPos, this);
    hand_sprd_pos_srv = nb_.advertiseService("spread_pos", &WamBringup::handSpreadPos, this);
    hand_pinch_pos_srv = nb_.advertiseService("pinch_pos", &WamBringup::handPinchPos, this);
    hand_fngr_vel_srv = nb_.advertiseService("finger_vel", &WamBringup::handFingerVel, this);
    hand_grsp_vel_srv = nb_.advertiseService("grasp_vel", &WamBringup::handGraspVel, this);
    hand_sprd_vel_srv = nb_.advertiseService("spread_vel", &WamBringup::handSpreadVel, this);
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
    // END BHAND SETUP
    mypm = &pm;
    pm.getExecutionManager()->startManaging(ramp); //starting ramp manager
    ROS_INFO("%zu-DOF WAM", DOF);
    jp_home = wam.getJointPositions();
    wam.gravityCompensate(true); // Turning on Gravity Compenstation by Default when starting the WAM Node
    //Setting up WAM joint state publisher
    const char* wam_jnts[] = {  "wam/YawJoint",
                                "wam/ShoulderPitchJoint",
                                "wam/ShoulderYawJoint",
                                "wam/ElbowJoint",
                                "wam/UpperWristYawJoint",
                                "wam/UpperWristPitchJoint",
                                "wam/LowerWristYawJoint"
                             };
    std::vector < std::string > wam_joints(wam_jnts, wam_jnts + 7);
    wam_joint_state.name = wam_joints;
    wam_joint_state.name.resize(DOF);
    wam_joint_state.position.resize(DOF);
    wam_joint_state.velocity.resize(DOF);
    wam_joint_state.effort.resize(DOF);
    wam_jacobian_mn.data.resize(DOF*6);
    //Publishing the following rostopics
    wam_joint_state_pub = n_.advertise < sensor_msgs::JointState > ("joint_states", 1);
    wam_pose_pub = n_.advertise < geometry_msgs::PoseStamped > ("pose", 1);
    wam_jacobian_mn_pub =n_.advertise < wam_msgs::MatrixMN > ("jacobian",1);
    //Subscribing to the following rostopics
    cart_vel_sub = n_.subscribe("cart_vel_cmd", 1, &WamBringup::cartVelCB, this);
    ortn_vel_sub = n_.subscribe("ortn_vel_cmd", 1, &WamBringup::ortnVelCB, this);
    jnt_vel_sub = n_.subscribe("jnt_vel_cmd", 1, &WamBringup::jntVelCB, this);
    jnt_pos_sub = n_.subscribe("jnt_pos_cmd", 1, &WamBringup::jntPosCB, this);
    jnt_hand_tool_sub = n_.subscribe("jnt_hand_tool_cmd", 1, &WamBringup::jntHandToolCB, this);
    cart_pos_sub = n_.subscribe("cart_pos_cmd", 1, &WamBringup::cartPosCB, this);
    vs_error = n_.subscribe("v_err", 1, &WamBringup::vsErrCB, this);
    //Advertising the following rosservices
    gravity_srv = n_.advertiseService("gravity_comp", &WamBringup::gravity, this);
    go_home_srv = n_.advertiseService("go_home", &WamBringup::goHome, this);

    haptic_sphere_srv = n_.advertiseService("haptic_sphere", &WamBringup::hapticSphere, this);
    // LP control experiments
    jp_pid_srv = n_.advertiseService("jp_pid_control", &WamBringup::jpPIDControl,this);
    jv_pid_srv = n_.advertiseService("jv_pid_control", &WamBringup::jvPIDControl,this);
    tp_pid_srv = n_.advertiseService("tp_pid_control", &WamBringup::tpPIDControl,this);
    force_torque_tool_time_srv = n_.advertiseService("force_torque_tool_time", &WamBringup::forceTorqueToolTime,this);
    force_torque_base_time_srv = n_.advertiseService("force_torque_base_time", &WamBringup::forceTorqueBaseTime,this);

    hold_jpos_srv = n_.advertiseService("hold_joint_pos", &WamBringup::holdJPos, this);
    hold_cpos_srv = n_.advertiseService("hold_cart_pos", &WamBringup::holdCPos, this);
    hold_ortn_srv = n_.advertiseService("hold_ortn", &WamBringup::holdOrtn, this);
    hold_ortn2_srv = n_.advertiseService("hold_ortn2", &WamBringup::holdOrtn2, this);
    joint_move_srv = n_.advertiseService("joint_move", &WamBringup::jointMove, this);
    joint_move_block_srv = n_.advertiseService("joint_move_block", &WamBringup::jointMoveBlock, this);
    pose_move_srv = n_.advertiseService("pose_move", &WamBringup::poseMove, this);
    cart_move_srv = n_.advertiseService("cart_move", &WamBringup::cartMove, this);
    cart_vel_srv = n_.advertiseService("cart_vel", &WamBringup::cartVel, this);
    ortn_move_srv = n_.advertiseService("ortn_move", &WamBringup::ortnMove, this);
    ortn_split_move_srv = n_.advertiseService("ortn_split_move", &WamBringup::ortnSplitMove, this);
    force_torque_base_srv = n_.advertiseService("force_torque_base", &WamBringup::forceTorqueBase, this);
    force_torque_tool_srv = n_.advertiseService("force_torque_tool", &WamBringup::forceTorqueTool, this);
    teach_srv = n_.advertiseService("teach_motion", &WamBringup::teachMotion, this);
    play_srv = n_.advertiseService("play_motion", &WamBringup::playMotion, this);
    link_arm_srv = n_.advertiseService("link_arm", &WamBringup::linkArm, this);
    unlink_arm_srv = n_.advertiseService("unlink_arm", &WamBringup::unLinkArm, this);
    start_visual_fix = n_.advertiseService("start_visual_fix", &WamBringup::startVisualFix, this);
    stop_visual_fix = n_.advertiseService("stop_visual_fix", &WamBringup::stopVisualFix, this);
    follow_path_srv = n_.advertiseService("follow_path", &WamBringup::followPath,this);
    ROS_INFO("wam services now advertised");
}

// BHAND FUNCTIONS //////////////////////////////////////////////////////////////////////////////////////////
template<size_t DOF>
bool WamBringup<DOF>::handInitialize(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("BHand Initialize Request");
    write("HI");
    return true;
}
//Function to open the BarrettHand Grasp
template<size_t DOF>
bool WamBringup<DOF>::handOpenGrasp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("BHand Open Request");
    write("GO");
    return true;
}

//Function to close the BarrettHand Grasp
template<size_t DOF>
bool WamBringup<DOF>::handCloseGrasp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("BHand Close Request");
    write("GC");
    return true;
}

//Function to open the BarrettHand Spread
template<size_t DOF>
bool WamBringup<DOF>::handOpenSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Spread Open Request");
    write("SO");
    return true;
}

//Function to close the BarrettHand Spread
template<size_t DOF>
bool WamBringup<DOF>::handCloseSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Spread Close Request");
    write("SC");
    return true;
}

//Function to control a BarrettHand Finger Position
template<size_t DOF>
bool WamBringup<DOF>::handFingerPos(wam_srvs::BHandFingerPos::Request &req, wam_srvs::BHandFingerPos::Response &res)
{
    ROS_INFO("Finger Position Request");
    bhand_command = "1M " + boost::lexical_cast<std::string>(ceil(req.radians[0] * FINGER_RATIO));
    write(bhand_command);
    bhand_command = "2M " + boost::lexical_cast<std::string>(ceil(req.radians[1] * FINGER_RATIO));
    write(bhand_command);
    bhand_command = "3M " + boost::lexical_cast<std::string>(ceil(req.radians[2] * FINGER_RATIO));
    write(bhand_command);
    return true;
}

//Function to control the BarrettHand Grasp Position
template<size_t DOF>
bool WamBringup<DOF>::handGraspPos(wam_srvs::BHandGraspPos::Request &req, wam_srvs::BHandGraspPos::Response &res)
{
    ROS_INFO("Grasp Position Request");
    bhand_command = "GM " + boost::lexical_cast<std::string>(ceil(req.radians * FINGER_RATIO));
    write(bhand_command);
    return true;
}

//Function to control the BarrettHand Spread Position
template<size_t DOF>
bool WamBringup<DOF>::handSpreadPos(wam_srvs::BHandSpreadPos::Request &req, wam_srvs::BHandSpreadPos::Response &res)
{
    ROS_INFO("Spread Position Request");
    bhand_command = "SM " + boost::lexical_cast<std::string>(ceil(req.radians * SPREAD_RATIO));
    write(bhand_command);
    return true;
}
//Function to control the BarrettHand Pinch Position
template<size_t DOF>
bool WamBringup<DOF>::handPinchPos(wam_srvs::BHandPinchPos::Request &req, wam_srvs::BHandPinchPos::Response &res)
{
    ROS_INFO("Pinch Position Request");
    bhand_command = "12M " + boost::lexical_cast<std::string>(ceil(req.radians * FINGER_RATIO));
    write(bhand_command);
    return true;
}
//Function to control a BarrettHand Finger Velocity
/* If finger velocity value is positive then closeing velocities are set for all fingers.
* If all velocities are negative then finger opening velocities are set.
*
* Default is 100 for all finger opening and closing
*/
template<size_t DOF>
bool WamBringup<DOF>::handFingerVel(wam_srvs::BHandFingerVel::Request &req, wam_srvs::BHandFingerVel::Response &res)
{
    ROS_INFO("Finger Velocities Request");
    bhand_command = "1FSET MCV " + boost::lexical_cast<std::string>(ceil(abs(req.velocity[0])));
    write(bhand_command);
    bhand_command = "2FSET MCV " + boost::lexical_cast<std::string>(ceil(abs(req.velocity[1])));
    write(bhand_command);
    bhand_command = "3FSET MCV " + boost::lexical_cast<std::string>(ceil(abs(req.velocity[2])));
    write(bhand_command);
    bhand_command = "1FSET MOV " + boost::lexical_cast<std::string>(ceil(abs(req.velocity[0])));
    write(bhand_command);
    bhand_command = "2FSET MOV " + boost::lexical_cast<std::string>(ceil(abs(req.velocity[1])));
    write(bhand_command);
    bhand_command = "3FSET MOV " + boost::lexical_cast<std::string>(ceil(abs(req.velocity[2])));
    write(bhand_command);
    return true;
}

//Function to control a BarrettHand Grasp Velocity
/*
* Default grasp velocities are 100 for MCV and MOV
*/
template<size_t DOF>
bool WamBringup<DOF>::handGraspVel(wam_srvs::BHandGraspVel::Request &req, wam_srvs::BHandGraspVel::Response &res)
{
    bhand_command = "GFSET MCV " + boost::lexical_cast<std::string>(ceil(abs(req.velocity)));
    write(bhand_command);
    bhand_command = "GFSET MOV " + boost::lexical_cast<std::string>(ceil(abs(req.velocity)));
    write(bhand_command);
    return true;
}

//Function to control a BarrettHand Spread Velocity
/*
* Default for MCV and MOV for spread is 60
*/
template<size_t DOF>
bool WamBringup<DOF>::handSpreadVel(wam_srvs::BHandSpreadVel::Request &req, wam_srvs::BHandSpreadVel::Response &res)
{
    bhand_command = "SFSET MCV " + boost::lexical_cast<std::string>(ceil(abs(req.velocity)));
    write(bhand_command);
    bhand_command = "SFSET MOV " + boost::lexical_cast<std::string>(ceil(abs(req.velocity)));
    write(bhand_command);
    return true;
}

template<size_t DOF>
std::string WamBringup<DOF>::read_line()
{
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

// WAM ARM FUNCTIONS //////////////////////////////////////////////////////////////////////////////////////////
/*
    low level PID Controllers available
    PIDController<jp_type, jt_type> jpController;
    PIDController<jv_type, jt_type> jvController1;
    PIDController<cp_type, cf_type> tpController;
*/
template<size_t DOF>
bool WamBringup<DOF>::jpPIDControl(wam_srvs::JP_PID::Request &req, wam_srvs::JP_PID::Response &res)
{
    // request in form [idx, kp value for joint[idx - 1], idx, kp value for joint[idx - 1],..... ]
    jp_type kp, kd, ki;
    double x;
    ROS_INFO("jpPIDControl Request");
    kp = wam.jpController.getKp();
    kd = wam.jpController.getKd();
    ki = wam.jpController.getKi();
    // check kp
    std::cout << "\tCurrent jpController kp values: " << kp << std::endl;
    if (req.kp.size() == 0) {
        std::cout << "\t\tKeeping current kp values" << std::endl;
    } else {
        for (size_t i = 0; i < req.kp.size(); i+=2) {
            x = req.kp[i];
            if (1 <= x && x < 8) {
                std::cout << "\t\tChanging joint " << x-1 << " kp value from " << kp[x-1] << " to " << req.kp[i+1] << std::endl;
                kp[x-1] = req.kp[i+1];
            } else {
                std::cout << "\t\tERROR: joint selected must be in range [1,7], you chose: " << x << std::endl;
            }
        }
        std::cout << "\tSetting jpController kp values to: " << kp << std::endl;
    }
    // check kd
    std::cout << "\n\tCurrent jpController kd values: " << kd << std::endl;
    if (req.kd.size() == 0) {
        std::cout << "\t\tKeeping current kd values" << std::endl;
    } else {
        for (size_t i = 0; i < req.kd.size(); i+=2) {
            x = req.kd[i];
            if (1 <= x && x < 8) {
                std::cout << "\t\tChanging joint " << x-1 << " kd value from " << kd[x-1] << " to " << req.kd[i+1] << std::endl;
                kd[x-1] = req.kd[i+1];
            } else {
                std::cout << "\t\tERROR: joint selected must be in range [1,7], you chose: " << x << std::endl;
            }
        }
        std::cout << "\t\tSetting jpController kd values to: " << kd << std::endl;
    }
    // // check ki
    std::cout << "\n\tCurrent jpController ki values: " << ki << std::endl;
    if (req.ki.size() == 0) {
        std::cout << "\t\tKeeping current ki values" << std::endl;
    } else {
        for (size_t i = 0; i < req.ki.size(); i+=2) {
            x = req.ki[i];
            if (1 <= x && x < 8) {
                std::cout << "\t\tChanging joint " << x-1 << " ki value from " << ki[x-1] << " to " << req.ki[i+1] << std::endl;
                ki[x-1] = req.ki[i+1];
            } else {
                std::cout << "\t\tERROR: joint selected must be in range [1,7], you chose: " << x << std::endl;
            }
        }
        std::cout << "\tSetting jpController ki values to: " << ki << std::endl;
    }
    return true;
}

template<size_t DOF>
bool WamBringup<DOF>::jvPIDControl(wam_srvs::JV_PID::Request &req, wam_srvs::JV_PID::Response &res)
{
    // low level PID Controllers available
    // PIDController<jp_type, jt_type> jpController;
    // PIDController<jv_type, jt_type> jvController1;
    // PIDController<cp_type, cf_type> tpController;
    // double ki = req.ki;
    // double kp = req.kp;
    // double kd = req.kd;
    ROS_INFO("jvPIDControl Request");
    std::cout << "jpController values: " << wam.jpController.getKp() << wam.jpController.getKd() << wam.jpController.getKi() << std::endl;
    std::cout << "jvController1 values: " << wam.jvController1.getKp() << wam.jvController1.getKd() << wam.jvController1.getKi() << std::endl;
    std::cout << "tpController values: " << wam.tpController.getKp() << wam.tpController.getKd() << wam.tpController.getKi() << std::endl;
    return true;
}

template<size_t DOF>
bool WamBringup<DOF>::tpPIDControl(wam_srvs::TP_PID::Request &req, wam_srvs::TP_PID::Response &res)
{
    // low level PID Controllers available
    // PIDController<jp_type, jt_type> jpController;
    // PIDController<jv_type, jt_type> jvController1;
    // PIDController<cp_type, cf_type> tpController;
    // double ki = req.ki;
    // double kp = req.kp;
    // double kd = req.kd;
    ROS_INFO("tpPIDControl Request");
    std::cout << "jpController values: " << wam.jpController.getKp() << wam.jpController.getKd() << wam.jpController.getKi() << std::endl;
    std::cout << "jvController1 values: " << wam.jvController1.getKp() << wam.jvController1.getKd() << wam.jvController1.getKi() << std::endl;
    std::cout << "tpController values: " << wam.tpController.getKp() << wam.tpController.getKd() << wam.tpController.getKi() << std::endl;
    return true;
}

// haptic sphere function, creates a sphere with requested radius, kp and kd values
template<size_t DOF>
bool WamBringup<DOF>::hapticSphere(wam_srvs::HapticSphere::Request &req, wam_srvs::HapticSphere::Response &res)
{
    double radius = req.radius;
    double kp = req.kp;
    double kd = req.kd;
    bool trigger = req.trigger;
    ROS_INFO("Haptic Sphere Request -- radius: %f kp: %f kd: %f trigger: %s", radius, kp, kd, (trigger) ? "true" : "false");
    mypm->getSafetyModule()->setVelocityLimit(1.25);
    mypm->getSafetyModule()->waitForMode(SafetyModule::ACTIVE);
    wam.gravityCompensate();
    if (trigger){
        std::cout << "trigger true" << std::endl;
        exit_haptic_sphere = false;
        cp_type center(wam.getToolPosition());
        haptic_ball.setCenter(center);
        haptic_ball.setRadius(radius);
        pid_control_ball.setKp(kp);
        pid_control_ball.setKd(kd);

        systems::forceConnect(wam.toolPosition.output, haptic_ball.input);
        systems::forceConnect(wam.kinematicsBase.kinOutput, tf2jt_haptic_ball.kinInput);
        systems::forceConnect(haptic_ball.directionOutput, tg_haptic_ball.getInput<0>());
        systems::forceConnect(haptic_ball.depthOutput, pid_control_ball.referenceInput);
        systems::forceConnect(zero.output, pid_control_ball.feedbackInput);
        systems::forceConnect(pid_control_ball.controlOutput, tg_haptic_ball.getInput<1>());

        systems::forceConnect(tg_haptic_ball.output, mult_haptic_ball.input);
        systems::forceConnect(mult_haptic_ball.output, tf2jt_haptic_ball.input);
        systems::forceConnect(tf2jt_haptic_ball.output, jtSat.input);
        systems::forceConnect(jtSat.output, wam.input);
        wam.idle();

    } else {
        systems::disconnect(wam.input);
        wam.idle();
    }
    return true;
    //
}

// gravity_comp service callback
template<size_t DOF>
bool WamBringup<DOF>::gravity(wam_srvs::GravityComp::Request &req, wam_srvs::GravityComp::Response &res)
{
    wam.gravityCompensate(req.gravity);
    ROS_INFO("Gravity Compensation Request: %s", (req.gravity) ? "true" : "false");
    return true;
}

// goHome Function for sending the WAM safely back to its home starting position.
template<size_t DOF>
bool WamBringup<DOF>::goHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Returning to Home Position");
    // open grasp and close spread before sending home
    write("GO");
    write("SC");
    for (size_t i = 0; i < DOF; i++)
    {
        jp_cmd[i] = 0.0;
    }
    wam.moveTo(jp_cmd, true);
    jp_home[3] -= 0.3;
    wam.moveTo(jp_home, true);
    jp_home[3] += 0.3;
    wam.moveTo(jp_home, true);
    return true;
}

//Function to hold WAM Joint Positions
template<size_t DOF>
bool WamBringup<DOF>::holdJPos(wam_srvs::Hold::Request &req, wam_srvs::Hold::Response &res)
{
    ROS_INFO("Joint Position Hold request: %s", (req.hold) ? "true" : "false");
    if (req.hold)
    {
        wam.moveTo(wam.getJointPositions());
    }
    else
    {
        wam.idle();
    }
    return true;
}

//Function to hold WAM end effector Cartesian Position
template<size_t DOF>
bool WamBringup<DOF>::holdCPos(wam_srvs::Hold::Request &req, wam_srvs::Hold::Response &res)
{
    ROS_INFO("Cartesian Position Hold request: %s", (req.hold) ? "true" : "false");
    // if (req.hold)
    // {
    //  wam.moveTo(wam.getToolPosition());
    // }
    // else
    // {
    //  wam.idle();
    // }
    return true;
}

//Function to hold WAM end effector Orientation
template<size_t DOF>
bool WamBringup<DOF>::holdOrtn(wam_srvs::Hold::Request &req, wam_srvs::Hold::Response &res)
{
    ROS_INFO("Orientation Hold request: %s", (req.hold) ? "true" : "false");
    if (req.hold)
    {
        orientationSetPoint.setValue(wam.getToolOrientation());
        wam.trackReferenceSignal(orientationSetPoint.output);
    }
    else
    {
        wam.idle();
    }
    return true;
}

//Function to hold WAM end effector Orientation 2
template<size_t DOF>
bool WamBringup<DOF>::holdOrtn2(wam_srvs::Hold::Request &req, wam_srvs::Hold::Response &res)
{
    ROS_INFO("Orientation Hold request: %s", (req.hold) ? "true" : "false");
    if (req.hold)
    {
        orientationSetPoint.setValue(wam.getToolOrientation());
        //    wam.trackReferenceSignal(orientationSetPoint.output); //CP
        toolOrntController.setKp(5.0);
        toolOrntController.setKd(0.05);
        //Tool Orientation Controller
        systems::forceConnect(wam.kinematicsBase.kinOutput,toolOrntController.kinInput);
        systems::forceConnect(wam.toolOrientation.output,toolOrntController.feedbackInput);
        systems::forceConnect(orientationSetPoint.output,toolOrntController.referenceInput);
        systems::forceConnect(toolOrntController.controlOutput,tt2jtOrnController.input);
        systems::forceConnect(wam.kinematicsBase.kinOutput,tt2jtOrnController.kinInput);
        systems::forceConnect(tt2jtOrnController.output, jtSat.input);
        systems::forceConnect(jtSat.output,wam.input);
        /////////////////////////////////////////////////
    }
    else
    {
        systems::disconnect(wam.input);
        wam.idle();
    }
    return true;
}

//Function to command a joint space move to the WAM
template<size_t DOF>
bool WamBringup<DOF>::jointMove(wam_srvs::JointMove::Request &req, wam_srvs::JointMove::Response &res)
{
    if (req.joints.size() != DOF)
    {
        ROS_INFO("Request Failed: %zu-DOF request received, must be %zu-DOF", req.joints.size(), DOF);
        return false;
    }
    ROS_INFO("Moving Robot to Commanded Joint Pose");
    for (size_t i = 0; i < DOF; i++)
    {
        jp_cmd[i] = req.joints[i];
    }
    // wam.moveTo(jp_cmd, false);
    wam.moveTo(jp_cmd, true); // set second argument to true if you want to complete movement before returning true
    return true;
}
//boop
//Function to command a joint space move to the WAM with blocking specified
template<size_t DOF>
bool WamBringup<DOF>::jointMoveBlock(wam_srvs::JointMoveBlock::Request &req, wam_srvs::JointMoveBlock::Response &res)
{
    if (req.joints.size() != DOF)
    {
        ROS_INFO("Request Failed: %zu-DOF request received, must be %zu-DOF", req.joints.size(), DOF);
        return false;
    }
    ROS_INFO("Moving Robot to Commanded Joint Pose");
    for (size_t i = 0; i < DOF; i++)
    {
        jp_cmd[i] = req.joints[i];
    }
    bool b = req.blocking;
    wam.moveTo(jp_cmd, b);
    return true;
}

//Function to command a pose move to the WAM
template<size_t DOF>
bool WamBringup<DOF>::poseMove(wam_srvs::PoseMove::Request &req, wam_srvs::PoseMove::Response &res)
{
 //     std::cout<< " move robot to a desire pose"<<std::endl;
    // waitForEnter();
    // pose_cmd=wam.getToolPose();
    // ROS_INFO("Tool Pose Store");
    // ROS_INFO("Move robot to an arbitrary pose and press enter.");
    // waitForEnter();
    // boost::tuples::get<0>(pose_cmd)=cp_cmd;
    // boost::tuples::get<1>(pose_cmd)=ortn_cmd;
    // std::cout << "The captured position is:(x,y,z)=("<<cp_cmd[0]<<","<<cp_cmd[1]<<","<<cp_cmd[2]<<")" << std::endl;
    // std::cout << "The captured quaternion is:(x,y,z,w)=("<<ortn_cmd.x()<<","<<ortn_cmd.y()<<","<<ortn_cmd.z()<<","<<ortn_cmd.w()<<")" << std::endl;

    cp_cmd[0] = req.pose.position.x;
    cp_cmd[1] = req.pose.position.y;
    cp_cmd[2] = req.pose.position.z;
    std::cout << "The request position is:(x,y,z)=("<<cp_cmd[0]<<","<<cp_cmd[1]<<","<<cp_cmd[2]<<")" << std::endl;
    ortn_cmd.x() = req.pose.orientation.x;
    ortn_cmd.y() = req.pose.orientation.y;
    ortn_cmd.z() = req.pose.orientation.z;
    ortn_cmd.w() = req.pose.orientation.w;
    std::cout << "The request quaternion is:(x,y,z,w)=("<<ortn_cmd.x()<<","<<ortn_cmd.y()<<","<<ortn_cmd.z()<<","<<ortn_cmd.w()<<")" << std::endl;
    ortn_cmd.normalize();
    std::cout << "The new normalized quaternion is:(x,y,z,w)=("<<ortn_cmd.x()<<","<<ortn_cmd.y()<<","<<ortn_cmd.z()<<","<<ortn_cmd.w()<<")" << std::endl;
    pose_cmd = boost::make_tuple(cp_cmd, ortn_cmd);
    std::cout<< "Press enter to move "<<std::endl;
    waitForEnter();
    // wam.moveTo(pose_cmd, true);

    // pose_cmd = boost::make_tuple(cp_cmd, ortn_cmd);
    //wam.moveTo(pose_cmd, false); //(TODO:KM Update Libbarrett API for Pose Moves)
    //ROS_INFO("Pose Commands for WAM not yet supported by API");
    //return false;
    return true;
}

//Function to command a cartesian move to the WAM
template<size_t DOF>
bool WamBringup<DOF>::cartMove(wam_srvs::CartPosMove::Request &req, wam_srvs::CartPosMove::Response &res)
{
    ROS_INFO("Moving Robot to Commanded Cartesian Position");
    for (int i = 0; i < 3; i++)
    {
        cp_cmd[i] = req.position[i];
    }
//  wam.moveTo(cp_cmd, false);
    return true;
}

//Function to command a cartesian velocity to the WAM based on forces
template<size_t DOF>
bool WamBringup<DOF>::cartVel(wam_srvs::CartVel::Request &req, wam_srvs::CartVel::Response &res)
{
    cp_type tanDir;
    double tanVel;
    // ROS_INFO("Moving Robot to Commanded Cartesian Velocity, good default values:  v_magnitude=0.1, kp=75");
    for (int i = 0; i < 3; i++)
    {
        tanDir[i]=req.v_direction[i];
    }
    tanVel=req.v_magnitude;
    /***************systemConnections_START*******************/
    //*** connect tangent velocity controller
    double kp_tanVel = 0.0;
    kp_tanVel=req.kp;
    VelocityComp.setKp(kp_tanVel);
    /*  double Vd = 0.0;
    std::string Vd_Input;
    printf(">>> Desired tangential Velocity (m/s) : ");
    std::getline(std::cin, Vd_Input);
    Vd = strtod(Vd_Input.c_str(), NULL);
    */
    exposedDesiredVel.setValue(tanVel);
    exposedDirVel.setValue(tanDir);
    //Note:tf2jt is used in different services
    systems::forceConnect(wam.kinematicsBase.kinOutput, tf2jt_cartVel.kinInput);
    if(req.visual_system)
    {
        systems::forceConnect(vhp->tangentDirectionOutput, computeTangentVel.tangentDirInput); //exposedDirVel.output
    }
    else
    {
            systems::forceConnect(exposedDirVel.output, computeTangentVel.tangentDirInput); //
    }
    systems::forceConnect(wam.toolVelocity.output  , computeTangentVel.toolVelocityInput);
    systems::forceConnect(computeTangentVel.tangentVelOutput, VelocityComp.feedbackInput);
    systems::forceConnect(exposedDesiredVel.output, VelocityComp.referenceInput);
    if(req.visual_system)
    {
        systems::forceConnect(vhp->tangentDirectionOutput, tg2.getInput<0>());
    }
    else
    {
        systems::forceConnect(exposedDirVel.output, tg2.getInput<0>());
    }
    if(tanVel==0)
    {
        systems::forceConnect(zero.output, tg2.getInput<1>());
    }
    else
    {
        systems::forceConnect(VelocityComp.controlOutput, tg2.getInput<1>());
    }
    systems::forceConnect(tg2.output, mult2.input);
    systems::forceConnect(mult2.output, tf2jt_cartVel.input);
    systems::forceConnect(tf2jt_cartVel.output, jtSat_cartVel.input);
    if(req.visual_system)
    {
        systems::forceConnect(jtSat_cartVel.output, jtSummer.getInput(2));
    }
    else //connect for use independent cartVel
    {
        wam.trackReferenceSignal(jtSat_cartVel.output);
    }
 /***************systemConnections_END*******************/
    return true;
}

//Function to command an orientation move to the WAM
template<size_t DOF>
bool WamBringup<DOF>::ortnMove(wam_srvs::OrtnMove::Request &req, wam_srvs::OrtnMove::Response &res)
{
    ROS_INFO("Moving Robot to Commanded End Effector Orientation");
    ortn_cmd.x() = req.orientation[0];
    ortn_cmd.y() = req.orientation[1];
    ortn_cmd.z() = req.orientation[2];
    ortn_cmd.w() = req.orientation[3];
    // wam.moveTo(ortn_cmd, false);
    return true;
}

//Function to command an orientation with varaible gains
template<size_t DOF>
bool WamBringup<DOF>::ortnSplitMove(wam_srvs::OrtnSplitMove::Request &req, wam_srvs::OrtnSplitMove::Response &res)
{
    ROS_INFO("Moving Robot to Commanded End Effector Orientation");
    Eigen::Quaterniond Orn_Quaternion;
    cp_type OrnKp , OrnKd ;
    OrnKp << 0.0, 0.0, 0.0;
    OrnKd << 0.0, 0.0, 0.0;
    Orn_Quaternion.x() = req.orientation[0];
    Orn_Quaternion.y() = req.orientation[1];
    Orn_Quaternion.z() = req.orientation[2];
    Orn_Quaternion.w() = req.orientation[3];
    OrnKp[0]=req.kp_gain[0]; //rot-X-tool
    OrnKp[1]=req.kp_gain[1]; //rot-Y-tool
    OrnKp[2]=req.kp_gain[2]; //rot-Z-tool
    //In the kD we used the angular velocity that's why the order is different
    OrnKd[0]=req.kd_gain[2];//rot-Z-tool
    OrnKd[1]=req.kd_gain[1];//rot-Y-tool
    OrnKd[2]=req.kd_gain[0]; //rot-X-tool
    orientationSetPoint.setValue(Orn_Quaternion);
    KpOrnSet.setValue(OrnKp);
    KdOrnSet.setValue(OrnKd);
    systems::forceConnect(KpOrnSet.output  ,  OrtnSplitCont.KpGains);
    systems::forceConnect(KdOrnSet.output  ,  OrtnSplitCont.KdGains);
    systems::forceConnect(wam.toolOrientation.output , OrtnSplitCont.FeedbackOrnInput);
    systems::forceConnect(orientationSetPoint.output , OrtnSplitCont.ReferenceOrnInput);
    systems::forceConnect(wam.kinematicsBase.kinOutput, OrtnSplitCont.kinInput);
    systems::forceConnect(wam.kinematicsBase.kinOutput, tt2jt_ortn_split.kinInput);
    systems::forceConnect(OrtnSplitCont.CTOutput , tt2jt_ortn_split.input);
    systems::forceConnect(tt2jt_ortn_split.output, jtSat_ornSplit.input);
    systems::forceConnect(jtSat_ornSplit.output, wam.input);
    //wam.trackReferenceSignal(jtSat_ornSplit.output);
    return true;
}

//Function to apply a force and torque to the WAM end effector with respect to the base frame
template<size_t DOF>
bool WamBringup<DOF>::forceTorqueBaseTime(wam_srvs::ForceTorqueToolTime::Request &req, wam_srvs::ForceTorqueToolTime::Response &res)
{
    //    req.force[];
    // req.torque[];
    jt_type jtLimits(30.0);
    double force_normal;
    double force_tangent;
    cf_type ForceApplied;
    ct_type TorqueApplied;
    double sleep_time = req.time;
    for(int i = 0; i < 3; i++)
    {
        ForceApplied[i]=req.force[i];
        TorqueApplied[i]=req.torque[i];
    }
    exposedOutputForce.setValue(ForceApplied);
    exposedOutputTorque.setValue(TorqueApplied);
    systems::forceConnect(exposedOutputForce.output, toolforce2jt.input);
    systems::forceConnect(exposedOutputTorque.output, tooltorque2jt.input);
    systems::forceConnect(wam.kinematicsBase.kinOutput, toolforce2jt.kinInput);
    systems::forceConnect(wam.kinematicsBase.kinOutput, tooltorque2jt.kinInput);
    systems::forceConnect(toolforce2jt.output,  torqueSum.getInput(0));
    systems::forceConnect(tooltorque2jt.output, torqueSum.getInput(1));
    systems::forceConnect(torqueSum.output, jtSat.input);
    systems::forceConnect(jtSat.output,wam.input);
    // wam.trackReferenceSignal(torqueSum.output);
    std::cout << "Applying force("<<req.force[0]<<","<<req.force[1]<<","<<req.force[2]<<")N and torque ("<<req.torque[0]<<","<<req.torque[1]<<","<<req.torque[2]<<") N*m for " << sleep_time << " seconds" << std::endl;
    btsleep(sleep_time);
    systems::disconnect(wam.input);
    return true;
}

template<size_t DOF>
bool WamBringup<DOF>::forceTorqueToolTime(wam_srvs::ForceTorqueToolTime::Request &req, wam_srvs::ForceTorqueToolTime::Response &res)
{
    //    req.force[];
    // req.torque[];
    jt_type jtLimits(30.0);
    cf_type ForceApplied;
    ct_type TorqueApplied;
    double sleep_time = req.time;
    for(int i = 0; i < 3; i++)
    {
        ForceApplied[i]=req.force[i];
        TorqueApplied[i]=req.torque[i];
    }
    exposedOutputForce.setValue(ForceApplied);
    exposedOutputTorque.setValue(TorqueApplied);
    systems::forceConnect(wam.toolPosition.output, appForce1.CpInput);
    systems::forceConnect(wam.toolOrientation.output, appForce1.OrnInput);
    systems::forceConnect(exposedOutputForce.output, appForce1.FtInput);
    systems::forceConnect(exposedOutputTorque.output, appForce1.TtInput);
    systems::forceConnect(appForce1.CFOutput, toolforce2jt.input);
    systems::forceConnect(appForce1.CTOutput, tooltorque2jt.input);
    systems::forceConnect(wam.kinematicsBase.kinOutput, toolforce2jt.kinInput);
    systems::forceConnect(wam.kinematicsBase.kinOutput, tooltorque2jt.kinInput);
    systems::forceConnect(toolforce2jt.output, torqueSum.getInput(0));
    systems::forceConnect(tooltorque2jt.output, torqueSum.getInput(1));
    systems::forceConnect(torqueSum.output, jtSat.input);
    // std::cout << "Press [Enter] to apply force("<<req.force[0]<<","<<req.force[1]<<","<<","<<req.force[2]<<")N and torque ("<<req.torque[0]<<","<<req.torque[1]<<","<<req.torque[2]<<") N*m" << std::endl;
    //waitForEnter();
    //wam.idle();
    //btsleep(0.1);
    //wam.trackReferenceSignal(jtSat.output);
    systems::forceConnect(jtSat.output,wam.input);
    std::cout << "Applying force("<<req.force[0]<<","<<req.force[1]<<","<<req.force[2]<<")N and torque ("<<req.torque[0]<<","<<req.torque[1]<<","<<req.torque[2]<<") N*m for " << sleep_time << " seconds" << std::endl;
    btsleep(sleep_time);
    systems::disconnect(wam.input);
    return true;
}
//Function to apply a force and torque to the WAM end effector with respect to the base frame
template<size_t DOF>
bool WamBringup<DOF>::forceTorqueBase(wam_srvs::ForceTorqueBase::Request &req, wam_srvs::ForceTorqueBase::Response &res)
{
    if (req.mode == 0) {
        if (systems_connected) { // DISCONNECT SYSTEMS
            systems::disconnect(wam.input);
            systems_connected = false;
            ROS_INFO("ForceTorqueBase disconnected from mode 0");
        }
        return true;
    } 
    jt_type jtLimits(30.0);
    double force_normal;
    double force_tangent;
    cf_type ForceApplied;
    ct_type TorqueApplied;
    cp_type InputKp;
    cp_type InputKd;
    // double sleep_time = req.time;
    for(int i = 0; i < 3; i++) {
        ForceApplied[i] = req.force[i];
        TorqueApplied[i] = req.torque[i];
        InputKp[i] = req.kp[i];
        InputKd[i] = req.kd[i];
    }
    if (req.mode == 1) { // CONNECT SYSTEMS
        if (systems_connected) {
            systems::disconnect(wam.input);
            systems_connected = false;
            ROS_INFO("ForceTorqueBase disconnected from mode 1");
            // btsleep(1.0);
        }
        // OrnKp << 4.0, 4.0, 4.0;
        // OrnKd << 0.02, 0.02, 0.02;
        orientationSetPoint.setValue(wam.getToolOrientation());
        KpOrnSet.setValue(InputKp);
        KdOrnSet.setValue(InputKd);
        systems::forceConnect(KpOrnSet.output, OrtnSplitCont.KpGains);
        systems::forceConnect(KdOrnSet.output, OrtnSplitCont.KdGains);
        systems::forceConnect(wam.toolOrientation.output, OrtnSplitCont.FeedbackOrnInput);
        systems::forceConnect(orientationSetPoint.output, OrtnSplitCont.ReferenceOrnInput);
        systems::forceConnect(wam.kinematicsBase.kinOutput, OrtnSplitCont.kinInput);
        systems::forceConnect(wam.kinematicsBase.kinOutput, tt2jt_ortn_split.kinInput);
        systems::forceConnect(OrtnSplitCont.CTOutput, tt2jt_ortn_split.input);
        // set up force
        // TorqueApplied[2] = 0.0;
        exposedOutputForce.setValue(ForceApplied);
        exposedOutputTorque.setValue(TorqueApplied);
        systems::forceConnect(exposedOutputForce.output, toolforce2jt.input);
        systems::forceConnect(exposedOutputTorque.output, tooltorque2jt.input);
        systems::forceConnect(wam.kinematicsBase.kinOutput, toolforce2jt.kinInput);
        systems::forceConnect(wam.kinematicsBase.kinOutput, tooltorque2jt.kinInput);
        // connect all to summer
        systems::forceConnect(toolforce2jt.output, torqueSum.getInput(0));
        systems::forceConnect(tooltorque2jt.output, torqueSum.getInput(1));
        systems::forceConnect(tt2jt_ortn_split.output, torqueSum2.getInput(0));
        systems::forceConnect(torqueSum.output, torqueSum2.getInput(1));
        // saturate and send to wam input
        systems::forceConnect(torqueSum2.output, jtSat.input);
        systems::forceConnect(jtSat.output, wam.input); 
        systems_connected = true;
        ROS_INFO("ForceTorqueBase cartesian position connected from mode 1");
        return true;
    } else if (req.mode == 2) { // UPDATE SYSTEMS
        exposedOutputForce.setValue(ForceApplied);
        exposedOutputTorque.setValue(TorqueApplied);
        KpOrnSet.setValue(InputKp);
        KdOrnSet.setValue(InputKd);
        systems_connected = true;
        return true;
        // std::cout << "Applying force [" << ForceApplied[0]<<","<< ForceApplied[1]<<","<<ForceApplied[2]<<"]N and torque ["<<TorqueApplied[0]<<","<<TorqueApplied[1]<<","<<TorqueApplied[2]<<"] N*m " << std::endl;
    //     force_normal=ForceApplied[2];
    //     force_tangent=ForceApplied[1];
    //     tangentGain.setGain(force_tangent);
    //     normalForceGain.setGain(force_normal+fn);
    } else if (req.mode == 3) { 
        // CONNECT CARTESIAN SPRING POSITION FOR ORIENTATION CONTROL
        if (systems_connected) {
            systems::disconnect(wam.input);
            systems_connected = false;
            ROS_INFO("ForceTorqueBase disconnected from mode 3");
            btsleep(1.0);
        }
        cp_type Kx;
        cp_type Dx;
        cp_type Kth;
        cp_type Xd;
        cp_type Thetad;
        Kx << 0.0, 0.0, 0.0;
        Dx << 1.0, 1.0, 1.0;
        Kth << 0.0, 0.0, 0.0;        
        Xd << wam.getToolPosition(); // e.g. position at [0,0,0,-pi/2,0,0,0] is [0.4 , 0 , 0.6]
        Thetad << -1.57, 0.0, 0.0; // e.g. orientation at [0,0,0,-pi/2,0,0,0] is [-pi/2 , 0 , 0]
        KxSet.setValue(Kx);
        DxSet.setValue(Dx);
        KthSet.setValue(Kth);
        XdSet.setValue(Xd);
        ThetadSet.setValue(Thetad);
        systems::forceConnect(KxSet.output, ImpControl.KxInput);
        systems::forceConnect(DxSet.output, ImpControl.DxInput);
        systems::forceConnect(KthSet.output, ImpControl.KthInput);
        systems::forceConnect(XdSet.output, ImpControl.XdInput);
        systems::forceConnect(ThetadSet.output, ImpControl.ThetadInput);

        systems::forceConnect(wam.toolPosition.output, ImpControl.CpInput);
        systems::forceConnect(wam.toolVelocity.output, ImpControl.CvInput);
        systems::forceConnect(wam.toolOrientation.output, ImpControl.OrnInput);

        systems::forceConnect(wam.kinematicsBase.kinOutput, toolforce2jt.kinInput);
        systems::forceConnect(wam.kinematicsBase.kinOutput, tooltorque2jt.kinInput);

        systems::forceConnect(ImpControl.CFOutput, toolforce2jt.input);
        systems::forceConnect(ImpControl.CTOutput, tooltorque2jt.input);

        systems::forceConnect(toolforce2jt.output, torqueSum.getInput(0));
        systems::forceConnect(tooltorque2jt.output, torqueSum.getInput(1));

        // systems::forceConnect(torqueSum.output, jtSat.input);
        // CONNECT FORCE TORQUE TOOL 
        exposedOutputForce.setValue(ForceApplied);
        exposedOutputTorque.setValue(TorqueApplied);
        systems::forceConnect(wam.toolPosition.output, appForce1.CpInput);
        systems::forceConnect(wam.toolOrientation.output, appForce1.OrnInput);

        systems::forceConnect(exposedOutputForce.output, appForce1.FtInput);
        systems::forceConnect(exposedOutputTorque.output, appForce1.TtInput);

        systems::forceConnect(appForce1.CFOutput, toolforce2jt2.input);
        systems::forceConnect(appForce1.CTOutput, tooltorque2jt2.input);

        systems::forceConnect(wam.kinematicsBase.kinOutput, toolforce2jt2.kinInput);
        systems::forceConnect(wam.kinematicsBase.kinOutput, tooltorque2jt2.kinInput);

        systems::forceConnect(toolforce2jt2.output, torqueSum2.getInput(0));
        systems::forceConnect(tooltorque2jt2.output, torqueSum2.getInput(1));
        // CONNECT BOTH TO SUMMER
        systems::forceConnect(torqueSum.output, torqueSum3.getInput(0));
        systems::forceConnect(torqueSum2.output, torqueSum3.getInput(1));
        
        systems::forceConnect(torqueSum3.output, jtSat.input);
        systems::forceConnect(jtSat.output, wam.input); 
        ROS_INFO("ForceTorqueBase spring connected from mode 3");
        systems_connected = true;
        return true;
    } else if (req.mode == 4) { // UPDATE SPRING VALUES
        KxSet.setValue(InputKp);
        exposedOutputForce.setValue(ForceApplied);
        exposedOutputTorque.setValue(TorqueApplied);
        ROS_INFO("ForceTorqueBase updating Kx values");
        systems_connected = true;
        return true;
    } else {
        ROS_WARN_STREAM("INVALID MODE ASSIGNED");
    }

    // else if (req.mode == 4) { // UPDATE ORIENTATION CONTROL
    //     ROS_INFO("ForceTorqueBase mode 4");

    // }
    // wam.trackReferenceSignal(torqueSum.output);
    // std::cout << "Applying force("<<req.force[0]<<","<<req.force[1]<<","<<req.force[2]<<")N and torque ("<<req.torque[0]<<","<<req.torque[1]<<","<<req.torque[2]<<") N*m for " << sleep_time << " seconds" << std::endl;
    // btsleep(sleep_time);
    // systems::disconnect(wam.input);
    return true;
}

//Function to apply a force and torque to the WAM end effector with respect to the tool frame
template<size_t DOF>
bool WamBringup<DOF>::forceTorqueTool(wam_srvs::ForceTorqueTool::Request &req, wam_srvs::ForceTorqueTool::Response &res)
{
    //    req.force[];
    // req.torque[];
    jt_type jtLimits(30.0);
    cf_type ForceApplied;
    ct_type TorqueApplied;
    for(int i = 0; i < 3; i++)
    {
        ForceApplied[i]=req.force[i];
        TorqueApplied[i]=req.torque[i];
    }
    exposedOutputForce.setValue(ForceApplied);
    exposedOutputTorque.setValue(TorqueApplied);
    systems::forceConnect(wam.toolPosition.output, appForce1.CpInput);
    systems::forceConnect(wam.toolOrientation.output, appForce1.OrnInput);
    systems::forceConnect(exposedOutputForce.output, appForce1.FtInput);
    systems::forceConnect(exposedOutputTorque.output, appForce1.TtInput);
    systems::forceConnect(appForce1.CFOutput, toolforce2jt.input);
    systems::forceConnect(appForce1.CTOutput, tooltorque2jt.input);
    systems::forceConnect(wam.kinematicsBase.kinOutput, toolforce2jt.kinInput);
    systems::forceConnect(wam.kinematicsBase.kinOutput, tooltorque2jt.kinInput);
    systems::forceConnect(toolforce2jt.output, torqueSum.getInput(0));
    systems::forceConnect(tooltorque2jt.output, torqueSum.getInput(1));
    systems::forceConnect(torqueSum.output, jtSat.input);
    std::cout << "Press [Enter] to apply force("<<req.force[0]<<","<<req.force[1]<<","<<","<<req.force[2]<<")N and torque ("<<req.torque[0]<<","<<req.torque[1]<<","<<req.torque[2]<<") N*m" << std::endl;
    //waitForEnter();
    //wam.idle();
    //btsleep(0.1);
    //wam.trackReferenceSignal(jtSat.output);
    systems::forceConnect(jtSat.output,wam.input);
    return true;
}

//Function to play a taught motion to the WAM
template<size_t DOF>
bool WamBringup<DOF>::playMotion(wam_srvs::Play::Request &req, wam_srvs::Play::Response &res)
{
    // Build spline between recorded points
    std::string path = "/home/robot/motions/" + req.path;
    ROS_INFO_STREAM("Playing back motion from : " << path);
    log::Reader<jp_sample_type> lr(path.c_str());
    std::vector<jp_sample_type> vec;
    for (size_t i = 0; i < lr.numRecords(); ++i)
    {
        vec.push_back(lr.getRecord());
    }
    math::Spline<jp_type> spline(vec);
    btsleep(0.3);  // wait an execution cycle or two
    ROS_INFO_STREAM("Move to start");
    // First, move to the starting position
    // UNCOMMENT
    // wam.moveTo(spline.eval(spline.initialS()));
    // Then play back the recorded motion
    systems::Ramp time(mypm->getExecutionManager());
    ROS_INFO_STREAM("Stop time");
    time.stop();
    time.setOutput(spline.initialS());
    systems::Callback<double, jp_type> trajectory(boost::ref(spline));
    connect(time.output, trajectory.input);
    wam.trackReferenceSignal(trajectory.output);
    ROS_INFO_STREAM("Start time");
    time.start();
    ROS_INFO_STREAM("Wait to finish");
    while (trajectory.input.getValue() < spline.finalS()) {
            usleep(100000);
    }
    wam.moveTo(wam.getJointPositions());
    ROS_INFO_STREAM("Playback finished");
    return true;
}

template<size_t DOF>
bool WamBringup<DOF>::linkArm(wam_srvs::Link::Request &req, wam_srvs::Link::Response &res)
{
    //Setup master
    if(mm == NULL)
    {
        mm = new MasterMaster<DOF>(mypm->getExecutionManager(), const_cast<char*>(req.remote_ip.c_str()));
        rt_hand_tool_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, -0.01, -0.2 ; //CP: only last 3 joint are use for hand tool; the first joint set free mode if it is 1.0 or attached if it is different
        exposedOutputHandTool.setValue(rt_hand_tool_cmd);
        systems::connect(wam.jpOutput, mm->JpInput);
        systems::connect(exposedOutputHandTool.output, mm->JpHandTool);
    }
    const jp_type SYNC_POS(0.0);  // the position each WAM should move to before linking
    wam.gravityCompensate();
    wam.moveTo(SYNC_POS);
    ROS_INFO_STREAM("Press [Enter] to link with the other WAM.");
    waitForEnter();
    mm->tryLink();
    wam.trackReferenceSignal(mm->JpOutput);
    btsleep(0.1);  // wait an execution cycle or two
    if (mm->isLinked())
    {
        ROS_INFO_STREAM("Linked with remote WAM.\n");
    }
    else
    {
        ROS_INFO_STREAM("WARNING: Linking was unsuccessful.\n");
    }
    return true;
}

template<size_t DOF>
bool WamBringup<DOF>::unLinkArm(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    mm->unlink();
    wam.moveTo(wam.getJointPositions());
}

template<size_t DOF>
bool WamBringup<DOF>::startVisualFix(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    connect(wam.kinematicsBase.kinOutput, tf2jt.kinInput);
    connect(dir_sys.output, tg.getInput<0>());
    connect(vs_error_sys.output, comp.referenceInput);
    connect(zero.output, comp.feedbackInput);
    connect(comp.controlOutput, tg.getInput<1>());
    connect(tg.output, mult.input);
    connect(mult.output, tf2jt.input);
    connect(tf2jt.output, jtSat.input);
    // adjust velocity fault limit
    mypm->getSafetyModule()->setVelocityLimit(1.5);
    connect(jtSat.output, wam.input);
    return true;
}


template<size_t DOF>
bool WamBringup<DOF>::stopVisualFix(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    systems::disconnect(wam.input);
    return true;
}

//Function to make the robot follow a predefine path and tool orientation
template<size_t DOF>
bool WamBringup<DOF>::followPath(wam_srvs::FollowPath::Request &req, wam_srvs::FollowPath::Response &res)
{
    cp_type cp_position;
    cp_type cp_normal;
    std::vector<cp_type> vecPositions;
    std::vector<cp_type> vecNormalDirs;
    Eigen::Quaterniond quaternion_fixed_normal;
    std::string fix_normal_direction;
    std::string line;
    std::string lineForceTangentInput;
    std::string lineForceNormalInput;
    double fix_normal_flag=1.0;
    quaternion_fixed_normal.x()=0.0;
    quaternion_fixed_normal.y()=1.0;
    quaternion_fixed_normal.z()=0.0;
    quaternion_fixed_normal.w()=0.0;
    jt_type jt_temp;
    for(int i=0 ; i<DOF; i++)
    {
        jt_temp[i]=0.0;
    }
    ROS_INFO("The number of points are: %d", req.size);
    for (size_t i = 0; i < req.size; ++i)
    {
        cp_position[0]=req.position[i].x;
        cp_position[1]=req.position[i].y;
        cp_position[2]=req.position[i].z;
        vecPositions.push_back(cp_position);
        cp_normal[0]=req.normal[i].x;
        cp_normal[1]=req.normal[i].y;
        cp_normal[2]=req.normal[i].z;
        vecNormalDirs.push_back(cp_normal);
        ROS_INFO("point %d:(%f,%f,%f)",i,req.normal[i].x,req.normal[i].y,req.normal[i].z);
    }
    //Build spline between recorded points
    //math::Spline<cp_type> spline_Position(vecPositions);
    //math::Spline<cf_type> spline_NormalDir(vecNormalDirs);
    //Setup master
    //if(vhp == NULL) {
    vhp = new VisualHapticPathNormals(vecPositions,vecNormalDirs);
    //  }
    //systems::VisualHapticPath vhp(vecPositions);
    //systems::PIDController<double, double> comp;
    //systems::Constant<double> zero(0.0);
    //systems::TupleGrouper<cf_type, double> tg;
    //systems::Callback<boost::tuple<cf_type, double>, cf_type> mult(scale);
    //systems::Gain<cp_type, double, cf_type> tangentGain(0.0);//CP
    //systems::Summer<cf_type> tfSum;//CP
    //systems::Summer<cf_type> tfSumNormalTangential;/CP
    //systems::ToolForceToJointTorques<DOF> tf2jt;
    //systems::ExposedOutput<double> normalForceMagnitude ;
    //systems::Gain<cp_type, double, cf_type> normalForceGain(0.0);//CP
    //systems::ExposedOutput<Eigen::Quaterniond> orientationSetPoint ;//CP
    //jt_type jtLimits(35.0);
    //systems::Callback<jt_type> jtSat(boost::bind(saturateJt<DOF>, _1, jtLimits));
    //configure Systemszero
    double kp = 200;
    double kd = 10;
    std::string kp_path;
    std::string kd_path;
    printf(">>> type the kp_path controller (default=0)(range:0-5000) : ");
    std::getline(std::cin, kp_path);
    kp = strtod(kp_path.c_str(), NULL);
    printf(">>> type the kd_path controller(default=10)(range(0-50)) : ");
    std::getline(std::cin, kd_path);
    kd = strtod(kd_path.c_str(), NULL);
    if(kp<0 || kp>5000)
    {
        kp=0;
    }
        if(kd<0 || kd>50)
    {
        kd=0;
    }
    comp.setKp(kp);
    comp.setKd(kd);
    // connect Systems
    systems::forceConnect(wam.toolPosition.output, vhp->input);
    systems::forceConnect(wam.kinematicsBase.kinOutput, tf2jt.kinInput);
    systems::forceConnect(vhp->directionOutput, tg.getInput<0>());
    systems::forceConnect(vhp->depthOutput, comp.referenceInput);
    systems::forceConnect(zero.output, comp.feedbackInput);
    systems::forceConnect(comp.controlOutput, tg.getInput<1>());
    systems::forceConnect(tg.output, mult.input);
    systems::forceConnect(mult.output, tfSum.getInput(0));
    systems::forceConnect(vhp->tangentDirectionOutput, tangentGain.input);
    systems::forceConnect(tangentGain.output, tfSum.getInput(1));
    systems::forceConnect(tfSum.output ,  tfSumNormalTangential.getInput(0));
    systems::forceConnect(vhp->normalDirectionOutput, normalForceGain.input);
    systems::forceConnect(normalForceGain.output, tfSumNormalTangential.getInput(1));
    /*
    //
    //Tool Orientation Controller
    systems::forceConnect(wam.kinematicsBase.kinOutput,toolOrntController.kinInput);
    systems::forceConnect(wam.toolOrientation.output,toolOrntController.feedbackInput);
    //  systems::forceConnect(orientationSetPoint.output,toolOrntController.referenceInput);
    systems::forceConnect(vhp->tangentDirectionOutput, computeQuaternionSetPoint1.tangentDirInput);
    systems::forceConnect(vhp->normalDirectionOutput , computeQuaternionSetPoint1.normalDirInput);
    systems::forceConnect(computeQuaternionSetPoint1.OrnOutput, toolOrntController.referenceInput);
    systems::forceConnect(toolOrntController.controlOutput,tt2jtOrnController.input);
    systems::forceConnect(wam.kinematicsBase.kinOutput,tt2jtOrnController.kinInput);
    systems::forceConnect(tt2jtOrnController.output, jtSummer.getInput(0));
    //
    */
///ERASE//////////////////////////////////////////////////////////////////////////
    ROS_INFO("Set individual axis Orientation controller");
    cp_type OrnKp , OrnKd ;
    std::string kp_ort_x, kp_ort_y, kp_ort_z ;
    std::string kd_ort_x, kd_ort_y, kd_ort_z ;
    OrnKp << 0.0, 0.0, 0.0;
    OrnKd << 0.0, 0.0, 0.0;
    /*
    OrnKp[0]=req.kp_gain[0]; //rot-X-tool
    OrnKp[1]=req.kp_gain[1]; //rot-Y-tool
    OrnKp[2]=req.kp_gain[2]; //rot-Z-tool
    //In the kD we used the angular velocity that's why the order is different
    OrnKd[0]=req.kd_gain[2];//rot-Z-tool
    OrnKd[1]=req.kd_gain[1];//rot-Y-tool
    OrnKd[2]=req.kd_gain[0]; //rot-X-tool
    */
    // orientationSetPoint.setValue(quaternion_fixed_normal);
    ///////////////
    printf(">>> type the kp rot-X-tool controller (default=0.0)(range:0.0-50) : ");
    std::getline(std::cin, kp_ort_x);
    OrnKp[0] = strtod(kp_ort_x.c_str(), NULL);
    if(OrnKp[0]<= 0.0 || OrnKp[0]>50)
    {
        OrnKp[0]=0.0;
    }
    printf(">>> type the kp rot-Y-tool controller (default=0.0)(range:0.0-50) : ");
    std::getline(std::cin, kp_ort_y);
    OrnKp[1] = strtod(kp_ort_y.c_str(), NULL);
    if(OrnKp[1]<= 0.0 || OrnKp[1]>50)
    {
        OrnKp[1]=0.0;
    }
    printf(">>> type the kp rot-Z-tool controller (default=0.0)(range:0.0-50) : ");
    std::getline(std::cin, kp_ort_z);
    OrnKp[2] = strtod(kp_ort_z.c_str(), NULL);
    if(OrnKp[2]<= 0.0 || OrnKp[2]>50)
    {
        OrnKp[2]=0.0;
    }
    printf(">>> type the kd rot-X-tool controller (default=0.0)(range:0.0-1.0) : ");
    std::getline(std::cin, kd_ort_x);
    OrnKd[0] = strtod(kp_ort_x.c_str(), NULL);
    if(OrnKd[0]<= 0.0 || OrnKd[0]>1.0)
    {
        OrnKd[0]=0.0;
    }
    printf(">>> type the kd rot-Y-tool controller (default=0.0)(range:0.0-1.0) : ");
    std::getline(std::cin, kd_ort_y);
    OrnKd[1] = strtod(kd_ort_y.c_str(), NULL);
    if(OrnKd[1]<= 0.0 || OrnKd[1]>1.0)
    {
        OrnKd[1]=0.0;
    }
    printf(">>> type the kd rot-Z-tool controller (default=0.0)(range:0.0-1.0) : ");
    std::getline(std::cin, kd_ort_z);
    OrnKd[2] = strtod(kd_ort_z.c_str(), NULL);
    if(OrnKd[2]<= 0.0 || OrnKd[2]>1.0)
    {
        OrnKd[2]=0.0;
    }
/////////////////////////
    KpOrnSet.setValue(OrnKp);
    KdOrnSet.setValue(OrnKd);
    systems::forceConnect(KpOrnSet.output  ,  OrtnSplitCont.KpGains);
    systems::forceConnect(KdOrnSet.output  ,  OrtnSplitCont.KdGains);
    systems::forceConnect(wam.toolOrientation.output , OrtnSplitCont.FeedbackOrnInput);
    printf(">>>type  1--> z direction(table), 2 --> x direction(wall), 3 --> variable orientation: ");
    std::getline(std::cin, fix_normal_direction);
    fix_normal_flag=strtod(fix_normal_direction.c_str(), NULL);
    if(fix_normal_flag==2.0)
    {
        quaternion_fixed_normal.x()=0.0;
        quaternion_fixed_normal.y()=-0.707;
        quaternion_fixed_normal.z()=0.0;
        quaternion_fixed_normal.w()=0.707;
        orientationSetPoint.setValue(quaternion_fixed_normal);
        systems::forceConnect(orientationSetPoint.output , OrtnSplitCont.ReferenceOrnInput);
    }
    else if(fix_normal_flag==3.0)
    {
        systems::forceConnect(vhp->tangentDirectionOutput, computeQuaternionSetPoint1.tangentDirInput);
        systems::forceConnect(vhp->normalDirectionOutput , computeQuaternionSetPoint1.normalDirInput);
        systems::forceConnect(computeQuaternionSetPoint1.OrnOutput, OrtnSplitCont.ReferenceOrnInput);
    }
    else
    {
        quaternion_fixed_normal.x()=0.0;
        quaternion_fixed_normal.y()=1.0;
        quaternion_fixed_normal.z()=0.0;
        quaternion_fixed_normal.w()=0.0;
        orientationSetPoint.setValue(quaternion_fixed_normal);
        systems::forceConnect(orientationSetPoint.output , OrtnSplitCont.ReferenceOrnInput);
    }
    systems::forceConnect(wam.kinematicsBase.kinOutput, OrtnSplitCont.kinInput);
    systems::forceConnect(wam.kinematicsBase.kinOutput, tt2jt_ortn_split.kinInput);
    systems::forceConnect(OrtnSplitCont.CTOutput , tt2jt_ortn_split.input);
    systems::forceConnect(tt2jt_ortn_split.output, jtSummer.getInput(0));
    //systems::forceConnect(jtSat_ornSplit.output, wam.input);
//END_ERASE////////////////////////////////////////////////////////////////////
//This is for adding torque coming from forceTorqueBase service(joystick)
    zero_joystick_torque.setValue(jt_temp);
    systems::forceConnect(zero_joystick_torque.output,jtSummer.getInput(2));
    systems::forceConnect(tfSumNormalTangential.output, tf2jt.input);
    //connect(tf2jt.output, jtSat.input);
        systems::forceConnect(tf2jt.output, jtSummer.getInput(1));
        systems::forceConnect(jtSummer.output, jtSat.input);
/*
    double kp_ort_n = 1.0;
    double kd_ort_n = 0.01;
    std::string kp_ort;
    std::string kd_ort;
    printf(">>> type the kp_orientation controller (default=0.0)(range:0.0-10) : ");
    std::getline(std::cin, kp_ort);
    kp_ort_n = strtod(kp_ort.c_str(), NULL);
    printf(">>> type the kd_orientation controller(default=0.0)(range(0.0-1.0)) : ");
    std::getline(std::cin, kd_ort);
    kd_ort_n = strtod(kd_ort.c_str(), NULL);
    if(kp_ort_n <= 0.0 || kp_ort_n>10)
    {
        kp_ort_n=0.0;
    }
        if(kd_ort_n<=0.0 || kd_ort_n>1.0)
    {
        kd_ort_n=0.0;
    }
        toolOrntController.setKp(kp_ort_n);
        toolOrntController.setKd(kd_ort_n);
*/
    //wam.idle();
    printf(">>> type the tangential force mag. : ");
    std::getline(std::cin, lineForceTangentInput);
    double ft = strtod(lineForceTangentInput.c_str(), NULL);
    tangentGain.setGain(ft);
    printf(">>> type the normal force mag. : ");
    std::getline(std::cin, lineForceNormalInput);
    fn = strtod(lineForceNormalInput.c_str(), NULL);
    normalForceGain.setGain(fn);
    //wam.idle();
    printf("Press [Enter] to set  Orn and connect to wam.input.\n");
    waitForEnter();
    //btsleep(1.0);
    //orientationSetPoint.setValue(wam.getToolOrientation());
    //orientationSetPoint.setValue(quaternion_fixed_normal);
    //wam.trackReferenceSignal(orientationSetPoint.output);
    systems::forceConnect(jtSat.output, wam.input);
    //wam.trackReferenceSignal(jtSat.output);
    //printf("Press [Enter] to disconnect the forces \n");
    //waitForEnter();
    //printf("After pressing enter \n");
    //systems::disconnect(wam.input);
    //wam.idle();
    //wam.moveTo(wam.getJointPositions());
    //ROS_INFO_STREAM("finished");
    //wam.idle();
    //btsleep(5.0);
    printf("Out of service \n");
    return true;
}

//Function to teach a motion to the WAM
template<size_t DOF>
bool WamBringup<DOF>::teachMotion(wam_srvs::Teach::Request &req, wam_srvs::Teach::Response &res)
{
    ROS_INFO("Replaying a motion on the robot");
    systems::Ramp time(mypm->getExecutionManager());
    systems::TupleGrouper<double, jp_type> jpLogTg;
    const double T_s = mypm->getExecutionManager()->getPeriod();
    std::string path = "/home/robot/motions/" + req.path;
    ROS_INFO_STREAM("Teaching a motion to the robot. Saving to: " << path);
    // Record at 1/10th of the loop rate
    systems::PeriodicDataLogger<jp_sample_type> jpLogger(mypm->getExecutionManager(), new barrett::log::RealTimeWriter<jp_sample_type>(path.c_str(), 10*T_s), 10);
    printf("Press [Enter] to start teaching.\n");
    waitForEnter();
    {
        // Make sure the Systems are connected on the same execution cycle
        // that the time is started. Otherwise we might record a bunch of
        // samples all having t=0; this is bad because the Spline requires time
        // to be monotonic.
        BARRETT_SCOPED_LOCK(mypm->getExecutionManager()->getMutex());
        connect(time.output, jpLogTg.template getInput<0>());
        connect(wam.jpOutput, jpLogTg.template getInput<1>());
        connect(jpLogTg.output, jpLogger.input);
        time.start();
    }
    printf("Press [Enter] to stop teaching.\n");
    waitForEnter();
    jpLogger.closeLog();
    disconnect(jpLogger.input);
    ROS_INFO_STREAM("Teaching done.");
    return true;
}

//Callback function for RT Cartesian Velocity messages
template<size_t DOF>
void WamBringup<DOF>::cartVelCB(const wam_msgs::RTCartVel::ConstPtr& msg)
{
    // ROS_INFO_STREAM("Reached cartVelCB");
    if (cart_vel_status) {
        cp_type old_rt_cv_cmd = rt_cv_cmd;
        double old_cart_vel_mag = cart_vel_mag;
        for (size_t i = 0; i < 3; i++) {
            rt_cv_cmd[i] = msg->direction[i];
        } if (msg->magnitude != 0) {
            cart_vel_mag = msg->magnitude;
        }
        if (old_rt_cv_cmd != rt_cv_cmd || old_cart_vel_mag != cart_vel_mag) {
            ROS_INFO_STREAM("Change in Cartesian Velocity command received. New Magnitude: " << cart_vel_mag << ". New Cartesian Velocity: " << rt_cv_cmd);
        }
        new_rt_cmd = true;
    }
    last_cart_vel_msg_time = ros::Time::now();
}

//Callback function for RT Orientation Velocity Messages
template<size_t DOF>
void WamBringup<DOF>::ortnVelCB(const wam_msgs::RTOrtnVel::ConstPtr& msg)
{
    if (ortn_vel_status)
    {
        for (size_t i = 0; i < 3; i++)
        {
            rt_ortn_cmd[i] = msg->angular[i];
        }
        new_rt_cmd = true;
        if (msg->magnitude != 0)
        {
            ortn_vel_mag = msg->magnitude;
        }
    }
    last_ortn_vel_msg_time = ros::Time::now();
}

//Callback function for RT Joint Velocity Messages
template<size_t DOF>
void WamBringup<DOF>::jntVelCB(const wam_msgs::RTJointVel::ConstPtr& msg)
{
    if (msg->velocities.size() != DOF)
    {
        ROS_INFO("Commanded Joint Velocities != DOF of WAM");
        return;
    }
    if (jnt_vel_status)
    {
        for (size_t i = 0; i < DOF; i++)
        {
            rt_jv_cmd[i] = msg->velocities[i];
        }
        new_rt_cmd = true;
    }
    last_jnt_vel_msg_time = ros::Time::now();
}

//Callback function for RT Joint Position Messages
template<size_t DOF>
void WamBringup<DOF>::jntPosCB(const wam_msgs::RTJointPos::ConstPtr& msg)
{
    if (msg->joints.size() != DOF)
    {
        ROS_INFO("Commanded Joint Positions != DOF of WAM");
        return;
    }
    if (jnt_pos_status)
    {
        for (size_t i = 0; i < DOF; i++)
        {
            rt_jp_cmd[i] = msg->joints[i];
            rt_jp_rl[i] = msg->rate_limits[i];
        }
        new_rt_cmd = true;
    }
    else
    {
        for (size_t i = 0; i < DOF; i++)
        {
            rt_jp_rl[i] = 0;
        }
    }
    last_jnt_pos_msg_time = ros::Time::now();
}

//Callback function for RT Custom HandTool CP
template<size_t DOF>
void WamBringup<DOF>::jntHandToolCB(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (msg->position.size() != DOF)
    {
        ROS_INFO("Commanded Joint Positions != DOF of WAM");
        return;
    }
    if (jnt_hand_tool_status)
    {
        ROS_INFO("New hand tool msg");
        for (size_t i = 0; i < DOF; i++)
        {
            rt_hand_tool_cmd[i] = msg->position[i];
        }
        new_rt_cmd = true;
    }
    last_jnt_hand_tool_msg_time = ros::Time::now();
}

//Callback function for RT Cartesian Position Messages
template<size_t DOF>
void WamBringup<DOF>::cartPosCB(const wam_msgs::RTCartPos::ConstPtr& msg)
{
    if (cart_pos_status)
    {
        for (size_t i = 0; i < 3; i++)
        {
            rt_cp_cmd[i] = msg->position[i];
            rt_cp_rl[i] = msg->rate_limits[i];
        }
        new_rt_cmd = true;
    }
    else
    {
        for (size_t i = 0; i < 3; i++)
        {
            rt_cp_rl[i] = 0;
        }
    }
    last_cart_pos_msg_time = ros::Time::now();
}

template<size_t DOF>
void WamBringup<DOF>::vsErrCB(const std_msgs::Float32::ConstPtr& msg)
{
    float err = msg->data > 1000 ? 0.01 : msg->data/100000;
    vs_error_sys.setValue(msg->data);
}

//Function to update the WAM publisher
template<size_t DOF>
void WamBringup<DOF>::publishWam(ProductManager& pm)
{
    //Current values to be published
    jp_type jp = wam.getJointPositions();
    jt_type jt = wam.getJointTorques();
    jv_type jv = wam.getJointVelocities();
    cp_type cp_pub = wam.getToolPosition();
    Eigen::Quaterniond to_pub = wam.getToolOrientation();
    math::Matrix<6,DOF> robot_tool_jacobian=wam.getToolJacobian();
    //publishing sensor_msgs/JointState to wam/joint_states
    for (size_t i = 0; i < DOF; i++)
    {
        wam_joint_state.position[i] = jp[i];
        wam_joint_state.velocity[i] = jv[i];
        wam_joint_state.effort[i] = jt[i];
    }
    wam_joint_state.header.stamp = ros::Time::now();
    wam_joint_state_pub.publish(wam_joint_state);
    //publishing geometry_msgs/PoseStamed to wam/pose
    wam_pose.header.stamp = ros::Time::now();
    wam_pose.pose.position.x = cp_pub[0];
    wam_pose.pose.position.y = cp_pub[1];
    wam_pose.pose.position.z = cp_pub[2];
    wam_pose.pose.orientation.w = to_pub.w();
    wam_pose.pose.orientation.x = to_pub.x();
    wam_pose.pose.orientation.y = to_pub.y();
    wam_pose.pose.orientation.z = to_pub.z();
    wam_pose_pub.publish(wam_pose);
    //publishing wam_msgs/MatrixMN to wam/jacobian
    wam_jacobian_mn.m = 6;
    wam_jacobian_mn.n = DOF;
    //size_t index_loop=0;
    for (size_t h = 0; h < wam_jacobian_mn.n; ++h)
    {
        for (size_t k = 0; k < wam_jacobian_mn.m; ++k)
        {
            //wam_jacobian_mn[index_loop]=1;
            //index_loop++;index_loop++;
            //wam_jacobian_mn.data[index_loop]=robot_tool_jacobian(h,k);
            //index_loop++;
            wam_jacobian_mn.data[h*6+k]=robot_tool_jacobian(k,h);
            //wam_jacobian_mn.data.push_back((float32)robot_tool_jacobian(h,k));
            // wam_jacobian_mn.data.push_back(1.0);
        }
    }
    wam_jacobian_mn_pub.publish(wam_jacobian_mn);
}

//Function to update the real-time control loops
template<size_t DOF>
void WamBringup<DOF>::publishHand() //systems::PeriodicDataLogger<debug_tuple>& logger
{
    while (ros::ok())
    {
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
                bhand_joint_state.position[0] /= FINGER_RATIO;
                bhand_joint_state.position[1] /= FINGER_RATIO;
                bhand_joint_state.position[2] /= FINGER_RATIO;
                bhand_joint_state.position[3] /= SPREAD_RATIO;
                bhand_joint_state.position[4]  = bhand_joint_state.position[3];
                bhand_joint_state.position[5] = bhand_joint_state.position[0] * SECOND_LINK_RATIO;
                bhand_joint_state.position[6] = bhand_joint_state.position[1] * SECOND_LINK_RATIO;
                bhand_joint_state.position[7] = bhand_joint_state.position[2] * SECOND_LINK_RATIO;
                bhand_joint_state_pub.publish(bhand_joint_state);
            }
        }
        btsleep(1.0 / BHAND_PUBLISH_FREQ); // Sleep according to the specified publishing frequency
    }
}

//Function to update the real-time control loops
template<size_t DOF>
void WamBringup<DOF>::updateRT(ProductManager& pm) //systems::PeriodicDataLogger<debug_tuple>& logger
{
    //Real-Time Cartesian Velocity Control Portion
    // ros::Time time_check = last_cart_vel_msg_time + rt_msg_timeout;
    // std::cout << "\tupdateRT time check: " << time_check << std::endl;
    if (last_cart_vel_msg_time + rt_msg_timeout > ros::Time::now()) // checking if a cartesian velocity message has been published and if it is within timeout
    {
        if (!cart_vel_status)
        {
            cart_dir.setValue(cp_type(0.0, 0.0, 0.0)); // zeroing the cartesian direction
            current_cart_pos.setValue(wam.getToolPosition()); // Initializing the cartesian position
            current_ortn.setValue(wam.getToolOrientation()); // Initializing the orientation
            systems::forceConnect(ramp.output, mult_linear.input1); // connecting the ramp to multiplier
            systems::forceConnect(cart_dir.output, mult_linear.input2); // connecting the direction to the multiplier
            systems::forceConnect(mult_linear.output, cart_pos_sum.getInput(0)); // adding the output of the multiplier
            systems::forceConnect(current_cart_pos.output, cart_pos_sum.getInput(1)); // with the starting cartesian position offset
            systems::forceConnect(cart_pos_sum.output, rt_pose_cmd.getInput<0>()); // saving summed position as new commanded pose.position
            systems::forceConnect(current_ortn.output, rt_pose_cmd.getInput<1>()); // saving the original orientation to the pose.orientation
            ramp.setSlope(cart_vel_mag); // setting the slope to the commanded magnitude
            ramp.stop(); // ramp is stopped on startup
            ramp.setOutput(0.0); // ramp is re-zeroed on startup
            ramp.start(); // start the ramp
            // wam.trackReferenceSignal(rt_pose_cmd.output); // command WAM to track the RT commanded (500 Hz) updated pose
            wam.trackReferenceSignal(cart_pos_sum.output); // command WAM to track the RT commanded (500 Hz) updated pose
            cart_vel_status = true;
            ROS_INFO_STREAM("RT Cartesian Velocity setup to track reference signal");
            ROS_INFO_STREAM("cart_vel_status set to true");
        }
        else if (new_rt_cmd)
        {
            // ROS_INFO_STREAM("New RT Cartesian Velocity Command" << "\tMagnitude: " << cart_vel_mag << "\tCartesian Direction: " << rt_cv_cmd);
            ramp.reset(); // reset the ramp to 0
            ramp.setSlope(cart_vel_mag);
            cart_dir.setValue(rt_cv_cmd); // set our cartesian direction to subscribed command
            current_cart_pos.setValue(wam.tpoTpController.referenceInput.getValue()); // updating the current position to the actual low level commanded value
            // wam.moveTo(cart_pos_sum.output);
            new_rt_cmd = false;
        }
        cart_vel_status = true;
        new_rt_cmd = false;
    }
    // Real-Time Angular Velocity Control Portion
    else if (last_ortn_vel_msg_time + rt_msg_timeout > ros::Time::now()) // checking if a orientation velocity message has been published and if it is within timeout
    if (last_ortn_vel_msg_time + rt_msg_timeout > ros::Time::now()) // checking if a orientation velocity message has been published and if it is within timeout
    {
        if (!ortn_vel_status)
        {
            rpy_cmd.setValue(math::Vector<3>::type(0.0, 0.0, 0.0)); // zeroing the rpy command
            current_cart_pos.setValue(wam.getToolPosition()); // Initializing the cartesian position
            current_rpy_ortn.setValue(toRPY(wam.getToolOrientation())); // Initializing the orientation
            systems::forceConnect(ramp.output, mult_angular.input1); // connecting the ramp to multiplier
            systems::forceConnect(rpy_cmd.output, mult_angular.input2); // connecting the rpy command to the multiplier
            systems::forceConnect(mult_angular.output, ortn_cmd_sum.getInput(0)); // adding the output of the multiplier
            systems::forceConnect(current_rpy_ortn.output, ortn_cmd_sum.getInput(1)); // with the starting rpy orientation offset
            systems::forceConnect(ortn_cmd_sum.output, to_quat.input);
            systems::forceConnect(current_cart_pos.output, rt_pose_cmd.getInput<0>()); // saving the original position to the pose.position
            systems::forceConnect(to_quat.output, rt_pose_cmd.getInput<1>()); // saving the summed and converted new quaternion commmand as the pose.orientation
            ramp.setSlope(ortn_vel_mag); // setting the slope to the commanded magnitude
            ramp.stop(); // ramp is stopped on startup
            ramp.setOutput(0.0); // ramp is re-zeroed on startup
            ramp.start(); // start the ramp
            wam.trackReferenceSignal(rt_pose_cmd.output); // command the WAM to track the RT commanded up to (500 Hz) cartesian velocity
        }
        else if (new_rt_cmd)
        {
            ramp.reset(); // reset the ramp to 0
            ramp.setSlope(ortn_vel_mag); // updating the commanded angular velocity magnitude
            rpy_cmd.setValue(rt_ortn_cmd); // set our angular rpy command to subscribed command
            current_rpy_ortn.setValue(toRPY(wam.tpoToController.referenceInput.getValue())); // updating the current orientation to the actual low level commanded value
        }
        ortn_vel_status = true;
        new_rt_cmd = false;
    }
    //Real-Time Joint Velocity Control Portion
    else if (last_jnt_vel_msg_time + rt_msg_timeout > ros::Time::now()) // checking if a joint velocity message has been published and if it is within timeout
    {
        if (!jnt_vel_status)
        {
            jv_type jv_start;
            for (size_t i = 0; i < DOF; i++)
            {
                jv_start[i] = 0.0;
            }
            jv_track.setValue(jv_start); // zeroing the joint velocity command
            wam.trackReferenceSignal(jv_track.output); // command the WAM to track the RT commanded up to (500 Hz) joint velocities
        }
        else if (new_rt_cmd)
        {
            jv_track.setValue(rt_jv_cmd); // set our joint velocity to subscribed command
        }
        jnt_vel_status = true;
        new_rt_cmd = false;
    }
    //Real-Time Joint Position Control Portion
    else if (last_jnt_pos_msg_time + rt_msg_timeout > ros::Time::now()) // checking if a joint position message has been published and if it is within timeout
    {
        if (!jnt_pos_status)
        {
            jp_type jp_start = wam.getJointPositions();
            jp_track.setValue(jp_start); // setting initial the joint position command
            jp_rl.setLimit(rt_jp_rl);
            systems::forceConnect(jp_track.output, jp_rl.input);
            wam.trackReferenceSignal(jp_rl.output); // command the WAM to track the RT commanded up to (500 Hz) joint positions
        }
        else if (new_rt_cmd)
        {
            jp_track.setValue(rt_jp_cmd); // set our joint position to subscribed command
            jp_rl.setLimit(rt_jp_rl); // set our rate limit to subscribed rate to control the rate of the moves
        }
        jnt_pos_status = true;
        new_rt_cmd = false;
    }
    /**********************************/
    //Real-Time Joint Hant Tool Control Portion
    else if (last_jnt_hand_tool_msg_time + rt_msg_timeout > ros::Time::now()) // checking if a joint position message has been published and if it is within timeout
    {
        if (!jnt_hand_tool_status)
        {
        }
        else if (new_rt_cmd)
        {
            ROS_INFO("setting exposet outputhandtool");
            exposedOutputHandTool.setValue(rt_hand_tool_cmd);//CP: set hand tool command
            //jp_track.setValue(rt_hand_tool_cmd);
        }
        ROS_INFO("setting jnt_hand_tool_status to true");
        jnt_hand_tool_status = true;
        new_rt_cmd = false;
    }
    /*********************************CP*/
    //Real-Time Cartesian Position Control Portion
    else if (last_cart_pos_msg_time + rt_msg_timeout > ros::Time::now()) // checking if a cartesian position message has been published and if it is within timeout
    {
        if (!cart_pos_status)
        {
            cp_track.setValue(wam.getToolPosition());
            current_ortn.setValue(wam.getToolOrientation()); // Initializing the orientation
            cp_rl.setLimit(rt_cp_rl);
            systems::forceConnect(cp_track.output, cp_rl.input);
            systems::forceConnect(cp_rl.output, rt_pose_cmd.getInput<0>()); // saving the rate limited cartesian position command to the pose.position
            systems::forceConnect(current_ortn.output, rt_pose_cmd.getInput<1>()); // saving the original orientation to the pose.orientation
            wam.trackReferenceSignal(rt_pose_cmd.output); //Commanding the WAM to track the real-time pose command.
        }
        else if (new_rt_cmd)
        {
            cp_track.setValue(rt_cp_cmd); // Set our cartesian positions to subscribed command
            cp_rl.setLimit(rt_cp_rl); // Updating the rate limit to subscribed rate to control the rate of the moves
        }
        cart_pos_status = true;
        new_rt_cmd = false;
    }
    //If we fall out of 'Real-Time', hold joint positions
    else if (cart_vel_status | ortn_vel_status | jnt_vel_status | jnt_pos_status | jnt_hand_tool_status | cart_pos_status)
    {
        if(jnt_hand_tool_status)
        {
        }
        else
        {
            wam.moveTo(wam.getJointPositions()); // Holds current joint positions upon a RT message timeout
            cart_vel_status = ortn_vel_status = jnt_vel_status = jnt_pos_status=jnt_hand_tool_status = cart_pos_status = ortn_pos_status = false;
            ROS_WARN_STREAM("Fell out of RT control, holding joint positions");
        }
    }
}

//wam_main Function
template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam)
{
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    ros::init(argc, argv, "wam_bringup");
    WamBringup<DOF> wam_bringup(wam);
    wam_bringup.init(pm);
    ROS_INFO_STREAM("Done init");
    ros::Rate pub_rate(PUBLISH_FREQ);
    boost::thread handPubThread(&WamBringup<DOF>::publishHand, &wam_bringup);
    // }
    while (ros::ok() && pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE)
    {
        ros::spinOnce();
        wam_bringup.publishWam(pm);
        wam_bringup.updateRT(pm);
        pub_rate.sleep();
    }
    // wam_bringup.port.close();
    return 0;
}
