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

    File: wam_node.cpp
    Date: 5 June, 2012
    Author: Kyle Maroney
    Refactored by: Laura Petrich 17/06/19
*/

#include "wam_bringup/wam_node.h"

// Templated Initialization Function
template<size_t DOF>
void WamNode<DOF>::init(ProductManager& pm) {
    mm = NULL;
    vhp = NULL;
    //Setting up real-time command timeouts and initial values
    rt_msg_timeout.fromSec(0.3); //rt_status will be determined false if rt message is not received in specified time
    cart_vel_mag = SPEED; //Setting default cartesian velocity magnitude to SPEED
    ortn_vel_mag = SPEED;
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

    max_base_torque << 0.0, 0.0, 0.0;
    max_base_force << 6.0, 6.0, 6.0;
    max_tool_torque << -0.5, -0.3, 0.2;
    max_tool_force << -4.0, 5.0, 6.0;

    mypm = &pm;
    pm.getExecutionManager()->startManaging(ramp); // starting ramp manager
    ROS_INFO("%zu-DOF WAM", DOF);
    jp_home = wam.getJointPositions();
    SpringSetPoint << wam.getToolPosition();
    OrnSpringSetPoint = wam.getToolOrientation(); 
    wam.gravityCompensate(true); 
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
    // cart_vel_sub = n_.subscribe("cart_vel_cmd", 1, &WamNode::cartVelCB, this);
    // ortn_vel_sub = n_.subscribe("ortn_vel_cmd", 1, &WamNode::ortnVelCB, this);
    // jnt_vel_sub = n_.subscribe("jnt_vel_cmd", 1, &WamNode::jntVelCB, this);
    // jnt_pos_sub = n_.subscribe("jnt_pos_cmd", 1, &WamNode::jntPosCB, this);
    // jnt_hand_tool_sub = n_.subscribe("jnt_hand_tool_cmd", 1, &WamNode::jntHandToolCB, this);
    // cart_pos_sub = n_.subscribe("cart_pos_cmd", 1, &WamNode::cartPosCB, this);
    // vs_error = n_.subscribe("v_err", 1, &WamNode::vsErrCB, this);
    //Advertising the following rosservices

    disconnect_systems_srv = n_.advertiseService("disconnect_systems", &WamNode::disconnectSystems, this);
    joy_ft_base_srv = n_.advertiseService("joy_force_torque_base", &WamNode::joyForceTorqueBase, this);
    joy_ft_tool_srv = n_.advertiseService("joy_force_torque_tool", &WamNode::joyForceTorqueTool, this);


    gravity_srv = n_.advertiseService("gravity_comp", &WamNode::gravity, this);
    go_home_srv = n_.advertiseService("go_home", &WamNode::goHome, this);

    haptic_sphere_srv = n_.advertiseService("haptic_sphere", &WamNode::hapticSphere, this);
    // LP control experiments
    // jp_pid_srv = n_.advertiseService("jp_pid_control", &WamNode::jpPIDControl,this);
    // jv_pid_srv = n_.advertiseService("jv_pid_control", &WamNode::jvPIDControl,this);
    // tp_pid_srv = n_.advertiseService("tp_pid_control", &WamNode::tpPIDControl,this);
    // force_torque_tool_time_srv = n_.advertiseService("force_torque_tool_time", &WamNode::forceTorqueToolTime,this);
    // force_torque_base_time_srv = n_.advertiseService("force_torque_base_time", &WamNode::forceTorqueBaseTime,this);

    hold_jpos_srv = n_.advertiseService("hold_joint_pos", &WamNode::holdJPos, this);
    hold_cpos_srv = n_.advertiseService("hold_cart_pos", &WamNode::holdCPos, this);
    hold_ortn_srv = n_.advertiseService("hold_ortn", &WamNode::holdOrtn, this);
    // hold_ortn2_srv = n_.advertiseService("hold_ortn2", &WamNode::holdOrtn2, this);
    joint_move_srv = n_.advertiseService("joint_move", &WamNode::jointMove, this);
    joint_move_block_srv = n_.advertiseService("joint_move_block", &WamNode::jointMoveBlock, this);
    // pose_move_srv = n_.advertiseService("pose_move", &WamNode::poseMove, this);
    // cart_move_srv = n_.advertiseService("cart_move", &WamNode::cartMove, this);
    // cart_vel_srv = n_.advertiseService("cart_vel", &WamNode::cartVel, this);
    // ortn_move_srv = n_.advertiseService("ortn_move", &WamNode::ortnMove, this);
    // ortn_split_move_srv = n_.advertiseService("ortn_split_move", &WamNode::ortnSplitMove, this);
    // force_torque_base_srv = n_.advertiseService("force_torque_base", &WamNode::forceTorqueBase, this);
    // force_torque_tool_srv = n_.advertiseService("force_torque_tool", &WamNode::forceTorqueTool, this);
    teach_srv = n_.advertiseService("teach_motion", &WamNode::teachMotion, this);
    play_srv = n_.advertiseService("play_motion", &WamNode::playMotion, this);
    link_arm_srv = n_.advertiseService("link_arm", &WamNode::linkArm, this);
    unlink_arm_srv = n_.advertiseService("unlink_arm", &WamNode::unLinkArm, this);
    // start_visual_fix = n_.advertiseService("start_visual_fix", &WamNode::startVisualFix, this);
    // stop_visual_fix = n_.advertiseService("stop_visual_fix", &WamNode::stopVisualFix, this);
    // follow_path_srv = n_.advertiseService("follow_path", &WamNode::followPath,this);
    ROS_INFO("wam services now advertised");
}

template<size_t DOF>
void WamNode<DOF>::disconnectSystems() {
    if (systems_connected) {
        systems::disconnect(wam.input);
        systems_connected = false;
        ROS_INFO("systems disconnected");
    } else {
        ROS_INFO("systems already disconnected");
    }
    return;
}

template<size_t DOF>
bool WamNode<DOF>::disconnectSystems(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    disconnectSystems();
    return true;
}

// haptic sphere function, creates a sphere with requested radius, kp and kd values
// lpetrich 2018
template<size_t DOF>
bool WamNode<DOF>::hapticSphere(wam_srvs::HapticSphere::Request &req, wam_srvs::HapticSphere::Response &res) {
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
}

// gravity_comp service callback
template<size_t DOF>
bool WamNode<DOF>::gravity(wam_srvs::GravityComp::Request &req, wam_srvs::GravityComp::Response &res) {
    wam.gravityCompensate(req.gravity);
    ROS_INFO("Gravity Compensation Request: %s", (req.gravity) ? "true" : "false");
    return true;
}

// goHome Function for sending the WAM safely back to its home starting position.
template<size_t DOF>
bool WamNode<DOF>::goHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    ROS_INFO("Returning to Home Position");
    for (size_t i = 0; i < DOF; i++) {
        jp_cmd[i] = 0.0;
    }
    wam.moveTo(jp_cmd, true);
    jp_home[3] -= 0.3;
    wam.moveTo(jp_home, true);
    jp_home[3] += 0.3;
    wam.moveTo(jp_home, true);
    return true;
}

// Function to hold WAM Joint Positions
template<size_t DOF>
bool WamNode<DOF>::holdJPos(wam_srvs::Hold::Request &req, wam_srvs::Hold::Response &res) {
    ROS_INFO("Joint Position Hold request: %s", (req.hold) ? "true" : "false");
    if (req.hold) {
        wam.moveTo(wam.getJointPositions());
    } else {
        wam.idle();
    }
    return true;
}

// Function to hold WAM end effector Cartesian Position
// lpetrich 06/2019
template<size_t DOF>
bool WamNode<DOF>::holdCPos(wam_srvs::HoldGains::Request &req, wam_srvs::HoldGains::Response &res) {
    ROS_INFO("Cartesian Position Hold request: %s", (req.hold) ? "true" : "false");
    jt_type jtLimits(30.0);
    cp_type KpApplied;
    cp_type KdApplied;
    KpApplied << req.kp[0], req.kp[1], req.kp[2];
    KdApplied << req.kd[0], req.kd[1], req.kd[2];
    if (req.hold) {
        disconnectSystems();
        KxSet.setValue(KpApplied);
        DxSet.setValue(KdApplied);
        XdSet.setValue(wam.getToolPosition());
        OrnKxSet.setValue(cp_type(0.0, 0.0, 0.0));
        OrnDxSet.setValue(cp_type(0.0, 0.0, 0.0));
        OrnXdSet.setValue(wam.getToolOrientation());
        // CONNECT SPRING SYSTEM
        systems::forceConnect(KxSet.output, ImpControl.KxInput);
        systems::forceConnect(DxSet.output, ImpControl.DxInput);
        systems::forceConnect(XdSet.output, ImpControl.XdInput);

        systems::forceConnect(OrnKxSet.output, ImpControl.OrnKpGains);
        systems::forceConnect(OrnDxSet.output, ImpControl.OrnKdGains);
        systems::forceConnect(OrnXdSet.output, ImpControl.OrnReferenceInput);
        systems::forceConnect(wam.toolOrientation.output, ImpControl.OrnFeedbackInput);

        systems::forceConnect(wam.toolPosition.output, ImpControl.CpInput);
        systems::forceConnect(wam.toolVelocity.output, ImpControl.CvInput);
        systems::forceConnect(wam.kinematicsBase.kinOutput, ImpControl.kinInput);

        systems::forceConnect(wam.kinematicsBase.kinOutput, toolforce2jt.kinInput);
        systems::forceConnect(wam.kinematicsBase.kinOutput, tooltorque2jt.kinInput);

        systems::forceConnect(ImpControl.CFOutput, toolforce2jt.input);
        systems::forceConnect(ImpControl.CTOutput, tt2jt_ortn_split.input);
        systems::forceConnect(wam.kinematicsBase.kinOutput, tt2jt_ortn_split.kinInput);
        // CONNECT TO SUMMER
        systems::forceConnect(toolforce2jt.output, torqueSum.getInput(0));
        systems::forceConnect(tt2jt_ortn_split.output, torqueSum.getInput(1));
        // SATURATE AND CONNECT TO WAM INPUT
        systems::forceConnect(torqueSum.output, jtSat.input);        
        systems::forceConnect(jtSat.output, wam.input); 
    } else {
        systems::disconnect(wam.input);
        wam.idle();
    }
    return true;
}

// Function to hold WAM end effector Orientation
// lpetrich 06/2019
template<size_t DOF>
bool WamNode<DOF>::holdOrtn(wam_srvs::HoldGains::Request &req, wam_srvs::HoldGains::Response &res) {
    ROS_INFO("Orientation Hold request: %s", (req.hold) ? "true" : "false");
    if (req.hold) {
        cp_type OrnKp, OrnKd;
        // In Kd we used the angular velocity that's why the order is different
        OrnKp << req.kp[0], req.kp[1], req.kp[2];
        OrnKd << req.kd[2], req.kd[1], req.kd[0];
        orientationSetPoint.setValue(wam.getToolOrientation());
        KpOrnSet.setValue(OrnKp);
        KdOrnSet.setValue(OrnKd);
        systems::forceConnect(KpOrnSet.output, OrtnSplitCont.KpGains);
        systems::forceConnect(KdOrnSet.output, OrtnSplitCont.KdGains);
        systems::forceConnect(wam.toolOrientation.output, OrtnSplitCont.FeedbackOrnInput);
        systems::forceConnect(orientationSetPoint.output, OrtnSplitCont.ReferenceOrnInput);
        systems::forceConnect(wam.kinematicsBase.kinOutput, OrtnSplitCont.kinInput);
        systems::forceConnect(wam.kinematicsBase.kinOutput, tt2jt_ortn_split.kinInput);
        systems::forceConnect(OrtnSplitCont.CTOutput, tt2jt_ortn_split.input);
        systems::forceConnect(tt2jt_ortn_split.output, jtSat_ornSplit.input);
        systems::forceConnect(jtSat_ornSplit.output, wam.input);
    } else {
        systems::disconnect(wam.input);
        wam.idle();
    }
    return true;
}

// Function to hold WAM end effector Orientation 2
template<size_t DOF>
bool WamNode<DOF>::holdOrtn2(wam_srvs::Hold::Request &req, wam_srvs::Hold::Response &res) {
    ROS_INFO("Orientation Hold request: %s", (req.hold) ? "true" : "false");
    if (req.hold) {
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
    } else {
        systems::disconnect(wam.input);
        wam.idle();
    }
    return true;
}

// Function to command a joint space move to the WAM
template<size_t DOF>
bool WamNode<DOF>::jointMove(wam_srvs::JointMove::Request &req, wam_srvs::JointMove::Response &res) {
    if (req.joints.size() != DOF) {
        ROS_INFO("Request Failed: %zu-DOF request received, must be %zu-DOF", req.joints.size(), DOF);
        return false;
    }
    ROS_INFO("Moving Robot to Commanded Joint Pose");
    for (size_t i = 0; i < DOF; i++) {
        jp_cmd[i] = req.joints[i];
    }
    // wam.moveTo(jp_cmd, false);
    wam.moveTo(jp_cmd, true); // set second argument to true if you want to complete movement before returning true
    return true;
}
//boop
// Function to command a joint space move to the WAM with blocking specified
// lpetrich 2018
template<size_t DOF>
bool WamNode<DOF>::jointMoveBlock(wam_srvs::JointMoveBlock::Request &req, wam_srvs::JointMoveBlock::Response &res) {
    if (req.joints.size() != DOF) {
        ROS_INFO("Request Failed: %zu-DOF request received, must be %zu-DOF", req.joints.size(), DOF);
        return false;
    }
    ROS_INFO("Moving Robot to Commanded Joint Pose");
    for (size_t i = 0; i < DOF; i++) {
        jp_cmd[i] = req.joints[i];
    }
    bool b = req.blocking;
    wam.moveTo(jp_cmd, b);
    return true;
}

//Function to command an orientation with varaible gains
template<size_t DOF>
bool WamNode<DOF>::ortnSplitMove(wam_srvs::OrtnSplitMove::Request &req, wam_srvs::OrtnSplitMove::Response &res) {
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

// lpetrich works 06/2019
template<size_t DOF>
bool WamNode<DOF>::joyForceTorqueBase(wam_srvs::ForceTorque::Request &req, wam_srvs::ForceTorque::Response &res) {
    jt_type jtLimits(30.0);
    cf_type ForceApplied;
    ct_type TorqueApplied;
    cp_type KpApplied;
    cp_type KdApplied;
    cp_type OrnKpApplied; 
    cp_type OrnKdApplied;
    Eigen::Quaterniond OrnSetPoint; 

    ForceApplied << req.force[0] * max_base_force[0], req.force[1] * max_base_force[1], req.force[2] * max_base_force[2];
    TorqueApplied << req.torque[0] * max_base_torque[0], req.torque[1] * max_base_torque[1], req.torque[2] * max_base_torque[2];
    KpApplied << req.kp[0], req.kp[1], req.kp[2];
    KdApplied << req.kd[0], req.kd[1], req.kd[2];

    if (req.initialize) { // CONNECT SYSTEMS
        disconnectSystems();
        // SPRING SYSTEM VALUES
        // POSITIONAL SPRING
        SpringSetPoint << wam.getToolPosition(); // e.g. position at [0,0,0,-pi/2,0,0,0] is [0.4 , 0 , 0.6]
        KxSet.setValue(KpApplied);
        DxSet.setValue(KdApplied);
        XdSet.setValue(SpringSetPoint);
        // ORIENTATION SPRING
        OrnKpApplied << 1.0, 1.0, 1.0;       
        OrnKdApplied << 0.02, 0.02, 0.02;      
        OrnSetPoint = wam.getToolOrientation(); 
        OrnKxSet.setValue(OrnKpApplied);
        OrnDxSet.setValue(OrnKdApplied);
        OrnXdSet.setValue(OrnSetPoint);
        // CONNECT SPRING SYSTEM
        systems::forceConnect(KxSet.output, ImpControl.KxInput);
        systems::forceConnect(DxSet.output, ImpControl.DxInput);
        systems::forceConnect(XdSet.output, ImpControl.XdInput);

        systems::forceConnect(OrnKxSet.output, ImpControl.OrnKpGains);
        systems::forceConnect(OrnDxSet.output, ImpControl.OrnKdGains);
        systems::forceConnect(OrnXdSet.output, ImpControl.OrnReferenceInput);
        systems::forceConnect(wam.toolOrientation.output, ImpControl.OrnFeedbackInput);

        systems::forceConnect(wam.toolPosition.output, ImpControl.CpInput);
        systems::forceConnect(wam.toolVelocity.output, ImpControl.CvInput);
        systems::forceConnect(wam.kinematicsBase.kinOutput, ImpControl.kinInput);

        systems::forceConnect(wam.kinematicsBase.kinOutput, toolforce2jt.kinInput);
        systems::forceConnect(wam.kinematicsBase.kinOutput, tooltorque2jt.kinInput);

        systems::forceConnect(ImpControl.CFOutput, toolforce2jt.input);
        systems::forceConnect(ImpControl.CTOutput, tt2jt_ortn_split.input);
        systems::forceConnect(wam.kinematicsBase.kinOutput, tt2jt_ortn_split.kinInput);
        // FORCE TORQUE VALUES
        exposedOutputForce.setValue(ForceApplied);
        exposedOutputTorque.setValue(TorqueApplied);
        // CONNECT FORCE TORQUE SYSTEMS
        systems::forceConnect(exposedOutputForce.output, toolforce2jt2.input);
        systems::forceConnect(exposedOutputTorque.output, tooltorque2jt2.input);
        systems::forceConnect(wam.kinematicsBase.kinOutput, toolforce2jt2.kinInput);
        systems::forceConnect(wam.kinematicsBase.kinOutput, tooltorque2jt2.kinInput);
        // CONNECT ALL TO SUMMER
        systems::forceConnect(toolforce2jt.output, torqueSum.getInput(0));
        systems::forceConnect(tt2jt_ortn_split.output, torqueSum.getInput(1));
        systems::forceConnect(toolforce2jt2.output, torqueSum2.getInput(0));
        systems::forceConnect(tooltorque2jt2.output, torqueSum2.getInput(1));
        systems::forceConnect(torqueSum.output, torqueSum3.getInput(1));
        systems::forceConnect(torqueSum2.output, torqueSum3.getInput(0));
        // SATURATE AND CONNECT TO WAM INPUT
        systems::forceConnect(torqueSum3.output, jtSat.input);        
        systems::forceConnect(jtSat.output, wam.input); 
        ROS_INFO("force torque base initialized: ");
        std::cout << "\t\t\t\tforce [" << ForceApplied[0] << " " << ForceApplied[1] << " " << ForceApplied[2] <<
            "] torque [" << TorqueApplied[0] << " " << TorqueApplied[1] << " " << TorqueApplied[2] <<
            "] kp [" << KpApplied[0] << " " << KpApplied[1] << " " << KpApplied[2] <<
            "] kd [" << KdApplied[0] << " " << KdApplied[1] << " " << KdApplied[2] << 
            "]" << std::endl;
    } else if (!systems_connected) { 
        // MAKE SURE SYSTEMS ARE CONNECTED
        ROS_WARN_STREAM("ERROR: JOYFTB SYSTEMS NOT CONNECTED");
        return true;
    } else { 
        // UPDATE VALUES
        // orientationSetPoint.setValue(wam.getToolOrientation());
        cp_type tp = wam.getToolPosition();
        if (!ForceApplied[0]) {
            // std::cout << "X ZERO" << std::endl;
            tp[0] = SpringSetPoint[0];
        } 
        if (!ForceApplied[1]) {
            // std::cout << "Y ZERO" << std::endl;
            tp[1] = SpringSetPoint[1];
        }
        if (!ForceApplied[2]) {
            // std::cout << "Z ZERO" << std::endl;
            tp[2] = SpringSetPoint[2];
        }
        SpringSetPoint = tp;
        XdSet.setValue(SpringSetPoint);
        KxSet.setValue(KpApplied);
        DxSet.setValue(KdApplied);
        exposedOutputForce.setValue(ForceApplied);
        exposedOutputTorque.setValue(TorqueApplied);
        // KpOrnSet.setValue(Kx);
        // KdOrnSet.setValue(Dx);
        // ROS_INFO("force torque cartesian base update: ")
    }
    systems_connected = true;
    return true;
}

// Function to apply a force and torque to the WAM end effector with respect to the tool frame
// lpetrich 06/2019
template<size_t DOF>
bool WamNode<DOF>::joyForceTorqueTool(wam_srvs::ForceTorque::Request &req, wam_srvs::ForceTorque::Response &res) {
    jt_type jtLimits(30.0);
    cf_type ForceApplied;
    ct_type TorqueApplied;
    cp_type KpApplied;
    cp_type KdApplied; 
    cp_type OrnKpApplied; 
    cp_type OrnKdApplied;
    // Eigen::Quaterniond OrnSetPoint; 

    ForceApplied << req.force[0] * max_tool_force[0], req.force[1] * max_tool_force[1], req.force[2] * max_tool_force[2];
    TorqueApplied << req.torque[0] * max_tool_torque[0], req.torque[1] * max_tool_torque[1], req.torque[2] * max_tool_torque[2];
    KpApplied << req.kp[0], req.kp[1], req.kp[2];
    KdApplied << req.kd[0], req.kd[1], req.kd[2];
    OrnKpApplied << 1.5, 1.5, 1.5;       
    OrnKdApplied << 0.02, 0.02, 0.02;

    if (req.initialize) { 
        disconnectSystems();
        // CONNECT SPRING SYSTEM
        SpringSetPoint << wam.getToolPosition();
        KxSet.setValue(KpApplied);
        DxSet.setValue(KdApplied);
        XdSet.setValue(SpringSetPoint);
        // ORIENTATION SPRING
        OrnSpringSetPoint = wam.getToolOrientation(); 
        OrnKxSet.setValue(OrnKpApplied);
        OrnDxSet.setValue(OrnKdApplied);
        OrnXdSet.setValue(OrnSpringSetPoint);
        // CONNECT SPRING SYSTEM
        systems::forceConnect(KxSet.output, ImpControl.KxInput);
        systems::forceConnect(DxSet.output, ImpControl.DxInput);
        systems::forceConnect(XdSet.output, ImpControl.XdInput);

        systems::forceConnect(OrnKxSet.output, ImpControl.OrnKpGains);
        systems::forceConnect(OrnDxSet.output, ImpControl.OrnKdGains);
        systems::forceConnect(OrnXdSet.output, ImpControl.OrnReferenceInput);
        systems::forceConnect(wam.toolOrientation.output, ImpControl.OrnFeedbackInput);

        systems::forceConnect(wam.toolPosition.output, ImpControl.CpInput);
        systems::forceConnect(wam.toolVelocity.output, ImpControl.CvInput);
        systems::forceConnect(wam.kinematicsBase.kinOutput, ImpControl.kinInput);

        systems::forceConnect(wam.kinematicsBase.kinOutput, toolforce2jt.kinInput);
        systems::forceConnect(wam.kinematicsBase.kinOutput, tooltorque2jt.kinInput);

        systems::forceConnect(ImpControl.CFOutput, toolforce2jt.input);
        systems::forceConnect(ImpControl.CTOutput, tt2jt_ortn_split.input);
        systems::forceConnect(wam.kinematicsBase.kinOutput, tt2jt_ortn_split.kinInput);
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
        // CONNECT BOTH TO SUMMER
        systems::forceConnect(toolforce2jt.output, torqueSum.getInput(0));
        systems::forceConnect(tt2jt_ortn_split.output, torqueSum.getInput(1));
        systems::forceConnect(toolforce2jt2.output, torqueSum2.getInput(0));
        systems::forceConnect(tooltorque2jt2.output, torqueSum2.getInput(1));
        systems::forceConnect(torqueSum.output, torqueSum3.getInput(0));
        systems::forceConnect(torqueSum2.output, torqueSum3.getInput(1));
        // SATURATE AND CONNECT TO WAM INPUT
        systems::forceConnect(torqueSum3.output, jtSat.input);
        systems::forceConnect(jtSat.output, wam.input);

        ROS_INFO("force torque tool initialized: ");
        std::cout << "\t\t\t\tforce [" << ForceApplied[0] << " " << ForceApplied[1] << " " << ForceApplied[2] <<
            "] torque [" << TorqueApplied[0] << " " << TorqueApplied[1] << " " << TorqueApplied[2] <<
            "] kp [" << KpApplied[0] << " " << KpApplied[1] << " " << KpApplied[2] <<
            "] kd [" << KdApplied[0] << " " << KdApplied[1] << " " << KdApplied[2] << 
            "]" << std::endl;
    } else if (!systems_connected) { // MAKE SURE SYSTEMS ARE CONNECTED
        ROS_WARN_STREAM("ERROR: JOYFTT SYSTEMS NOT CONNECTED");
        return true;
    } else { // UPDATE
        cp_type tp = wam.getToolPosition();
        bool orient = false;
        for (int i = 0; i < 3; ++i) {
            if (!ForceApplied[i]) {
                tp[i] = SpringSetPoint[i];
            }
            if (TorqueApplied[i]) {
                orient = true;
                OrnSpringSetPoint = wam.getToolOrientation(); 
            }
        }
        if (orient) {
            OrnKpApplied << 0.0, 0.0, 0.0;       
            OrnKdApplied << 0.0, 0.0, 0.0;  
        }
        SpringSetPoint = tp;
        OrnXdSet.setValue(OrnSpringSetPoint);
        OrnKxSet.setValue(OrnKpApplied);
        OrnDxSet.setValue(OrnKdApplied);
        XdSet.setValue(SpringSetPoint);
        KxSet.setValue(KpApplied);
        DxSet.setValue(KdApplied);
        exposedOutputForce.setValue(ForceApplied);
        exposedOutputTorque.setValue(TorqueApplied);
        // std::cout << "\tforce [" << ForceApplied[0] << " " << ForceApplied[1] << " " << ForceApplied[2] <<
        //     "] torque [" << TorqueApplied[0] << " " << TorqueApplied[1] << " " << TorqueApplied[2] <<
        //     "] set point [" << SpringSetPoint[0] << " " << SpringSetPoint[1] << " " << SpringSetPoint[2] <<
        //     "] kp [" << KpApplied[0] << " " << KpApplied[1] << " " << KpApplied[2] <<
        //     "] kd [" << KdApplied[0] << " " << KdApplied[1] << " " << KdApplied[2] << 
        //     "]" << std::endl;
    } 
    systems_connected = true;
    return true;
}

template<size_t DOF>
bool WamNode<DOF>::linkArm(wam_srvs::Link::Request &req, wam_srvs::Link::Response &res) {
    //Setup master
    if (mm == NULL) {
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
    if (mm->isLinked()) {
        ROS_INFO_STREAM("Linked with remote WAM.\n");
    } else {
        ROS_INFO_STREAM("WARNING: Linking was unsuccessful.\n");
    }
    return true;
}

template<size_t DOF>
bool WamNode<DOF>::unLinkArm(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    mm->unlink();
    wam.moveTo(wam.getJointPositions());
}

//Function to play a taught motion to the WAM
template<size_t DOF>
bool WamNode<DOF>::playMotion(wam_srvs::Play::Request &req, wam_srvs::Play::Response &res) {
    // Build spline between recorded points
    std::string path = "/home/robot/motions/" + req.path;
    ROS_INFO_STREAM("Playing back motion from : " << path);
    log::Reader<jp_sample_type> lr(path.c_str());
    std::vector<jp_sample_type> vec;
    for (size_t i = 0; i < lr.numRecords(); ++i) {
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

//Function to teach a motion to the WAM
template<size_t DOF>
bool WamNode<DOF>::teachMotion(wam_srvs::Teach::Request &req, wam_srvs::Teach::Response &res) {
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

//Function to update the WAM publisher
template<size_t DOF>
void WamNode<DOF>::publishWam(ProductManager& pm) {
    //Current values to be published
    jp_type jp = wam.getJointPositions();
    jt_type jt = wam.getJointTorques();
    jv_type jv = wam.getJointVelocities();
    cp_type cp_pub = wam.getToolPosition();
    Eigen::Quaterniond to_pub = wam.getToolOrientation();
    math::Matrix<6,DOF> robot_tool_jacobian=wam.getToolJacobian();
    //publishing sensor_msgs/JointState to wam/joint_states
    for (size_t i = 0; i < DOF; i++) {
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
    for (size_t h = 0; h < wam_jacobian_mn.n; ++h) {
        for (size_t k = 0; k < wam_jacobian_mn.m; ++k) {
            wam_jacobian_mn.data[h*6+k]=robot_tool_jacobian(k,h);
        }
    }
    wam_jacobian_mn_pub.publish(wam_jacobian_mn);
}

//wam_main Function
template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam)
{
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    ros::init(argc, argv, "wam");
    ros::NodeHandle n_;
    WamNode<DOF> wam_node(wam);
    wam_node.init(pm);
    ROS_INFO_STREAM("wam node initialized");
    ros::Rate pub_rate(PUBLISH_FREQ);
    while (ros::ok() && pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
        ros::spinOnce();
        wam_node.publishWam(pm);
        // wam_node.updateRT(pm);
        pub_rate.sleep();
    }
    ROS_INFO_STREAM("wam node shutting down");
    return 0;
}
