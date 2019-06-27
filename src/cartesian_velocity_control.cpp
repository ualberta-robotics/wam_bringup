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

File: cartesian_velocity_control.cpp
Date: 12/06/18
Author: Laura Petrich 
*/

#include "wam_bringup/cartesian_velocity_control.h"
#include "wam_bringup/bhand.h"

// Templated Initialization Function
template<size_t DOF>
void CartesianVelocityControl<DOF>::init(ProductManager& pm)
{
    update_gains = false;
    cv_realtime_status = false;
    cf_realtime_status = false;
    ortn_realtime_status = false;
    locked_orientation = false;
    locked_joints = false;
    new_cf_command = false;
    new_cv_command = false;
    new_ortn_command = false;
    cf_magnitude = SPEED;
    cv_magnitude = 0.0;
    msg_timeout.fromSec(0.3);
    mypm = &pm;
    pm.getExecutionManager()->startManaging(ramp); // starting ramp manager
    ROS_INFO("%zu-DOF WAM", DOF);
    jp_home = wam.getJointPositions();
    wam.gravityCompensate(true); // gravity compensation default set to true
    // setting up WAM joint state publisher
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
    // ros services
    go_home_srv = n_.advertiseService("go_home", &CartesianVelocityControl::go_home_callback, this);
    joint_move_block_srv = n_.advertiseService("joint_move_block", &CartesianVelocityControl::joint_move_block_callback, this);
    hold_joint_pos_srv = n_.advertiseService("hold_joint_pos", &CartesianVelocityControl::hold_joint_pos_callback, this);
    hold_ortn_srv = n_.advertiseService("hold_ortn", &CartesianVelocityControl::hold_tool_ortn_callback, this);
    set_ortn_gains_srv = n_.advertiseService("set_ortn_gains", &CartesianVelocityControl::set_ortn_gains_callback, this);
    connect_force_srv = n_.advertiseService("connect_force_orientation", &CartesianVelocityControl::connect_force_orientation_callback, this);
    connect_vel_srv = n_.advertiseService("connect_velocity", &CartesianVelocityControl::connect_velocity_callback, this);
    // ros publishers
    wam_joint_state_pub = n_.advertise < sensor_msgs::JointState > ("joint_states", 1);
    wam_pose_pub = n_.advertise < geometry_msgs::PoseStamped > ("pose", 1);
    wam_jacobian_mn_pub =n_.advertise < wam_msgs::MatrixMN > ("jacobian",1);
    wam_tool_pub =n_.advertise < wam_msgs::RTToolInfo > ("tool_info",1);
    // ros subscribers
    cart_force_sub = n_.subscribe("cart_vel_cmd", 1, &CartesianVelocityControl::cart_force_callback, this);
    cart_vel_sub = n_.subscribe("cartesian_velocity_control", 1, &CartesianVelocityControl::cart_velocity_callback, this);
    ortn_sub = n_.subscribe("orientation_control", 1, &CartesianVelocityControl::orientation_callback, this);
    
    ROS_INFO("wam services now advertised");
    ros::AsyncSpinner spinner(0);
    spinner.start();
}


// goHome Function for sending the WAM safely back to its home starting position.
template<size_t DOF>
bool CartesianVelocityControl<DOF>::go_home_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Returning to Home Position");
    for (size_t i = 0; i < DOF; i++) {
        jp_cmd[i] = 0.0;
    }
    wam.moveTo(jp_cmd, true);
    jp_home[3] -= 0.3;
    wam.moveTo(jp_home, true);
    jp_home[3] += 0.3;
    wam.moveTo(jp_home, true);
    locked_joints = true;
    return true;
}

//Function to hold WAM Joint Positions
template<size_t DOF>
bool CartesianVelocityControl<DOF>::hold_joint_pos_callback(wam_srvs::Hold::Request &req, wam_srvs::Hold::Response &res)
{
    ROS_INFO("Joint Position Hold request: %s", (req.hold) ? "true" : "false");
    if (req.hold) {
        wam.moveTo(wam.getJointPositions());
        locked_joints = true;
    } else {
        wam.idle();
        locked_joints = false;
    }
    return true;
}

//Function to hold WAM end effector Orientation 2
template<size_t DOF>
bool CartesianVelocityControl<DOF>::hold_tool_ortn_callback(wam_srvs::Hold::Request &req, wam_srvs::Hold::Response &res)
{
    ROS_INFO("Tool Orientation Hold request: %s", (req.hold) ? "true" : "false");
    if (req.hold)
    {
        orientationSetPoint.setValue(wam.getToolOrientation());
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
        locked_orientation = true;
        /////////////////////////////////////////////////
    } else {
        // toolOrntController.setKp(0.0);
        // toolOrntController.setKd(0.0);
        disconnect_systems();
    }
    return true;
}

//Function to command an orientation with varaible gains
template<size_t DOF>
bool CartesianVelocityControl<DOF>::set_ortn_gains_callback(wam_srvs::OrtnGains::Request &req, wam_srvs::OrtnGains::Response &res)
{
    ROS_INFO("Orientation gains set request");
    cp_type OrnKp , OrnKd ;
    OrnKp[0]=req.kp_gain[0]; //rot-X-tool
    OrnKp[1]=req.kp_gain[1]; //rot-Y-tool
    OrnKp[2]=req.kp_gain[2]; //rot-Z-tool
    //In the kD we used the angular velocity that's why the order is different
    OrnKd[0]=req.kd_gain[2];//rot-Z-tool
    OrnKd[1]=req.kd_gain[1];//rot-Y-tool
    OrnKd[2]=req.kd_gain[0]; //rot-X-tool
    KpOrnSet.setValue(OrnKp);
    KdOrnSet.setValue(OrnKd);
    return true;
}

//Function to command an orientation with varaible gains
template<size_t DOF>
bool CartesianVelocityControl<DOF>::connect_force_orientation_callback(wam_srvs::Connect::Request &req, wam_srvs::Connect::Response &res)
{
    ROS_INFO("Connect force request: %s", (req.connect) ? "true" : "false");
    if (req.connect) {
        // set up orientation
        cp_type OrnKp , OrnKd ;
        OrnKp << 3.5, 3.5, 0.0;
        OrnKd << 0.02, 0.02, 0.02;
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
        // set up force
        exposedOutputForce.setValue(cf_type(0.0, 0.0, 0.0));
        exposedOutputTorque.setValue(ct_type(0.0, 0.0, 0.0));
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
        ramp.setSlope(cf_magnitude); // setting the slope to the commanded magnitude
        ramp.stop(); // ramp is stopped on startup
        ramp.setOutput(0.0); // ramp is re-zeroed on startup
        ramp.start(); // start the ramp
        cf_realtime_status = true;
        ROS_INFO_STREAM("realtime_status set to true");
    } else {
        disconnect_systems();
    }
    return true;
}

//Function to command an orientation with varaible gains
template<size_t DOF>
bool CartesianVelocityControl<DOF>::connect_velocity_callback(wam_srvs::Connect::Request &req, wam_srvs::Connect::Response &res)
{
    ROS_INFO("Connect force request: %s", (req.connect) ? "true" : "false");
    if (req.connect) {
        // set up orientation
        cp_type OrnKp , OrnKd ;
        OrnKp << 3.5, 3.5, 0.0;
        OrnKd << 0.02, 0.02, 0.02;
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
        // set up velocity controller, good default values:  v_magnitude=0.1, kp=75
        cp_type v_dir;
        v_dir << 0.0, 0.0, 0.0;
        // VelocityComp.setKp(75.0); // default
        VelocityComp.setKp(150.0);
        exposedDesiredVel.setValue(0.1);
        exposedDirVel.setValue(v_dir);
        systems::forceConnect(wam.kinematicsBase.kinOutput, tf2jt_cartVel.kinInput);
        systems::forceConnect(exposedDirVel.output, computeTangentVel.tangentDirInput);
        systems::forceConnect(wam.toolVelocity.output, computeTangentVel.toolVelocityInput);
        systems::forceConnect(computeTangentVel.tangentVelOutput, VelocityComp.feedbackInput);
        systems::forceConnect(exposedDesiredVel.output, VelocityComp.referenceInput);
        systems::forceConnect(exposedDirVel.output, tg2.getInput<0>());
        systems::forceConnect(VelocityComp.controlOutput, tg2.getInput<1>());
        systems::forceConnect(tg2.output, mult.input);
        systems::forceConnect(mult.output, tf2jt_cartVel.input);
        // connect all to summer
        systems::forceConnect(tf2jt_cartVel.output, torqueSum.getInput(0));
        systems::forceConnect(tt2jt_ortn_split.output, torqueSum.getInput(1));
        // saturate and send to wam input
        systems::forceConnect(torqueSum.output, jtSat.input);
        systems::forceConnect(jtSat.output, wam.input);
        cv_realtime_status = true;
        ortn_realtime_status = true;
        ROS_INFO_STREAM("cartesian velocity: realtime_status set to true");
    } else {
        disconnect_systems();
    }
    return true;
}

//Function to command a joint space move to the WAM with blocking specified
template<size_t DOF>
bool CartesianVelocityControl<DOF>::joint_move_block_callback(wam_srvs::JointMoveBlock::Request &req, wam_srvs::JointMoveBlock::Response &res)
{
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

//Callback function for RT Cartesian Velocity messages
template<size_t DOF>
void CartesianVelocityControl<DOF>::cart_force_callback(const wam_msgs::RTCartVel::ConstPtr& msg)
{
    // ROS_INFO_STREAM("Reached velocity_callback");
    if (cf_realtime_status) {
        for (size_t i = 0; i < 3; i++) {
            force_applied[i] = msg->direction[i];
        } if (msg->magnitude != 0) {
            cf_magnitude = msg->magnitude;
        }
        new_cf_command = true;
    }
    last_cf_msg_time = ros::Time::now();
}

//Callback function for RT Cartesian Velocity messages
template<size_t DOF>
void CartesianVelocityControl<DOF>::cart_velocity_callback(const wam_msgs::RTVelocity::ConstPtr& msg)
{
    // ROS_INFO_STREAM("Reached velocity_callback");
    if (cv_realtime_status) {
        for (size_t i = 0; i < 3; i++) {
            cv_direction[i] = msg->v_direction[i];
        }
        cv_magnitude = msg->v_magnitude;
        if (msg->change_gains) {
            cv_kp = msg->kp;
            update_gains = true;
        }
        new_cv_command = true;
    }
    last_cv_msg_time = ros::Time::now();
}

//Callback function for RT Orientation messages
template<size_t DOF>
void CartesianVelocityControl<DOF>::orientation_callback(const wam_msgs::RTOrtn::ConstPtr& msg)
{
    // ROS_INFO_STREAM("Reached velocity_callback");
    if (ortn_realtime_status) {
        Eigen::Quaterniond q, qd;
        // q = wam.kinInput.getValue().impl->tool_velocity_angular;
        qd.x() = msg->orientation[0];
        qd.y() = msg->orientation[1];
        qd.z() = msg->orientation[2];
        qd.w() = msg->orientation[3];
        if (msg->change_gains) {
            for (size_t i = 0; i < 3; i++) {
                OrtnKp[i] = msg->kp[i];
            }
            for (size_t i = 0; i < 3; i++) {
                OrtnKd[i] = msg->kd[i];
            }
            update_gains = true;
        }
        new_ortn_command = true;
    }
    last_ortn_msg_time = ros::Time::now();
}

template<size_t DOF>
void CartesianVelocityControl<DOF>::disconnect_systems()
{
    systems::disconnect(wam.input);
    locked_orientation = false;
    cv_realtime_status = false;
    cf_realtime_status = false;
    ortn_realtime_status = false;
    ROS_INFO("All systems disconnected");
}

//Function to update the real-time control loops
template<size_t DOF>
void CartesianVelocityControl<DOF>::update_realtime(ProductManager& pm) 
{
    if (last_cf_msg_time + msg_timeout > ros::Time::now()) {
        if (cf_realtime_status) {
            ramp.reset(); // reset the ramp to 0
            ramp.setSlope(cf_magnitude);
            exposedOutputForce.setValue(force_applied);
            // exposedOutputTorque.setValue(ct_type(0.0, 0.0, 0.0));
            new_cf_command = false;
        }
    }
    if (last_cv_msg_time + msg_timeout > ros::Time::now()) {
        if (cv_realtime_status) {
            if (update_gains) {
                VelocityComp.setKp(cv_kp);
                update_gains = false;
            }
            // ROS_INFO_STREAM("REACHED HERE");
            exposedDesiredVel.setValue(cv_magnitude);
            exposedDirVel.setValue(cv_direction);
            new_cv_command = false;
        }
    }
    if (last_ortn_msg_time + msg_timeout > ros::Time::now()) {
        if (ortn_realtime_status) {
            if (update_gains) {
                KpOrnSet.setValue(OrtnKp);
                KdOrnSet.setValue(OrtnKd);                
                update_gains = false;
            }
            orientationSetPoint.setValue(ortn_direction);
            new_ortn_command = false;
        }
    }
}

//Function to update the WAM publisher
template<size_t DOF>
void CartesianVelocityControl<DOF>::publish_wam(ProductManager& pm)
{
    //Current values to be published
    jp_type jp = wam.getJointPositions();
    jt_type jt = wam.getJointTorques();
    jv_type jv = wam.getJointVelocities();
    cp_type cp_pub = wam.getToolPosition();
    cv_type cv_pub = wam.getToolVelocity();
    cv_type cv_ang = wam.getToolAngularVelocity();
    // std::cout << cv_ang[0] << " " << cv_ang[1] << " " << cv_ang[2] << std::endl;
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
    for (size_t h = 0; h < wam_jacobian_mn.n; ++h) {
        for (size_t k = 0; k < wam_jacobian_mn.m; ++k) {
            wam_jacobian_mn.data[h*6+k]=robot_tool_jacobian(k,h);
        }
    }
    wam_jacobian_mn_pub.publish(wam_jacobian_mn);

    for (size_t j = 0; j < 3; j++) {
        wam_tool_info.position[j] = cp_pub[j];
        wam_tool_info.velocity[j] = cv_pub[j];
    }
    wam_tool_pub.publish(wam_tool_info);
}

//wam_main Function
template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam)
{
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    ros::init(argc, argv, "cartesian_velocity_control");
    BHand bhand;
    bhand.init();
    CartesianVelocityControl<DOF> cvc(wam);
    cvc.init(pm);
    ROS_INFO_STREAM("Finished initializing hand and arm");
    ros::Rate pub_rate(PUBLISH_FREQ);
    while (ros::ok() && pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
        ros::spinOnce();
        try {
            bhand.publish();
            cvc.publish_wam(pm);
            cvc.update_realtime(pm);
        } catch (...) {
            ROS_WARN_STREAM("Exception occured");
        }
        pub_rate.sleep();
    }
    bhand.port.close();
    return 0;
}