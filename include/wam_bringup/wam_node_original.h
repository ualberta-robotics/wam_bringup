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

#include "wam_bringup/utilities.h"
#include "wam_bringup/arm_linking.h"
#include "wam_bringup/tool_force_application.h"
#include "wam_bringup/visual_path_with_normals.h"
#include "wam_bringup/path_normals_to_quaternion.h"
#include "wam_bringup/tangential_velocity.h"
#include "wam_bringup/orientation_controller_variable_gains.h"
#include "wam_bringup/impedence_controller.h"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <string>
#include <sstream>

#include <boost/thread.hpp> // BarrettHand threading
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_comparison.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <Eigen/Geometry>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "wam_msgs/RTJointPos.h"
#include "wam_msgs/RTJointVel.h"
#include "wam_msgs/RTCartPos.h"
#include "wam_msgs/RTCartVel.h"
#include "wam_msgs/RTOrtnPos.h"
#include "wam_msgs/RTOrtnVel.h"
#include "wam_msgs/MatrixMN.h"
#include "std_srvs/Empty.h"
#include "wam_srvs/BHandFingerPos.h"
#include "wam_srvs/BHandGraspPos.h"
#include "wam_srvs/BHandSpreadPos.h"
#include "wam_srvs/BHandFingerVel.h"
#include "wam_srvs/BHandGraspVel.h"
#include "wam_srvs/BHandSpreadVel.h"
#include "wam_srvs/BHandPinchPos.h"
#include "wam_srvs/GravityComp.h"
#include "wam_srvs/HapticSphere.h"
#include "wam_srvs/Hold.h"
#include "wam_srvs/JointMove.h"
#include "wam_srvs/JointMoveBlock.h"
#include "wam_srvs/PoseMove.h"
#include "wam_srvs/CartPosMove.h"
#include "wam_srvs/CartVel.h"
#include "wam_srvs/OrtnMove.h"
#include "wam_srvs/OrtnSplitMove.h"
#include "wam_srvs/Teach.h"
#include "wam_srvs/Play.h"
#include "wam_srvs/Link.h"
#include "wam_srvs/ForceTorqueTool.h"
#include "wam_srvs/FollowPath.h"
#include "wam_srvs/TP_PID.h"
#include "wam_srvs/JP_PID.h"
#include "wam_srvs/JV_PID.h"
#include "wam_srvs/ForceTorqueToolTime.h"
#include "wam_srvs/ForceTorqueBase.h"
#include "wam_srvs/ForceTorque.h"

#include <barrett/exception.h>
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
#include <barrett/systems/wam.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/log.h>


static const int PUBLISH_FREQ = 250; // Default Control Loop / Publishing Frequency
static const double SPEED = 0.03; // Default Cartesian Velocity

using namespace barrett;
using barrett::detail::waitForEnter;
using systems::connect;
using systems::disconnect;
using systems::reconnect;
// using detail::waitForEnter;

BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

//WamNode Class
template<size_t DOF>
	class WamNode
	{
		BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	protected:
		typedef boost::tuple<double, jp_type> jp_sample_type;

		bool cart_vel_status; // real time control
		bool ortn_vel_status; // real time control
		bool jnt_vel_status; // real time control
		bool jnt_pos_status; // real time control
		bool jnt_hand_tool_status; // real time control
		bool cart_pos_status; // real time control
		bool ortn_pos_status; // real time control
		bool new_rt_cmd; // real time control
		bool systems_connected;

		double cart_vel_mag; // real time control
		double ortn_vel_mag; // real time control
		double fn; // global normal force

		VisualHapticPathNormals *vhp; // haptics 
		applyForceToolFrame<DOF> appForce1; // force control


		MasterMaster<DOF> *mm; // linking
		tangentialVelocityCompute<DOF> computeTangentVel; // cartVel
		OrientationControllerVariableGains<DOF> OrtnSplitCont;
		systems::Wam<DOF>& wam;

		jp_type jp;
		jp_type jp_cmd;
		jp_type jp_home;
		jp_type rt_jp_cmd;
		jp_type rt_hand_tool_cmd;
		jp_type rt_jp_rl;

		jv_type rt_jv_cmd;
		
		cp_type rt_cv_cmd;
		cp_type cp_cmd;
		cp_type rt_cp_cmd;
		cp_type rt_cp_rl;

		cp_type max_base_torque;
		cp_type max_base_force;
		cp_type max_tool_torque;
		cp_type max_tool_force;

		cp_type SpringSetPoint;
		Eigen::Quaterniond OrnSpringSetPoint;

		pose_type pose_cmd;

		// haptic sphere variables
        systems::HapticBall haptic_ball;
        systems::PIDController<double, double> pid_control_ball;
        systems::TupleGrouper<cf_type, double> tg_haptic_ball;
        systems::Callback<boost::tuple<cf_type, double>, cf_type> mult_haptic_ball;
        systems::ToolForceToJointTorques<DOF> tf2jt_haptic_ball;

        // pid_control variables
        systems::PIDController<double, double> dynamic_pid_control;
        // impedance controller for ForceTorqueBase
        ImpedanceController6DOF<DOF> ImpControl;
		systems::ExposedOutput<cp_type> KxSet;
		systems::ExposedOutput<cp_type> DxSet;
		systems::ExposedOutput<cp_type> XdSet;
		systems::ExposedOutput<cp_type> OrnKxSet;
		systems::ExposedOutput<cp_type> OrnDxSet;
		systems::ExposedOutput<Eigen::Quaterniond> OrnXdSet;

		//Note: orientationSetPoint is used both in holdOrtn, holdOrtn2 and ortnSplitMove
		systems::ExposedOutput<Eigen::Quaterniond> orientationSetPoint;
		systems::ExposedOutput<Eigen::Quaterniond> current_ortn;
		systems::ExposedOutput<cp_type> cart_dir;
		systems::ExposedOutput<cp_type> current_cart_pos;
		systems::ExposedOutput<cp_type> cp_track;
		systems::ExposedOutput<math::Vector<3>::type> rpy_cmd;
		systems::ExposedOutput<math::Vector<3>::type> current_rpy_ortn;
		systems::ExposedOutput<jv_type> jv_track;
		systems::ExposedOutput<jp_type> jp_track;
		systems::ExposedOutput<jp_type> exposedOutputHandTool;
		systems::TupleGrouper<cp_type, Eigen::Quaterniond> rt_pose_cmd;
		systems::Summer<cp_type> cart_pos_sum;
		systems::Summer<math::Vector<3>::type> ortn_cmd_sum;
		systems::Ramp ramp;
		systems::RateLimiter<jp_type> jp_rl;
		systems::RateLimiter<cp_type> cp_rl;
		Multiplier<double, cp_type, cp_type> mult_linear;
		Multiplier<double, math::Vector<3>::type, math::Vector<3>::type> mult_angular;
		ToQuaternion to_quat;
		ToQuaternion to_quat_print;
		Eigen::Quaterniond ortn_print;
		Eigen::Quaterniond ortn_cmd;
		Eigen::Quaterniond rt_op_cmd;
		Eigen::Quaterniond rt_op_rl;
		math::Vector<3>::type rt_ortn_cmd;
		//Haptics
		systems::ExposedOutput<double> vs_error_sys;
		systems::ExposedOutput<cf_type> dir_sys;
		systems::Summer<cf_type> dirSum;
		systems::Summer<double> depthSum;
		systems::PIDController<double, double> comp;
		systems::PIDController<double, double> VelocityComp;
		systems::Constant<double> zero;
		systems::ExposedOutput<jt_type> zero_joystick_torque;
		systems::TupleGrouper<cf_type, double> tg;
		systems::TupleGrouper<cp_type, double> tg2;
		systems::Callback<boost::tuple<cf_type, double>, cf_type> mult;
		systems::Callback<boost::tuple<cp_type, double>, cf_type> mult2;
		systems::ToolForceToJointTorques<DOF> tf2jt;
		systems::ToolForceToJointTorques<DOF> tf2jt_cartVel;
		jt_type jtLimits;
		systems::Callback<jt_type> jtSat, jtSat_cartVel, jtSat_ornSplit;
		//Path Following Service
		systems::Gain<cp_type, double, cf_type> tangentGain;//(0.0);
		systems::Summer<cf_type> tfSum;
		systems::Summer<cf_type> tfSumNormalTangential;
		systems::Gain<cp_type, double, cf_type> normalForceGain;//(0.0);
		//Orientation to set the tool frame perpendicular to the path. 
		computeQuaternionSetPoint<DOF> computeQuaternionSetPoint1;
		//ForceTorque Control
		//systems::Callback<jt_type> jtSat(boost::bind(saturateJt<DOF>, _1, jtLimits));
		systems::ToolForceToJointTorques<DOF> toolforce2jt;
		systems::ToolTorqueToJointTorques<DOF> tooltorque2jt;
		systems::ToolForceToJointTorques<DOF> toolforce2jt2;
		systems::ToolTorqueToJointTorques<DOF> tooltorque2jt2;
		systems::ExposedOutput<cf_type> exposedOutputForce;
		systems::ExposedOutput<ct_type> exposedOutputTorque;
		systems::Summer<jt_type> torqueSum;
		systems::Summer<jt_type> torqueSum2;
		systems::Summer<jt_type> torqueSum3;
		systems::Summer<jt_type> torqueSum4;
		//VelocityControl Service
		systems::ExposedOutput<double> exposedDesiredVel;
		systems::ExposedOutput<cp_type> exposedDirVel;
		//orientation Controller-->not using neither moveTo nor trackReference 
		systems::ToolOrientationController<DOF> toolOrntController;
		systems::ToolTorqueToJointTorques<DOF> tt2jtOrnController;
		systems::Summer<jt_type,3> jtSummer;
		//Orientation Split Move service 
		systems::ExposedOutput<cp_type> KpOrnSet ;
		systems::ExposedOutput<cp_type> KdOrnSet ;
		systems::ToolTorqueToJointTorques<DOF> tt2jt_ortn_split;
		// ros
		ros::Time last_cart_vel_msg_time;
		ros::Time last_ortn_vel_msg_time;
		ros::Time last_jnt_vel_msg_time;
		ros::Time last_jnt_pos_msg_time;
		ros::Time last_jnt_hand_tool_msg_time;
		ros::Time last_cart_pos_msg_time;
		ros::Time last_ortn_pos_msg_time;
		ros::Duration rt_msg_timeout;
		//Subscribed Topics
		wam_msgs::RTCartVel cart_vel_cmd;
		wam_msgs::RTOrtnVel ortn_vel_cmd;
		//Subscribers
		ros::Subscriber cart_vel_sub;
		ros::Subscriber ortn_vel_sub;
		ros::Subscriber jnt_vel_sub;
		ros::Subscriber jnt_pos_sub;
		ros::Subscriber jnt_hand_tool_sub;
		ros::Subscriber cart_pos_sub;
		ros::Subscriber ortn_pos_sub;
		ros::Subscriber vs_error;
		//Published Topics
		sensor_msgs::JointState wam_joint_state;
		geometry_msgs::PoseStamped wam_pose;
		wam_msgs::MatrixMN wam_jacobian_mn;
		//Publishers
		ros::Publisher wam_joint_state_pub;
		ros::Publisher wam_pose_pub;
		ros::Publisher wam_jacobian_mn_pub;
		//Services
		ros::ServiceServer gravity_srv;
		ros::ServiceServer go_home_srv;
		ros::ServiceServer haptic_sphere_srv;
		ros::ServiceServer hold_jpos_srv;
		ros::ServiceServer hold_cpos_srv;
		ros::ServiceServer hold_ortn_srv;
		ros::ServiceServer hold_ortn2_srv;
		ros::ServiceServer joint_move_srv;
		ros::ServiceServer joint_move_block_srv;
		ros::ServiceServer pose_move_srv;
		ros::ServiceServer force_torque_base_srv;
		ros::ServiceServer force_torque_tool_srv;
		ros::ServiceServer cart_move_srv;
		ros::ServiceServer cart_vel_srv;
		ros::ServiceServer ortn_move_srv;
		ros::ServiceServer ortn_split_move_srv;
		ros::ServiceServer teach_srv;
		ros::ServiceServer play_srv;
		ros::ServiceServer link_arm_srv;
		ros::ServiceServer unlink_arm_srv;
		ros::ServiceServer start_visual_fix;
		ros::ServiceServer stop_visual_fix;
		ros::ServiceServer follow_path_srv;
		// LP control experiments
		ros::ServiceServer jp_pid_srv;
		ros::ServiceServer jv_pid_srv;
		ros::ServiceServer tp_pid_srv;
		ros::ServiceServer force_torque_tool_time_srv;
		ros::ServiceServer force_torque_base_time_srv;	

		ros::ServiceServer disconnect_systems_srv;	
		ros::ServiceServer joy_ft_base_srv;	
		ros::ServiceServer joy_ft_tool_srv;	

	public:
		ros::NodeHandle n_;
		ProductManager* mypm;
		bool exit_haptic_sphere;

		WamNode(systems::Wam<DOF>& wam_) :
			n_("wam"),
			wam(wam_), 
			ramp(NULL, SPEED),
			vs_error_sys(0), 
			dir_sys(cf_type::Unit(1) * -1), 
			zero(0), 
			mult(scale), 
			mult2(scale), 
			mult_haptic_ball(scale),
			haptic_ball(cp_type(0.0, 0.0, 0.0), 0.05),
			jtLimits(35.0), 
			jtSat(boost::bind(saturateJt<DOF>, _1, jtLimits)), 
			jtSat_cartVel(boost::bind(saturateJt<DOF>, _1, jtLimits)),
			jtSat_ornSplit(boost::bind(saturateJt<DOF>, _1, jtLimits)), 
			tangentGain(0.0),
			normalForceGain(0.0)
   		{
			// configure Systems
			comp.setKp(3e3);
			comp.setKd(3e1);
		}
		~WamNode() {}

		void init(ProductManager& pm);
        void disconnectSystems();
		bool disconnectSystems(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool joyForceTorqueBase(wam_srvs::ForceTorque::Request &req, wam_srvs::ForceTorque::Response &res);
		bool joyForceTorqueTool(wam_srvs::ForceTorque::Request &req, wam_srvs::ForceTorque::Response &res);

		bool jpPIDControl(wam_srvs::JP_PID::Request &req, wam_srvs::JP_PID::Response &res);
		bool jvPIDControl(wam_srvs::JV_PID::Request &req, wam_srvs::JV_PID::Response &res);
		bool tpPIDControl(wam_srvs::TP_PID::Request &req, wam_srvs::TP_PID::Response &res);
		bool hapticSphere(wam_srvs::HapticSphere::Request &req, wam_srvs::HapticSphere::Response &res);
		bool gravity(wam_srvs::GravityComp::Request &req, wam_srvs::GravityComp::Response &res);
		bool goHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool holdJPos(wam_srvs::Hold::Request &req, wam_srvs::Hold::Response &res);
		bool holdCPos(wam_srvs::Hold::Request &req, wam_srvs::Hold::Response &res);
		bool holdOrtn(wam_srvs::Hold::Request &req, wam_srvs::Hold::Response &res);
		bool holdOrtn2(wam_srvs::Hold::Request &req, wam_srvs::Hold::Response &res);
		bool jointMove(wam_srvs::JointMove::Request &req, wam_srvs::JointMove::Response &res);
		bool jointMoveBlock(wam_srvs::JointMoveBlock::Request &req, wam_srvs::JointMoveBlock::Response &res);
		bool poseMove(wam_srvs::PoseMove::Request &req, wam_srvs::PoseMove::Response &res);
		bool cartMove(wam_srvs::CartPosMove::Request &req, wam_srvs::CartPosMove::Response &res);
		bool cartVel(wam_srvs::CartVel::Request &req, wam_srvs::CartVel::Response &res);
		bool ortnMove(wam_srvs::OrtnMove::Request &req, wam_srvs::OrtnMove::Response &res);
		bool ortnSplitMove(wam_srvs::OrtnSplitMove::Request &req, wam_srvs::OrtnSplitMove::Response &res);
		bool forceTorqueBase(wam_srvs::ForceTorqueBase::Request &req, wam_srvs::ForceTorqueBase::Response &res);
		bool forceTorqueTool(wam_srvs::ForceTorqueTool::Request &req, wam_srvs::ForceTorqueTool::Response &res);
		bool forceTorqueToolTime(wam_srvs::ForceTorqueToolTime::Request &req, wam_srvs::ForceTorqueToolTime::Response &res);
		bool forceTorqueBaseTime(wam_srvs::ForceTorqueToolTime::Request &req, wam_srvs::ForceTorqueToolTime::Response &res);


		bool teachMotion(wam_srvs::Teach::Request &req, wam_srvs::Teach::Response &res);
		bool playMotion(wam_srvs::Play::Request &req, wam_srvs::Play::Response &res);
		bool linkArm(wam_srvs::Link::Request &req, wam_srvs::Link::Response &res);
		bool unLinkArm(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool startVisualFix(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool stopVisualFix(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool followPath(wam_srvs::FollowPath::Request &req, wam_srvs::FollowPath::Response &res);
		void cartVelCB(const wam_msgs::RTCartVel::ConstPtr& msg);
		void ortnVelCB(const wam_msgs::RTOrtnVel::ConstPtr& msg);
		void jntVelCB(const wam_msgs::RTJointVel::ConstPtr& msg);
		void jntPosCB(const wam_msgs::RTJointPos::ConstPtr& msg);
		void jntHandToolCB(const sensor_msgs::JointState::ConstPtr& msg); 
		void cartPosCB(const wam_msgs::RTCartPos::ConstPtr& msg);
		void vsErrCB(const std_msgs::Float32::ConstPtr& msg);
		void publishWam(ProductManager& pm);
		void updateRT(ProductManager& pm);
	};
