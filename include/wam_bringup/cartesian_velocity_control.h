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
 */

#include "wam_bringup/utilities.h"
#include "wam_bringup/orientation_controller_variable_gains.h"
#include "wam_bringup/tangential_velocity.h"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <string>
#include <sstream>

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
#include "wam_msgs/RTOrtn.h"
#include "wam_msgs/RTCartVel.h"
#include "wam_msgs/RTVelocity.h"
#include "wam_msgs/MatrixMN.h"
#include "wam_msgs/RTToolInfo.h"
#include "std_srvs/Empty.h"
#include "wam_srvs/JointMoveBlock.h"
#include "wam_srvs/Hold.h"
#include "wam_srvs/Connect.h"
#include "wam_srvs/OrtnGains.h"

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

BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

//CartesianVelocityControl Class
template<size_t DOF>
	class CartesianVelocityControl
	{
		BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	protected:
		bool locked_orientation;
		bool locked_joints;
		bool update_gains;
		jp_type jp;
		jp_type jp_cmd;
		jp_type jp_home;
	    cf_type force_applied;
	    ct_type torque_applied;

		systems::Wam<DOF>& wam;
		systems::PIDController<double, double> comp;
		// real-time variables for velocity controller
		bool cv_realtime_status;
		bool new_cv_command;
		double cv_magnitude;
		double cv_kp;
		ros::Time last_cv_msg_time;
		cv_type cv_direction;
		systems::PIDController<double, double> VelocityComp;
		systems::ExposedOutput<double> exposedDesiredVel;
		systems::ExposedOutput<cp_type> exposedDirVel;
		systems::ToolForceToJointTorques<DOF> tf2jt_cartVel;
		// VisualHapticPathNormals *vhp;
		tangentialVelocityCompute<DOF> computeTangentVel;
		systems::Constant<double> zero;
		systems::TupleGrouper<cp_type, double> tg2;
		systems::Callback<boost::tuple<cp_type, double>, cf_type> mult;
		// systems::Summer<jt_type,3> jtSummer;

		// real-time variables for force controller
		bool cf_realtime_status;
		bool new_cf_command;
		double cf_magnitude;
		ros::Time last_cf_msg_time;
		systems::Ramp ramp;
		systems::ExposedOutput<cf_type> exposedOutputForce;
		systems::ExposedOutput<ct_type> exposedOutputTorque;
		systems::ToolForceToJointTorques<DOF> toolforce2jt;
		systems::ToolTorqueToJointTorques<DOF> tooltorque2jt;
		systems::Summer<jt_type> torqueSum, torqueSum2;
		// orientation controller 
		bool ortn_realtime_status;
		bool new_ortn_command;
        cp_type OrtnKp, OrtnKd;
		ros::Time last_ortn_msg_time;
        Eigen::Quaterniond ortn_direction;
		systems::ExposedOutput<Eigen::Quaterniond> orientationSetPoint;
		systems::ToolOrientationController<DOF> toolOrntController;
		systems::ToolTorqueToJointTorques<DOF> tt2jtOrnController;
		// orientation split move controller
		OrientationControllerVariableGains<DOF> OrtnSplitCont;
		systems::ExposedOutput<cp_type> KpOrnSet;
		systems::ExposedOutput<cp_type> KdOrnSet;
		systems::ToolTorqueToJointTorques<DOF> tt2jt_ortn_split;

		jt_type jtLimits;
		systems::Callback<jt_type> jtSat;
		systems::Callback<jt_type> jtSat_ornSplit;
		systems::Callback<jt_type> jtSat_cartVel;

		ros::Duration msg_timeout;

		// subscribed topics
		// wam_msgs::RTCartVel cart_vel_cmd;
		// subscribers
		ros::Subscriber cart_force_sub;
		ros::Subscriber cart_vel_sub;
		ros::Subscriber ortn_sub;
		// published topics
		sensor_msgs::JointState wam_joint_state;
		geometry_msgs::PoseStamped wam_pose;
		wam_msgs::MatrixMN wam_jacobian_mn;
		wam_msgs::RTToolInfo wam_tool_info;
		// publishers
		ros::Publisher wam_joint_state_pub;
		ros::Publisher wam_pose_pub;
		ros::Publisher wam_jacobian_mn_pub;
		ros::Publisher wam_tool_pub;
		// services
		// ros::ServiceServer gravity_srv;
		ros::ServiceServer go_home_srv;
		ros::ServiceServer joint_move_block_srv;
		ros::ServiceServer hold_joint_pos_srv;
		ros::ServiceServer hold_ortn_srv;
		ros::ServiceServer set_ortn_gains_srv;
		ros::ServiceServer connect_force_srv;
		ros::ServiceServer connect_vel_srv;

	public:
		ros::NodeHandle n_; // WAM specific nodehandle
		ProductManager* mypm;
		CartesianVelocityControl(systems::Wam<DOF>& wam_) :
			n_("wam"),
			wam(wam_), 
			zero(0), 
			mult(scale), 			
			jtLimits(35.0), 
			jtSat(boost::bind(saturateJt<DOF>, _1, jtLimits)), 
			jtSat_cartVel(boost::bind(saturateJt<DOF>, _1, jtLimits)),
			jtSat_ornSplit(boost::bind(saturateJt<DOF>, _1, jtLimits)), 
			ramp(NULL, SPEED)
   		{
			// configure Systems
			comp.setKp(3e3);
			comp.setKd(3e1);
		}
		~CartesianVelocityControl(){}
		void init(ProductManager& pm);
		bool go_home_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool hold_joint_pos_callback(wam_srvs::Hold::Request &req, wam_srvs::Hold::Response &res);
		bool joint_move_block_callback(wam_srvs::JointMoveBlock::Request &req, wam_srvs::JointMoveBlock::Response &res);
		bool hold_tool_ortn_callback(wam_srvs::Hold::Request &req, wam_srvs::Hold::Response &res);
		bool set_ortn_gains_callback(wam_srvs::OrtnGains::Request &req, wam_srvs::OrtnGains::Response &res);
		bool connect_force_orientation_callback(wam_srvs::Connect::Request &req, wam_srvs::Connect::Response &res);
		bool connect_velocity_callback(wam_srvs::Connect::Request &req, wam_srvs::Connect::Response &res);
		void cart_force_callback(const wam_msgs::RTCartVel::ConstPtr& msg);
		void cart_velocity_callback(const wam_msgs::RTVelocity::ConstPtr& msg);
		void orientation_callback(const wam_msgs::RTOrtn::ConstPtr& msg);
		void disconnect_systems();
		void publish_wam(ProductManager& pm);
		void update_realtime(ProductManager& pm);
	};
