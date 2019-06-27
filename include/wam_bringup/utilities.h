/* 
 * lpetrich 19/08/18
 */

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

#include <barrett/exception.h>
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
#include <barrett/systems/wam.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/log.h>

using namespace barrett;
using barrett::detail::waitForEnter;
using systems::connect;
using systems::disconnect;
using systems::reconnect;

BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

// haptics
cf_type scale(boost::tuple<cf_type, double> t) 
{
	return t.get<0>() * t.get<1>();
}

//haptics
template <size_t DOF>
	typename units::JointTorques<DOF>::type saturateJt(const typename units::JointTorques<DOF>::type& x, const typename units::JointTorques<DOF>::type& limit) 
	{
		int index;
		double minRatio;
		minRatio = (limit.cwise() / (x.cwise().abs())).minCoeff(&index);
		if (minRatio < 1.0) { return minRatio * x; } 
		else { return x; }
	}

//Creating a templated multiplier for our real-time computation
template<typename T1, typename T2, typename OutputType>
	class Multiplier : public systems::System, public systems::SingleOutput<OutputType>
	{
	public:
		Input<T1> input1;

	public:
		Input<T2> input2;

	public:
		Multiplier(std::string sysName = "Multiplier") : 
			systems::System(sysName), systems::SingleOutput<OutputType>(this), input1(this), input2(this) 
		{
		}
		virtual ~Multiplier() 
		{ 
			mandatoryCleanUp(); 
		}

	protected:
		OutputType data;
		virtual void operate()
		{
			data = input1.getValue() * input2.getValue();
			this->outputValue->setData(&data);
		}

	private:
		DISALLOW_COPY_AND_ASSIGN(Multiplier);

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

//Creating a templated converter from Roll, Pitch, Yaw to Quaternion for real-time computation
class ToQuaternion : public systems::SingleIO<math::Vector<3>::type, Eigen::Quaterniond>
{
	public:
		Eigen::Quaterniond outputQuat;

	public:
		ToQuaternion(std::string sysName = "ToQuaternion") :
			systems::SingleIO<math::Vector<3>::type, Eigen::Quaterniond>(sysName)
		{
		}
		virtual ~ToQuaternion()
		{
			mandatoryCleanUp();
		}

	protected:
		tf::Quaternion q;
		virtual void operate()
		{
			const math::Vector<3>::type &inputRPY = input.getValue();
			q.setEuler(inputRPY[2], inputRPY[1], inputRPY[0]);
			outputQuat.x() = q.getX();
			outputQuat.y() = q.getY();
			outputQuat.z() = q.getZ();
			outputQuat.w() = q.getW();
			this->outputValue->setData(&outputQuat);
		}

	private:
		DISALLOW_COPY_AND_ASSIGN(ToQuaternion);

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//Simple Function for converting Quaternion to RPY
math::Vector<3>::type toRPY(Eigen::Quaterniond inquat)
{
	math::Vector<3>::type newRPY;
	tf::Quaternion q(inquat.x(), inquat.y(), inquat.z(), inquat.w());
	tf::Matrix3x3(q).getRPY(newRPY[0], newRPY[1], newRPY[2]);
	return newRPY;
}
