
#ifndef ORIENTATION_CONTROLLER_VARIABLE_GAINS_H_
#define ORIENTATION_CONTROLLER_VARIABLE_GAINS_H_
#include <cmath>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <libconfig.h++>
#include <barrett/detail/ca_macro.h>
#include <barrett/cdlbt/calgrav.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/systems/kinematics_base.h>
#include <barrett/math/kinematics.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <cstdlib>  // For std::atexit()
#include <barrett/os.h>  // For btsleep()
#include <barrett/math.h>  // For barrett::math::saturate()
#include <Eigen/Dense>
#include <Eigen/Core>
#include <barrett/standard_main_function.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>
#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>

using namespace barrett;
using namespace systems;
using systems::connect;
BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;


template<size_t DOF>
class OrientationControllerVariableGains : public systems::System  , public KinematicsInput<DOF>
{
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	Input<Eigen::Quaterniond> ReferenceOrnInput;
	Input<Eigen::Quaterniond> FeedbackOrnInput;
	Input<cp_type> KpGains;
	Input<cp_type> KdGains;

public:
	Output<ct_type> CTOutput;

protected:
	typename Output<ct_type>::Value* ctOutputValue;

public:
	OrientationControllerVariableGains(
//    const units::CartesianPosition::type& ProportionalGains,
//    const units::CartesianPosition::type& DerivativeGains,
    const std::string& sysName = "OrientationControllerVariableGains") :
      System(sysName), KinematicsInput<DOF>(this), ReferenceOrnInput(this), FeedbackOrnInput(this),
      KpGains(this) , KdGains(this),
      CTOutput(this, &ctOutputValue)
//      , ProportionalGains(ProportionalGains), DerivativeGains(DerivativeGains)



{
}
virtual ~OrientationControllerVariableGains()
{
this->mandatoryCleanUp();
}


protected:
	cp_type ProportionalGains;
	cp_type DerivativeGains;

	Eigen::AngleAxisd error;
	ct_type ct;

	cp_type tempVect;
	cp_type tempVect2;
	cp_type errorVect;

	virtual void operate() {

		ProportionalGains = KpGains.getValue();
		DerivativeGains	  = KdGains.getValue();

//		error = this->referenceInput.getValue() * this->feedbackInput.getValue().inverse();  // I think it should be this way
		error = this->FeedbackOrnInput.getValue() * this->ReferenceOrnInput.getValue().inverse();  // but CD's math (that works, BTW) does it this way

		double angle = error.angle();
		// TODO(dc): I looked into Eigen's implementation and noticed that angle will always be between 0 and 2*pi. We should test for this so if Eigen changes, we notice.
		if (angle > M_PI) {
			angle -= 2.0*M_PI;
		}

		if (math::abs(angle) > 3.13) {	// a little dead-zone near the discontinuity at +/-180 degrees
			ct.setZero();
		} else {
		    tempVect = ProportionalGains ;				// copy the ProportiocalGains
		    errorVect = error.axis() * angle ;
		    gsl_vector_mul (tempVect.asGslType() , errorVect.asGslType()  ) ; 	// tempVect <- tempVect * errorVect
		    ct = this->ReferenceOrnInput.getValue().inverse() * ( tempVect );

		}

		tempVect2 = DerivativeGains;
		gsl_vector_mul ( tempVect2.asGslType() , this->kinInput.getValue().impl->tool_velocity_angular ) ;
		ct -= tempVect2 ;
//		ct -= DerivativeGains.dot( this->kinInput.getValue().impl->tool_velocity_angular ) ;
//		gsl_blas_daxpy( -kd, this->kinInput.getValue().impl->tool_velocity_angular, ct.asGslType());

		this->ctOutputValue->setData(&ct);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(OrientationControllerVariableGains);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif /*ORIENTATION_CONTROLLER_VARIABLE_GAINS_H_*/
