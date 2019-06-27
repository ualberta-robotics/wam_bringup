/*
 * applyForceToolFrame.h
 *
 *  Created on: Sep 11, 2015
 *      Author: mas
 */
#ifndef TANGENTIAL_VELOCITY_H_
#define TANGENTIAL_VELOCITY_H_
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



// --------- tangential velocity controller --------
// inputs: v = wam.toolVelocity, output: tangential velocity Vt =  dotproduct (v , tanDir)
template<size_t DOF>
class tangentialVelocityCompute : public systems::System
{
BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

// IO  (inputs)
public:
Input<cv_type> toolVelocityInput;
Input<cp_type> tangentDirInput;

// IO  (outputs)
public:
Output<double> tangentVelOutput;     // tangential velocity

protected:
typename Output<double>::Value* tangentVelOutputValue;

public:
double cpomputedTangentialVel; 	 // can be used to display later in the program

public:
tangentialVelocityCompute(
      const std::string& sysName = "tangentialVelocityCompute") :
      System(sysName), toolVelocityInput(this), tangentDirInput(this),
      tangentVelOutput(this, &tangentVelOutputValue)
{
}
virtual ~tangentialVelocityCompute()
{
this->mandatoryCleanUp();
}
protected:
    cp_type tanDir;
    cv_type vel;

    double tanVel;

virtual void operate()
{

  tanDir = tangentDirInput.getValue();
  vel = toolVelocityInput.getValue();

  tanVel = tanDir.dot(vel) ;
  cpomputedTangentialVel = tanVel;

  tangentVelOutputValue->setData( & tanVel );

}

private:
DISALLOW_COPY_AND_ASSIGN(tangentialVelocityCompute);
};
// -----------------------------------

#endif /*TANGENTIAL_VELOCITY_H_*/
