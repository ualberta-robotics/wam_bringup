/*
 * applyForceToolFrame.h
 *
 *  Created on: Sep 11, 2015
 *      Author: mas
 */
#ifndef APPLY_FORCE_TOOLFRAME_H_
#define APPLY_FORCE_TOOLFRAME_H_
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
class applyForceToolFrame : public systems::System
{
BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

// IO  (inputs)
public:
Input<cp_type> CpInput;                 // cart pos. input
Input<Eigen::Quaterniond> OrnInput;     // orientation input (quaternion)
Input<cf_type> FtInput;                 // cart force input (in tool frame)
Input<ct_type> TtInput;                // cart torque input (in tool frame)

// IO  (outputs)
public:
Output<cf_type> CFOutput;    // output Cartesian forces (in base frame)
Output<ct_type> CTOutput;    // output Cartesian torque (in base frame)

protected:
typename Output<cf_type>::Value* cfOutputValue;
typename Output<ct_type>::Value* ctOutputValue;

public:
cf_type computedF;          // can be used to display later in the program
ct_type computedT;          // can be used to display later in the program

public:
applyForceToolFrame(
      const std::string& sysName = "applyForceToolFrame") :
      System(sysName), CpInput(this), OrnInput(this), FtInput(this) , TtInput(this) ,
      CFOutput(this, &cfOutputValue), CTOutput(this, &ctOutputValue)
{
}
virtual ~applyForceToolFrame()
{
this->mandatoryCleanUp();
}
protected:
    cf_type cf;
    ct_type ct;

    cf_type ForceTool;
    ct_type TorqueTool;
    cp_type Xcurr;
    Eigen::Quaterniond OrnCurr;

virtual void operate()
{

  Xcurr      = CpInput.getValue();       // current Cart. Pose
  OrnCurr    = OrnInput.getValue();      // current tool Orientation (quaternion)
  ForceTool  = FtInput.getValue();       // desired tool force
  TorqueTool = TtInput.getValue();       // desired tool torque

  OrnCurr.x() = - OrnCurr.x();           // wam quaternion imaginary part in negative
  OrnCurr.y() = - OrnCurr.y();           // wam quaternion imaginary part in negative
  OrnCurr.z() = - OrnCurr.z();           // wam quaternion imaginary part in negative
//****************************************
//  gsl_matrix* RotMat = gsl_matrix_alloc (3,3);
//  gsl_vector* Ft = gsl_vector_alloc(3);   // Force  in tool frame
//  gsl_vector* Tt = gsl_vector_alloc(3);   // Torque in tool frame
//
//  gsl_vector* Fb = gsl_vector_alloc(3);   // Force  in base frame
//  gsl_vector* Tb = gsl_vector_alloc(3);   // Torque in base frame
//
//  // RotMat = [  1 - 2*( OrnCurr.y()*OrnCurr.y() + OrnCurr.z()*OrnCurr.z() ) ,       2*(OrnCurr.x()*OrnCurr.y() - OrnCurr.w()*OrnCurr.z())   ,       2*(OrnCurr.w()*OrnCurr.y() + OrnCurr.x()*OrnCurr.z()) ;
//  //                 2*(OrnCurr.x()*OrnCurr.y() + OrnCurr.w()*OrnCurr.z())   ,   1 - 2*(OrnCurr.x()^2 + OrnCurr.z()^2)     ,       2*(OrnCurr.y()*OrnCurr.z() - OrnCurr.w()*OrnCurr.x()) ;
//  //                 2*(OrnCurr.x()*OrnCurr.z() - OrnCurr.w()*OrnCurr.y())   ,       2*(OrnCurr.w()*OrnCurr.x() + OrnCurr.y()*OrnCurr.z())   ,   1 - 2*(OrnCurr.x()^2 + OrnCurr.y()^2)     ] ;
//
// gsl_matrix_set(RotMat, 0 , 0 , 1 - 2*(OrnCurr.y()*OrnCurr.y() + OrnCurr.z()*OrnCurr.z()) );
// gsl_matrix_set(RotMat, 0 , 1 ,     2*(OrnCurr.x()*OrnCurr.y() - OrnCurr.w()*OrnCurr.z()) );
// gsl_matrix_set(RotMat, 0 , 2 ,     2*(OrnCurr.w()*OrnCurr.y() + OrnCurr.x()*OrnCurr.z()) );
//
// gsl_matrix_set(RotMat, 1 , 0 ,     2*(OrnCurr.x()*OrnCurr.y() + OrnCurr.w()*OrnCurr.z()) );
// gsl_matrix_set(RotMat, 1 , 1 , 1 - 2*(OrnCurr.x()*OrnCurr.x() + OrnCurr.z()*OrnCurr.z()) );
// gsl_matrix_set(RotMat, 1 , 2 ,     2*(OrnCurr.y()*OrnCurr.z() - OrnCurr.w()*OrnCurr.x())  );
//
// gsl_matrix_set(RotMat, 2 , 0 ,     2*(OrnCurr.x()*OrnCurr.z() - OrnCurr.w()*OrnCurr.y())  );
// gsl_matrix_set(RotMat, 2 , 1 ,     2*(OrnCurr.w()*OrnCurr.x() + OrnCurr.y()*OrnCurr.z())  );
// gsl_matrix_set(RotMat, 2 , 2 , 1 - 2*(OrnCurr.x()*OrnCurr.x() + OrnCurr.y()*OrnCurr.y())  );
//
//
//
// for (int i = 0; i<3 ; i++)
//        {
//           gsl_vector_set (Ft , i , ForceTool[i] ) ;
//           gsl_vector_set (Tt , i , TorqueTool[i] ) ;
//        }
//
// gsl_blas_dgemv(CblasNoTrans, 1.0, RotMat, Ft, 0.0, Fb);    // Fb = 1.0 * RotMat * Ft + 0.0
// gsl_blas_dgemv(CblasNoTrans, 1.0, RotMat, Tt, 0.0, Tb);    // Tb = 1.0 * RotMat * Tt + 0.0
//
// for (int i = 0; i<3 ; i++)
//       {
//           cf[i] = gsl_vector_get ( Fb , i );
//           ct[i] = gsl_vector_get ( Tb , i );
//       }
//****************************************

  cf[0] = ( 1 - 2*(OrnCurr.y()*OrnCurr.y() + OrnCurr.z()*OrnCurr.z()) ) * ForceTool[0] +
          (     2*(OrnCurr.x()*OrnCurr.y() - OrnCurr.w()*OrnCurr.z()) ) * ForceTool[1] +
          (     2*(OrnCurr.w()*OrnCurr.y() + OrnCurr.x()*OrnCurr.z()) ) * ForceTool[2]   ;

  cf[1] = (     2*(OrnCurr.x()*OrnCurr.y() + OrnCurr.w()*OrnCurr.z()) ) * ForceTool[0] +
          ( 1 - 2*(OrnCurr.x()*OrnCurr.x() + OrnCurr.z()*OrnCurr.z()) ) * ForceTool[1] +
          (     2*(OrnCurr.y()*OrnCurr.z() - OrnCurr.w()*OrnCurr.x()) ) * ForceTool[2]   ;

  cf[2] = (     2*(OrnCurr.x()*OrnCurr.z() - OrnCurr.w()*OrnCurr.y()) ) * ForceTool[0] +
          (     2*(OrnCurr.w()*OrnCurr.x() + OrnCurr.y()*OrnCurr.z()) ) * ForceTool[1] +
          ( 1 - 2*(OrnCurr.x()*OrnCurr.x() + OrnCurr.y()*OrnCurr.y()) ) * ForceTool[2]   ;

//****************************************
  ct[0] = ( 1 - 2*(OrnCurr.y()*OrnCurr.y() + OrnCurr.z()*OrnCurr.z()) ) * TorqueTool[0] +
          (     2*(OrnCurr.x()*OrnCurr.y() - OrnCurr.w()*OrnCurr.z()) ) * TorqueTool[1] +
          (     2*(OrnCurr.w()*OrnCurr.y() + OrnCurr.x()*OrnCurr.z()) ) * TorqueTool[2]   ;

  ct[1] = (     2*(OrnCurr.x()*OrnCurr.y() + OrnCurr.w()*OrnCurr.z()) ) * TorqueTool[0] +
          ( 1 - 2*(OrnCurr.x()*OrnCurr.x() + OrnCurr.z()*OrnCurr.z()) ) * TorqueTool[1] +
          (     2*(OrnCurr.y()*OrnCurr.z() - OrnCurr.w()*OrnCurr.x()) ) * TorqueTool[2]   ;

  ct[2] = (     2*(OrnCurr.x()*OrnCurr.z() - OrnCurr.w()*OrnCurr.y()) ) * TorqueTool[0] +
          (     2*(OrnCurr.w()*OrnCurr.x() + OrnCurr.y()*OrnCurr.z()) ) * TorqueTool[1] +
          ( 1 - 2*(OrnCurr.x()*OrnCurr.x() + OrnCurr.y()*OrnCurr.y()) ) * TorqueTool[2]   ;

//****************************************

  computedF =  cf;
  computedT =  ct;

  cfOutputValue->setData(&cf);
  ctOutputValue->setData(&ct);
}

private:
DISALLOW_COPY_AND_ASSIGN(applyForceToolFrame);
};



#endif /*APPLY_FORCE_TOOLFRAME_H_*/
