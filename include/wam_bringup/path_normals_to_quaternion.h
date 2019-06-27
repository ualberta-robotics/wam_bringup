/*
 * path_normals_to_quaternion.h
 *
 *  Created on: Feb 8, 2016
 *      Author: mas
 */

#ifndef PATH_NORMALS_TO_QUATERNION_H_
#define PATH_NORMALS_TO_QUATERNION_H_


#include <cmath>
#include <cassert>
#include <vector>

#include <Eigen/Core>
#include <barrett/systems.h>
#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/math.h>


namespace barrett {
namespace systems {
/*------------------------------------------------------------
 System get the tangential & normal direction of the path
 and compute the orientation of the robot
 first get the Rotation matrix and the convert to quaternion
-------------------------------------------------------------*/
template<size_t DOF>
class computeQuaternionSetPoint : public systems::System
{
BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

// IO  (inputs)
public:
Input<cp_type> tangentDirInput;
Input<cp_type> normalDirInput;

// IO  (outputs)
public:
Output<Eigen::Quaterniond> OrnOutput;     // orientation output (quaternion)

protected:
typename Output<Eigen::Quaterniond>::Value* quaterniondOutputValue;

public:
Eigen::Quaterniond computedQuaternion;          // can be used to display later in the program

public:
computeQuaternionSetPoint(
      const std::string& sysName = "computeQuaternionSetPoint") :
      System(sysName), tangentDirInput(this), normalDirInput(this),
      OrnOutput(this, &quaterniondOutputValue)
{
}
virtual ~computeQuaternionSetPoint()
{
this->mandatoryCleanUp();
}
protected:
    cp_type tanDir;
    cp_type norDir;

    cp_type sideDir;
    Eigen::Quaterniond Orn;

virtual void operate()
{

  tanDir = tangentDirInput.getValue();
  norDir = normalDirInput.getValue();

  sideDir = tanDir.cross(-norDir);      // s = t x (-n)  ... X1 = Y1 x Z1 (define the X1 frame)

  // R: rotation matrix to orient the tool to the orientation parallel to normal (Z1 which is -n) aligned with Y1
  // R = [X1 | Y1 | Z1 ]

  /*

  float tr1 = 1.0 + sideDir[0] - tanDir[1] - (-norDir[2]);
  float tr2 = 1.0 - sideDir[0] + tanDir[1] - (-norDir[2]);
  float tr3 = 1.0 - sideDir[0] - tanDir[1] + (-norDir[2]);
  double qw, qx, qy, qz;
  if ( (tr1 > tr2) && (tr1 > tr3) )
  {
    float S = sqrt(tr1) * 2; // S=4*qx
    qw = (tanDir[2] - (-norDir[1])) / S;
    qx = 0.25 * S;
    qy = (tanDir[0] + sideDir[1]) / S;
    qz = ((-norDir[0]) + sideDir[2]) / S;
  }
  else if ( (tr2 > tr1) && (tr2 > tr3) )
  {
    float S = sqrt(tr2) * 2; // S=4*qy
    qw = ((-norDir[0]) - sideDir[2]) / S;
    qx = (tanDir[0] + sideDir[1]) / S;
    qy = 0.25 * S;
    qz = ((-norDir[1]) + tanDir[2]) / S;
  }
  else
  {
    float S = sqrt(tr3) * 2; // S=4*qz
    qw = (sideDir[1] - tanDir[0]) / S;
    qx = ((-norDir[0]) + sideDir[2]) / S;
    qy = ((-norDir[1]) + tanDir[2]) / S;
    qz = 0.25 * S;
  }

  Orn.x() = qx;
  Orn.y() = qy;
  Orn.z() = qz;
  Orn.w() = qw;
  */

  cp_type u , v , w;


  u = -norDir;
  u = u.normalized() ;

  // v: robot base ref frame  (0,0,1)
  v(0) = 0.0;
  v(1) = 0.0;
  v(2) = 1.0;

  float real_part = 1.0f + u.dot(v);

  if (real_part < 1.e-6f )
      {
          /* If u and v are exactly opposite, rotate 180 degrees
           * around an arbitrary orthogonal axis. Axis normalisation
           * can happen later, when we normalise the quaternion. */
          real_part = 0.0f;
          if ( abs( u(0) )  >  abs ( u(2) ) ) {
              w(0) = -u(0);
              w(1) =  u(1);
              w(2) =  0.f ;
          }
          else {
              w(0) =  0.f ;
              w(1) = -u(2);
              w(2) =  u(1);
          }
      }
  else
      {
          /* Otherwise, build quaternion the standard way. */
	  w = v.cross(u);
      }


  Orn.x() = -w(0);
  Orn.y() = -w(1);
  Orn.z() = -w(2);
  Orn.w() = real_part;



  Orn = Orn.normalized();
  computedQuaternion = Orn;
  quaterniondOutputValue->setData(&Orn);

}

private:
DISALLOW_COPY_AND_ASSIGN(computeQuaternionSetPoint);
};
// -----------------------------------




}
}


#endif /* PATH_NORMALS_TO_QUATERNION_H_ */
