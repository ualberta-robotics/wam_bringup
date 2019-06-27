/*
 * visual_path_with_normals.h
 *
 *  Created on: Feb 8, 2016
 *      Author: mas
 */

#ifndef VISUAL_PATH_WITH_NORMALS_H_
#define VISUAL_PATH_WITH_NORMALS_H_


#include <cmath>
#include <cassert>
#include <vector>

#include <Eigen/Core>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/math.h>
#include <barrett/systems/abstract/haptic_object.h>


namespace barrett {
namespace systems {


class VisualHapticPathNormals : public HapticObject {
    BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

    static const double COARSE_STEP = 0.01;
    static const double FINE_STEP = 0.001;  //this was 0.0001, change to let the path following service go outside the loop

public:     System::Output<cp_type> tangentDirectionOutput;
protected:  System::Output<cp_type>::Value* tangentDirectionOutputValue;

public:     System::Output<cp_type> normalDirectionOutput;
protected:  System::Output<cp_type>::Value* normalDirectionOutputValue;

public:
    VisualHapticPathNormals(const std::vector<cp_type>& path, const std::vector<cp_type>& pathNormals,
            const std::string& sysName = "VisualHapticPathNormals") :
        HapticObject(sysName),
        tangentDirectionOutput(this, &tangentDirectionOutputValue),
        normalDirectionOutput (this, &normalDirectionOutputValue),
        nearestIndex(0), spline(NULL), splineNormals(NULL)
    {
        // Sample the path & pathNormals
        cp_type prev = path[0];
        cp_type prevNormals = pathNormals[0];
        for (size_t i = 0; i < path.size(); ++i) {
            if ((path[i] - prev).norm() > COARSE_STEP) {
                coarsePath.push_back(path[i]);
                coarsePathNormals.push_back(pathNormals[i]);
                prev = path[i];
                prevNormals = pathNormals[i];
            }
        }
        spline = new math::Spline<cp_type>(coarsePath);
        splineNormals = new math::Spline<cp_type>(coarsePathNormals);
    }

    virtual ~VisualHapticPathNormals() {
        mandatoryCleanUp();
        delete spline;
        delete splineNormals;
    }

protected:
    virtual void operate() {
        const cp_type& cp = input.getValue();

        // Coarse search
        minDist = (coarsePath[nearestIndex] - cp).norm();
        for (size_t i = 0; i < coarsePath.size(); ++i) {
            double dist = (coarsePath[i] - cp).norm();
            if (dist < minDist) {
                minDist = dist;
                nearestIndex = i;
            }
        }

        // Fine search
        // TODO(dc): Can we do this without relying on Spline's implementation?
        double sNearest = spline->getImplementation()->ss[nearestIndex];
        double sLow = sNearest - COARSE_STEP;
        double sHigh = sNearest + COARSE_STEP;
        for (double s = sLow; s <= sHigh; s += FINE_STEP) {
            double dist = (spline->eval(s) - cp).norm();
            if (dist < minDist) {
                minDist = dist;
                sNearest = s;
            }
        }

        dir = (cp - spline->eval(sNearest)).normalized();       //  d = x - path(sNearest)    ,    dir = normalize(d)

//        normalDir << 0.0 , 0.0 , 1.0 ;
	// normalDir = splineNormals->eval(sNearest);
	normalDir = coarsePathNormals[nearestIndex];
	
        tangentDir = spline->evalDerivative(sNearest).normalized();

        sideDir = -normalDir.cross(tangentDir);                  //  s = n x t       % slideDir=cross-product(normalDir , tangentDir)
        sideForceMag = -1 * minDist * dir.dot(sideDir) ;             // |Fs| = |d| d.s

        //        depthOutputValue->setData(&minDist);
        depthOutputValue->setData(&sideForceMag);
        //        directionOutputValue->setData(&dir);
        directionOutputValue->setData(&sideDir);
        tangentDirectionOutputValue->setData(&tangentDir);
        normalDirectionOutputValue->setData(&normalDir);

    }

    double minDist;
    size_t nearestIndex;
    double sideForceMag;
    double normalForceMag;

    cf_type dir;

    cp_type normalDir;
//    cf_type normalDir;
    cf_type sideDir;

    cp_type tangentDir;

    std::vector<cp_type> coarsePath;
    std::vector<cp_type> coarsePathNormals;
    math::Spline<cp_type>* spline;
    math::Spline<cp_type>* splineNormals;

private:
    DISALLOW_COPY_AND_ASSIGN(VisualHapticPathNormals);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
}


#endif /* VISUAL_PATH_WITH_NORMALS_H_ */
