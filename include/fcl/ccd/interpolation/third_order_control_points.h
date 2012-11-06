#ifndef FCL_CCD_INTERPOLATION_THIRD_ORDER_CONTROL_POINTS_H
#define FCL_CCD_INTERPOLATION_THIRD_ORDER_CONTROL_POINTS_H

#include "fcl/data_types.h"

#include <vector>

#include <boost/shared_ptr.hpp>

namespace fcl {

class InterpolationThirdOrderData;

class ThirdOrderControlPoints
{
public:
	ThirdOrderControlPoints(const boost::shared_ptr<const InterpolationThirdOrderData>& data, 
		FCL_REAL abs_entire_distance);

	FCL_REAL getTimePoint(const std::size_t index) const;
	FCL_REAL getVelocityPoint(const std::size_t index) const;
	FCL_REAL getDistancePoint(const std::size_t index) const;

	std::size_t getTimeUpperBound(const FCL_REAL time) const;

	FCL_REAL getEntireTime() const;
	FCL_REAL getAbsEntireDistance() const;

private:
	void init();

	void initTimePoints();
	void initDistanceAndVelocityPoints();	

private:
	const boost::shared_ptr<const InterpolationThirdOrderData> data_;	

	std::vector<FCL_REAL> time_point_; // time points from t0 to t7
	std::vector<FCL_REAL> velocity_point_; // velocity values in times t0, ..., t7
	std::vector<FCL_REAL> distance_point_; // distance values in times t0, ..., t7	

	FCL_REAL abs_entire_distance_;
};

}

#endif