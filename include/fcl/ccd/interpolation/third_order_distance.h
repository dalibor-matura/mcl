#ifndef FCL_CCD_INTERPOLATION_THIRD_ORDER_DISTANCE_H
#define FCL_CCD_INTERPOLATION_THIRD_ORDER_DISTANCE_H

#include "fcl/data_types.h"

#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

namespace fcl {

class ThirdOrderControlPoints;
class InterpolationThirdOrderData;

class ThirdOrderDistance
{
public:
	ThirdOrderDistance(const boost::shared_ptr<const InterpolationThirdOrderData>& data,
		const boost::shared_ptr<const ThirdOrderControlPoints>& control_points);

	FCL_REAL getDistance(const FCL_REAL time) const;
private:
	void init();

	void initDistanceCalculation();

	FCL_REAL getDistance_0_0(const FCL_REAL time) const;
	FCL_REAL getDistance_0_1(const FCL_REAL time) const;
	FCL_REAL getDistance_1_2(const FCL_REAL time) const;
	FCL_REAL getDistance_2_3(const FCL_REAL time) const;
	FCL_REAL getDistance_3_4(const FCL_REAL time) const;
	FCL_REAL getDistance_4_5(const FCL_REAL time) const;
	FCL_REAL getDistance_5_6(const FCL_REAL time) const;
	FCL_REAL getDistance_6_7(const FCL_REAL time) const;

private:
	const boost::shared_ptr<const ThirdOrderControlPoints> points_;
	const boost::shared_ptr<const InterpolationThirdOrderData> data_;

	std::vector<boost::function<FCL_REAL(FCL_REAL)> > distance_functions_;
};

}

#endif