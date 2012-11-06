#include "fcl/ccd/interpolation/third_order_distance.h"

#include "fcl/ccd/interpolation/interpolation_data.h"
#include "fcl/ccd/interpolation/interpolation_third_order.h"
#include "fcl/ccd/interpolation/third_order_control_points.h"

#include <boost/bind.hpp>

namespace fcl {

ThirdOrderDistance::ThirdOrderDistance(
	const boost::shared_ptr<const InterpolationThirdOrderData>& data,
	const boost::shared_ptr<const ThirdOrderControlPoints>& control_points) :

	data_(data),
	points_(control_points),	
	distance_functions_(std::vector<boost::function<FCL_REAL(FCL_REAL)> >(8) )
{
	init();
}

void ThirdOrderDistance::init()
{
	initDistanceCalculation();
}

void ThirdOrderDistance::initDistanceCalculation()
{
	distance_functions_[0] = boost::bind(&ThirdOrderDistance::getDistance_0_0, this, _1);
	distance_functions_[1] = boost::bind(&ThirdOrderDistance::getDistance_0_1, this, _1);
	distance_functions_[2] = boost::bind(&ThirdOrderDistance::getDistance_1_2, this, _1);
	distance_functions_[3] = boost::bind(&ThirdOrderDistance::getDistance_2_3, this, _1);
	distance_functions_[4] = boost::bind(&ThirdOrderDistance::getDistance_3_4, this, _1);
	distance_functions_[5] = boost::bind(&ThirdOrderDistance::getDistance_4_5, this, _1);
	distance_functions_[6] = boost::bind(&ThirdOrderDistance::getDistance_5_6, this, _1);
	distance_functions_[7] = boost::bind(&ThirdOrderDistance::getDistance_6_7, this, _1);
}

FCL_REAL ThirdOrderDistance::getDistance_0_0(const FCL_REAL time) const
{
	return 0.0;
}

FCL_REAL ThirdOrderDistance::getDistance_0_1(const FCL_REAL time) const
{
	return data_->getMaxJerk() * pow(time - points_->getTimePoint(0), 3) / 6.0;
}

FCL_REAL ThirdOrderDistance::getDistance_1_2(const FCL_REAL time) const
{
	return 0.5 * data_->getMaxAcceleration() * pow(time - points_->getTimePoint(1), 2) + 
		0.5 * data_->getMaxJerk() * pow(points_->getTimePoint(1) - points_->getTimePoint(0), 2) * time +
		points_->getDistancePoint(1);
}

FCL_REAL ThirdOrderDistance::getDistance_2_3(const FCL_REAL time) const
{
	return 0.5 * data_->getMaxAcceleration() * pow(time - points_->getTimePoint(1), 2) -
		data_->getMaxJerk() * pow(time - points_->getTimePoint(2), 3) / 6.0 + 
		0.5 * data_->getMaxJerk() * pow(points_->getTimePoint(1) - points_->getTimePoint(0), 2) * time + 
		points_->getDistancePoint(1);
}

FCL_REAL ThirdOrderDistance::getDistance_3_4(const FCL_REAL time) const
{
	return data_->getMaxVelocity() * (time - points_->getTimePoint(3) ) + 
		points_->getDistancePoint(2);
}

FCL_REAL ThirdOrderDistance::getDistance_4_5(const FCL_REAL time) const
{
	return data_->getMaxVelocity() * (time - points_->getTimePoint(4) ) - 
		data_->getMaxJerk() * pow(time - points_->getTimePoint(4), 3) / 6.0 + 
		points_->getDistancePoint(3);
}

FCL_REAL ThirdOrderDistance::getDistance_5_6(const FCL_REAL time) const
{
	return -0.5 * data_->getMaxAcceleration() * pow(time - points_->getTimePoint(5), 2) + 
		points_->getVelocityPoint(2) * (time - points_->getTimePoint(5) ) + 
		points_->getDistancePoint(4);
}

FCL_REAL ThirdOrderDistance::getDistance_6_7(const FCL_REAL time) const
{
	return -0.5 * data_->getMaxAcceleration() * pow(time - points_->getTimePoint(6), 2) +
		data_->getMaxJerk() * pow(time - points_->getTimePoint(6), 3) / 6.0 + 
		points_->getVelocityPoint(3) * (time - points_->getTimePoint(6) ) + 
		points_->getDistancePoint(5);
}

FCL_REAL ThirdOrderDistance::getDistance(const FCL_REAL time) const
{
	return distance_functions_[points_->getTimeUpperBound(time)](time);
}

}