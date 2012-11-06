#include "fcl/ccd/interpolation/third_order_control_points.h"

#include "fcl/ccd/interpolation/interpolation_data.h"

namespace fcl {

ThirdOrderControlPoints::ThirdOrderControlPoints(
	const boost::shared_ptr<const InterpolationThirdOrderData>& data, 
	FCL_REAL abs_entire_distance) :

	data_(data),
	abs_entire_distance_(abs_entire_distance),
	time_point_(std::vector<FCL_REAL>(8, 0) ),
	distance_point_(std::vector<FCL_REAL>(8, 0) ),
	velocity_point_(std::vector<FCL_REAL>(8, 0) )
{
	init();
}

void ThirdOrderControlPoints::init()
{
	initTimePoints();
	initDistanceAndVelocityPoints();
}

void ThirdOrderControlPoints::initTimePoints()
{	
	// time zero
	time_point_[0] = 0.0;
	// time after first jerk == time needed to accelerate from 0 to "acceleration" + "t0
	time_point_[1] = data_->getMaxAcceleration() / data_->getMaxJerk() + time_point_[0]; 
	// time needed to accelerate from "v" at the end of "jerk" to the "velocity" + "t0"
	time_point_[2] = data_->getMaxVelocity() / data_->getMaxAcceleration() + time_point_[0];
	// time after second jerk
	time_point_[3] = time_point_[2] + (time_point_[1] - time_point_[0]); 
	// time needed to move the "distance" with "velocity"
	time_point_[4] = getAbsEntireDistance() / data_->getMaxVelocity() + time_point_[0];
	/* the rest is symmetric to the first half */
	time_point_[5] = time_point_[4] + (time_point_[1] - time_point_[0]); 
	time_point_[6] = time_point_[5] + (time_point_[2] - time_point_[1]);
	time_point_[7] = time_point_[6] + (time_point_[1] - time_point_[0]);
}

FCL_REAL ThirdOrderControlPoints::getAbsEntireDistance() const
{
	return abs_entire_distance_;
}

void ThirdOrderControlPoints::initDistanceAndVelocityPoints()
{
	FCL_REAL duration_0_1 = time_point_[1] - time_point_[0];
	FCL_REAL duration_0_2 = time_point_[2] - time_point_[0];
	FCL_REAL duration_3_4 = time_point_[4] - time_point_[3];
	FCL_REAL duration_4_5 = time_point_[5] - time_point_[4];
	FCL_REAL duration_5_6 = time_point_[6] - time_point_[5];

	// velocity after t1 == velocity after first jerk finished
	velocity_point_[1] = 0.5 * data_->getMaxJerk() * pow(duration_0_1, 2); 

	velocity_point_[2] = data_->getMaxVelocity() - 0.5 * data_->getMaxJerk() * pow(duration_4_5, 2);

	velocity_point_[3] = velocity_point_[2] - data_->getMaxAcceleration() * duration_5_6;

	// distance after t1 == distance after first jerk finished
	distance_point_[1] = -0.5 * data_->getMaxJerk() * pow(duration_0_1, 2) * time_point_[1] + 
		data_->getMaxJerk() * pow(duration_0_1, 3) / 6.0;	

	distance_point_[2] = 0.5 * data_->getMaxAcceleration() * pow(duration_0_2, 2) + 
		0.5 * data_->getMaxJerk() * pow(duration_0_1, 2) * duration_0_2;

	distance_point_[3] = data_->getMaxVelocity() * duration_3_4 + 
		distance_point_[2];	

	distance_point_[4] = data_->getMaxVelocity() * duration_4_5 - 
		data_->getMaxJerk() * pow(duration_4_5, 3) / 6.0 + 
		distance_point_[3];

	distance_point_[5] = -0.5 * data_->getMaxAcceleration() * pow(duration_5_6, 2) +
		velocity_point_[2] * duration_5_6 +
		distance_point_[4];	
}

FCL_REAL ThirdOrderControlPoints::getTimePoint(const std::size_t index) const
{
	BOOST_ASSERT_MSG(index < 8, "Index not in range.");

	return time_point_[index];
}

FCL_REAL ThirdOrderControlPoints::getVelocityPoint(const std::size_t index) const
{
	BOOST_ASSERT_MSG(index < 8, "Index not in range.");

	return velocity_point_[index];
}

FCL_REAL ThirdOrderControlPoints::getDistancePoint(const std::size_t index) const
{
	BOOST_ASSERT_MSG(index < 8, "Index not in range.");

	return distance_point_[index];
}

FCL_REAL ThirdOrderControlPoints::getEntireTime() const
{
	return getTimePoint(7);
}

std::size_t ThirdOrderControlPoints::getTimeUpperBound(const FCL_REAL time) const
{
	BOOST_ASSERT_MSG(time >= 0 , "Time must be positive value.");

	std::vector<FCL_REAL>::const_iterator it;

	for (it = time_point_.begin(); it != time_point_.end(); ++it)
	{
		if (time <= *it)
		{
			return it - time_point_.begin();
		}
	}

	return time_point_.size() - 1;
}

}