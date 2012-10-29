/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Dalibor Matura, Jia Pan */

#include "fcl/ccd/interpolation/interpolation_third_order.h"
#include "fcl/ccd/interpolation/interpolation_factory.h"
#include "fcl/ccd/interpolation/interpolation_data.h"

#include <cmath>

#include <boost/assert.hpp>

namespace fcl 
{

InterpolationType interpolation_third_order_type = THIRD_ORDER;

InterpolationThirdOrder::InterpolationThirdOrder(const boost::shared_ptr<const InterpolationData>& data,
	FCL_REAL start_value, FCL_REAL end_value): 
	Interpolation(start_value, end_value),
	data_(boost::static_pointer_cast<const InterpolationThirdOrderData>(data) ),
	is_value_growing_(true),
	abs_entire_distance_(0),
	time_(std::vector<FCL_REAL>(8, 0) ),
	distance_(std::vector<FCL_REAL>(8, 0) ),
	velocity_(std::vector<FCL_REAL>(8, 0) )
{
	BOOST_ASSERT_MSG(data->getType() == interpolation_third_order_type, "Static cast is not safe.");

	init();
}

FCL_REAL InterpolationThirdOrder::getValue(FCL_REAL time) const
{
  // TODO
  return getStartValue() + (getEndValue() - getStartValue() ) * time;
}

FCL_REAL InterpolationThirdOrder::getValueLowerBound() const
{
  // TODO
  return getStartValue();
}

FCL_REAL InterpolationThirdOrder::getValueUpperBound() const
{
  // TODO
  return getEndValue();
}

InterpolationType InterpolationThirdOrder::getType() const
{
  return interpolation_third_order_type;
}

boost::shared_ptr<Interpolation> InterpolationThirdOrder::create(
	const boost::shared_ptr<const InterpolationData>& data,
	FCL_REAL start_value, FCL_REAL end_value)
{
  return boost::shared_ptr<Interpolation>(new InterpolationThirdOrder(data, start_value, end_value) );
}

void InterpolationThirdOrder::registerToFactory()
{
  InterpolationFactory::instance().registerClass(interpolation_third_order_type, create);
}

FCL_REAL InterpolationThirdOrder::getMovementLengthBound(FCL_REAL time) const
{
  // TODO
  return getValueUpperBound() - getValue(time);
}

FCL_REAL InterpolationThirdOrder::getVelocityBound(FCL_REAL time) const
{
  // TODO
  return (getEndValue() - getStartValue() );
}

void InterpolationThirdOrder::init()
{
	isValueGrowing(getStartValue() <= getEndValue() );

	initTimes();
	initDistancesAndVelocities();
}

bool InterpolationThirdOrder::isValueGrowing() const
{
	return is_value_growing_;
}

void InterpolationThirdOrder::isValueGrowing(const bool& is_value_growing)
{
	is_value_growing_ = is_value_growing;
}

void InterpolationThirdOrder::initAbsEntireDistance()
{
	abs_entire_distance_ = fabs(getEndValue() - getStartValue() );

	modifyInterpolationDataByDistance();
}

void InterpolationThirdOrder::modifyInterpolationDataByDistance()
{
	// TODO: modify InterpolationThirdOrderData by new Absolute Entire Distance
}

const FCL_REAL& InterpolationThirdOrder::getAbsEntireDistance() const
{
	return abs_entire_distance_;
}

void InterpolationThirdOrder::initTimes()
{	
	// time zero
	time_[0] = 0.0;
	// time after first jerk == time needed to accelerate from 0 to "acceleration" + "t0
	time_[1] = data_->getAcceleration() / data_->getJerk() + time_[0]; 
	// time needed to accelerate from "v" at the end of "jerk" to the "velocity" + "t0"
	time_[2] = data_->getVelocity() / data_->getAcceleration() + time_[0];
	// time after second jerk
	time_[3] = time_[2] + (time_[1] - time_[0]); 
	// time needed to move the "distance" with "velocity"
	time_[4] = getAbsEntireDistance() / data_->getVelocity() + time_[0];
	/* the rest is symmetric to the first half */
	time_[5] = time_[4] + (time_[1] - time_[0]); 
	time_[6] = time_[5] + (time_[2] - time_[1]);
	time_[7] = time_[6] + (time_[1] - time_[0]);
}

void InterpolationThirdOrder::initDistancesAndVelocities()
{
	FCL_REAL duration_0_1 = time_[1] - time_[0];
	FCL_REAL duration_0_2 = time_[2] - time_[0];
	FCL_REAL duration_3_4 = time_[4] - time_[3];
	FCL_REAL duration_4_5 = time_[5] - time_[4];
	FCL_REAL duration_5_6 = time_[6] - time_[5];

	// velocity after t1 == velocity after first jerk finished
	velocity_[1] = 0.5 * data_->getJerk() * pow(duration_0_1, 2); 

	velocity_[2] = data_->getVelocity() - 0.5 * data_->getJerk() * pow(duration_4_5, 2);

	velocity_[3] = velocity_[2] - data_->getAcceleration() * duration_5_6;

	// distance after t1 == distance after first jerk finished
	distance_[1] = -0.5 * data_->getJerk() * pow(duration_0_1, 2) * time_[1] + 
		(1 / 6) * data_->getJerk() * pow(duration_0_1, 3);	

	distance_[2] = 0.5 * data_->getAcceleration() * pow(duration_0_2, 2) + 
		0.5 * data_->getJerk() * pow(duration_0_1, 2) * duration_0_2;

	distance_[3] = data_->getVelocity() * duration_3_4 + 
		distance_[2];	

	distance_[4] = data_->getVelocity() * duration_4_5 - 
		(1 / 6) * data_->getJerk() * pow(duration_4_5, 3) + 
		distance_[3];

	distance_[5] = -0.5 * data_->getAcceleration() * pow(duration_5_6, 2) +
		velocity_[2] * duration_5_6 +
		distance_[4];	
}


}
