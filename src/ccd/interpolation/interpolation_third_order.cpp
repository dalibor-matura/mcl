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
#include "fcl/ccd/interpolation/third_order_control_points.h"
#include "fcl/ccd/interpolation/third_order_distance.h"
#include "fcl/ccd/interpolation/third_order_derivation.h"

#include <cmath>
#include <algorithm>

#include <boost/assert.hpp>
#include <boost/bind.hpp>

namespace fcl 
{

InterpolationType interpolation_third_order_type = THIRD_ORDER;

InterpolationThirdOrder::InterpolationThirdOrder(const boost::shared_ptr<const InterpolationData>& data,
	FCL_REAL start_value, FCL_REAL end_value): 
	Interpolation(start_value, end_value),		
	abs_entire_distance_(0)
{	
	init(data);
}

void InterpolationThirdOrder::init(const boost::shared_ptr<const InterpolationData>& data)
{
	initData(data);	

	initAbsEntireDistance();
	correctInterpolationData();

	initControlPoints();	
}

void InterpolationThirdOrder::initData(const boost::shared_ptr<const InterpolationData>& data)
{
	const boost::shared_ptr<const InterpolationThirdOrderData> third_order_data = 
		getThirdOrderInterpolationData(data);

	copyData(third_order_data);
}

const boost::shared_ptr<const InterpolationThirdOrderData> 
	InterpolationThirdOrder::getThirdOrderInterpolationData(const boost::shared_ptr<const InterpolationData>& data)
{
	BOOST_ASSERT_MSG(data->getType() == interpolation_third_order_type, "Static cast is not safe.");

	return boost::static_pointer_cast<const InterpolationThirdOrderData>(data);
}

void InterpolationThirdOrder::copyData(const boost::shared_ptr<const InterpolationThirdOrderData>& data)
{
	const InterpolationThirdOrderData& data_value = *(data.get() );

	data_.reset(new InterpolationThirdOrderData(data_value) );
}

void InterpolationThirdOrder::initAbsEntireDistance()
{
	abs_entire_distance_ = fabs(getEndValue() - getStartValue() );	
}

FCL_REAL InterpolationThirdOrder::getAbsEntireDistance() const
{
	return abs_entire_distance_;
}

void InterpolationThirdOrder::correctInterpolationData()
{
	if ( getAbsEntireDistance() == 0.0 )
	{
		return; // no need to change anything
	}

	correctAccelerationByJerk();
	correctAccelerationByDistance();
	correctVelocityByDistance();
}

void InterpolationThirdOrder::correctAccelerationByJerk()
{
	// When jerk is too small to reach maximum acceleration:
	// maximum acceleration will be adjusted.

	if (data_->getMaxJerk() < pow(data_->getMaxAcceleration(), 2) / data_->getMaxVelocity() )
	{
		FCL_REAL new_acceleration = sqrt(data_->getMaxVelocity() * data_->getMaxJerk() );

		data_->setMaxAcceleration(new_acceleration);
	}
}

void InterpolationThirdOrder::correctAccelerationByDistance()
{
	// When distance is too small to reach maximum acceleration:
	// maximum acceleration will be adjusted.

	if (getAbsEntireDistance() < 2 * pow(data_->getMaxAcceleration(), 3) / pow(data_->getMaxJerk(), 2) )
	{
		FCL_REAL new_acceleration = 
			pow( (getAbsEntireDistance() * pow(data_->getMaxJerk(), 2) / 2), 1.0 / 3.0);

		data_->setMaxAcceleration(new_acceleration);
	} 
}

void InterpolationThirdOrder::correctVelocityByDistance()
{
	// When distance is too small to reach maximum velocity:
	// maximum velocity will be adjusted.

	if (getAbsEntireDistance() < data_->getMaxVelocity() * 
		(data_->getMaxVelocity() / data_->getMaxAcceleration() + data_->getMaxAcceleration() / data_->getMaxJerk() ) )
	{
		FCL_REAL new_velocity = 
			pow(data_->getMaxAcceleration(), 2) / (2 * data_->getMaxJerk() ) * 
			(sqrt(1 + 4 * getAbsEntireDistance() * pow(data_->getMaxJerk(), 2) / 
			pow(data_->getMaxAcceleration(), 3) ) -1);

		data_->setMaxVelocity(new_velocity);
	}
}

void InterpolationThirdOrder::initControlPoints()
{
	third_order_control_points_.reset(new ThirdOrderControlPoints(data_, getAbsEntireDistance() ) );

	third_order_distance_.reset(new ThirdOrderDistance(data_, third_order_control_points_) );
	third_order_derivation_.reset(new ThirdOrderDerivation(data_, third_order_control_points_) );
}

FCL_REAL InterpolationThirdOrder::getValue(FCL_REAL time) const
{
	BOOST_ASSERT_MSG(time >= 0 && time <= 1, "Time is out of range [0, 1].");

	scaleTime(time);

	return getStartValue() + getDirectionalDistance(getDistance(time) );
}

FCL_REAL InterpolationThirdOrder::getDistance(const FCL_REAL time) const
{
	BOOST_ASSERT_MSG(time >= 0, "Time must be positive value.");

	if ( getAbsEntireDistance() == 0.0 )
	{
		return 0;
	}

	if (time >= third_order_control_points_->getEntireTime() )
	{
		return getAbsEntireDistance();
	}

	return third_order_distance_->getDistance(time);
}

FCL_REAL InterpolationThirdOrder::getDirectionalDistance(const FCL_REAL distance) const
{
	if (isValueGrowing() )
	{
		return distance;
	}
	else
	{
		return -(distance);
	}
}

FCL_REAL InterpolationThirdOrder::getTimeScale() const
{
	return third_order_control_points_->getEntireTime();
}

FCL_REAL InterpolationThirdOrder::getValueLowerBound() const
{
	if (isValueGrowing() )
	{
		return getStartValue();
	}
	else
	{
		return getEndValue();
	}	
}

FCL_REAL InterpolationThirdOrder::getValueUpperBound() const
{
	if (isValueGrowing() )
	{
		return getEndValue();		
	}
	else
	{
		return getStartValue();
	}	
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
	BOOST_ASSERT_MSG(time >= 0 && time <= 1, "Time is out of range [0, 1].");

	scaleTime(time);

	if (isValueGrowing() )
	{
		return getValueUpperBound() - getValue(time);
	}
	else
	{
		return getValue(time) - getValueLowerBound();
	}
}

FCL_REAL InterpolationThirdOrder::getVelocityBound(FCL_REAL time) const
{
	BOOST_ASSERT_MSG(time >= 0 && time <= 1, "Time is out of range [0, 1].");

	scaleTime(time);

	return third_order_derivation_->getAbsoluteMaxDerivation(time);
}

FCL_REAL InterpolationThirdOrder::scaleTime(FCL_REAL& time) const
{
	time *= getMaxTimeScale();

	return time;
}

}
