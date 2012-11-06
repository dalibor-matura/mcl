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

#ifndef FCL_CCD_INTERPOLATION_THIRD_ORDER_H
#define FCL_CCD_INTERPOLATION_THIRD_ORDER_H

#include "fcl/data_types.h"
#include "fcl/ccd/interpolation/interpolation.h"

#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

namespace fcl 
{

class InterpolationData;
class InterpolationThirdOrderData;
class InterpolationFactory;
class ThirdOrderControlPoints;
class ThirdOrderDistance;
class ThirdOrderDerivation;

class InterpolationThirdOrder : public Interpolation
{
public:
	InterpolationThirdOrder(const boost::shared_ptr<const InterpolationData>& data, 
		FCL_REAL start_value, FCL_REAL end_value);

	virtual FCL_REAL getValue(FCL_REAL time) const;

	virtual FCL_REAL getValueLowerBound() const;
	virtual FCL_REAL getValueUpperBound() const;

	virtual InterpolationType getType() const;

	virtual FCL_REAL getMovementLengthBound(FCL_REAL time) const;

	virtual FCL_REAL getVelocityBound(FCL_REAL time) const;

	virtual FCL_REAL getTimeScale() const;

public:
	static boost::shared_ptr<Interpolation> create(const boost::shared_ptr<const InterpolationData>& data,
		FCL_REAL start_value, FCL_REAL end_value);

	static void registerToFactory();

private:
	void init(const boost::shared_ptr<const InterpolationData>& data);	

	void initData(const boost::shared_ptr<const InterpolationData>& data);

	const boost::shared_ptr<const InterpolationThirdOrderData> 
		getThirdOrderInterpolationData(const boost::shared_ptr<const InterpolationData>& data);

	void copyData(const boost::shared_ptr<const InterpolationThirdOrderData>& data);

	void initAbsEntireDistance();
	FCL_REAL getAbsEntireDistance() const;

	// must be called after Absolute Entire Distance is initialized
	void correctInterpolationData();		

	void correctAccelerationByJerk();
	void correctAccelerationByDistance();
	void correctVelocityByDistance();

	// must be called after correctInterpolationData is called
	void initControlPoints();

	FCL_REAL getDistance(const FCL_REAL time) const;
	FCL_REAL getDirectionalDistance(const FCL_REAL distance) const;	

	// scales time to values used by third order interpolation calculations
	FCL_REAL scaleTime(FCL_REAL& time) const;

private:
	boost::shared_ptr<InterpolationThirdOrderData> data_;

	boost::shared_ptr<ThirdOrderControlPoints> third_order_control_points_;
	boost::shared_ptr<ThirdOrderDistance> third_order_distance_;
	boost::shared_ptr<ThirdOrderDerivation> third_order_derivation_;

	FCL_REAL abs_entire_distance_;

};

}

#endif
