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

#include "fcl/ccd/interpolation/interpolation_linear.h"
#include "fcl/ccd/interpolation/interpolation_factory.h"
#include "fcl/ccd/interpolation/interpolation_data.h"

#include <boost/assert.hpp>

namespace fcl 
{

InterpolationType interpolation_linear_type = LINEAR;

InterpolationLinear::InterpolationLinear(const boost::shared_ptr<const InterpolationData>& data,
    FCL_REAL start_value, FCL_REAL end_value) :
    Interpolation(start_value, end_value),
    data_(boost::static_pointer_cast<const InterpolationLinearData>(data) )
{
    BOOST_ASSERT(data->getType() == interpolation_linear_type && "Static cast is not safe.");
}

FCL_REAL InterpolationLinear::getValue(FCL_REAL time) const
{
  return getStartValue() + (getEndValue() - getStartValue() ) * time;
}

FCL_REAL InterpolationLinear::getValueLowerBound() const
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

FCL_REAL InterpolationLinear::getValueUpperBound() const
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

InterpolationType InterpolationLinear::getType() const
{
  return interpolation_linear_type;
}

boost::shared_ptr<Interpolation> InterpolationLinear::create(
    const boost::shared_ptr<const InterpolationData>& data,
    FCL_REAL start_value, FCL_REAL end_value)
{
  return boost::shared_ptr<Interpolation>(new InterpolationLinear(data, start_value, end_value) );
}

void InterpolationLinear::registerToFactory()
{
  InterpolationFactory::instance().registerClass(interpolation_linear_type, create);
}

FCL_REAL InterpolationLinear::getMovementLengthBound(FCL_REAL time) const
{
  return getValueUpperBound() - getValue(time);
}

FCL_REAL InterpolationLinear::getVelocityBound(FCL_REAL start_time) const
{
  return (getEndValue() - getStartValue() );
}

FCL_REAL InterpolationLinear::getVelocityBound(FCL_REAL start_time, FCL_REAL end_time) const
{
  return getVelocityBound(start_time);
}

FCL_REAL InterpolationLinear::getTimeScale() const
{
    return 1.0;
}


}
