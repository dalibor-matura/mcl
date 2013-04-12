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

#include "fcl/ccd/interpolation/interpolation.h"

#include <boost/assert.hpp>

namespace fcl 
{

Interpolation::Interpolation(FCL_REAL start_value, FCL_REAL end_value) :
  start_value_(start_value),
  end_value_(end_value),
  is_value_growing_(true),
  max_time_scale_(1.0)
{
  init();
}

void Interpolation::init()
{
  initIsValueGrowing();
}

void Interpolation::initIsValueGrowing()
{
  isValueGrowing(getStartValue() <= getEndValue() );
}

const FCL_REAL& Interpolation::getStartValue() const
{
  return start_value_;
}

const FCL_REAL& Interpolation::getEndValue() const
{
  return end_value_;
}

bool Interpolation::operator == (const Interpolation& interpolation) const
{
  return 
    (this->getType() == interpolation.getType()) &&
    (this->start_value_ == interpolation.start_value_) &&
    (this->end_value_ == interpolation.end_value_);
}

bool Interpolation::operator != (const Interpolation& interpolation) const
{
  return !(*this == interpolation);
}

FCL_REAL& Interpolation::getValue(FCL_REAL time, FCL_REAL& value) const
{
  value = getValue(time);

  return value;
}

bool Interpolation::isValueGrowing() const
{
    return is_value_growing_;
}

void Interpolation::isValueGrowing(const bool is_value_growing)
{
    is_value_growing_ = is_value_growing;
}

void Interpolation::setMaxTimeScale(FCL_REAL max_scale)
{
    BOOST_ASSERT(max_scale >= 0 && "Max Time Scale must be positive value.");

    max_time_scale_ = max_scale;
}

FCL_REAL Interpolation::getMaxTimeScale() const
{
    return max_time_scale_;
}

}
