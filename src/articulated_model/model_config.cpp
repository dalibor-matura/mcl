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

#include "fcl/articulated_model/model_config.h"
#include "fcl/articulated_model/model.h"
#include "fcl/articulated_model/joint.h"
#include <algorithm>

#include <boost/assert.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/make_shared.hpp>

namespace fcl
{

ModelConfig::ModelConfig() {}

ModelConfig::ModelConfig(const ModelConfig& model_cfg) :
  model_(model_cfg.model_),
  joint_cfgs_map_(model_cfg.joint_cfgs_map_)
{}

ModelConfig::ModelConfig(boost::shared_ptr<const Model> model) :
  model_(model)
{
  initJointCFGsMap(model->getJointsMap() ); 
}

ModelConfig::ModelConfig(std::map<std::string, boost::shared_ptr<Joint> > joints_map)
{
  initJointCFGsMap(joints_map);
}

void ModelConfig::initJointCFGsMap(const std::map<std::string, boost::shared_ptr<Joint> >& joints_map)
{
	std::map<std::string, boost::shared_ptr<Joint> >::const_iterator it;
	for(it = joints_map.begin(); it != joints_map.end(); ++it)
		joint_cfgs_map_[it->first] = JointConfig(it->second);
}

JointConfig ModelConfig::getJointConfig(const std::string& joint_name) const
{
  std::map<std::string, JointConfig>::const_iterator it = joint_cfgs_map_.find(joint_name);
  BOOST_ASSERT((it != joint_cfgs_map_.end()) && "Joint name not valid");

  return it->second;
}

JointConfig& ModelConfig::getJointConfig(const std::string& joint_name)
{
  std::map<std::string, JointConfig>::iterator it = joint_cfgs_map_.find(joint_name);
  BOOST_ASSERT((it != joint_cfgs_map_.end()) && "Joint name not valid");

  return it->second;
}

JointConfig ModelConfig::getJointConfig(const boost::shared_ptr<const Joint>& joint) const
{
  return getJointConfig(joint->getName());
}

JointConfig& ModelConfig::getJointConfig(const boost::shared_ptr<const Joint>& joint)
{
  return getJointConfig(joint->getName());
}

bool ModelConfig::operator==(const ModelConfig& model_config) const
{
	return getJointCfgsMap() == model_config.getJointCfgsMap();
}

bool ModelConfig::operator!=(const ModelConfig& model_config) const
{
	return !(*this == model_config);
}

boost::shared_ptr<const Model> ModelConfig::getModel() const
{
	return model_;
}

template<typename T>
ModelConfig applyFunction(const ModelConfig& first, const ModelConfig& second, boost::function<T (T,T)> function)
{
	ModelConfig new_model_config(first.getModel() );

	BOOST_ASSERT(first.getModel() == second.getModel() && "Can't add up model configuration of different models");

	if (first.getModel() != second.getModel() )
	{
		return new_model_config;
	}

	const std::map<std::string, JointConfig>& cfg_map_first = first.getJointCfgsMap();
	const std::map<std::string, JointConfig>& cfg_map_second = second.getJointCfgsMap();

	std::map<std::string, JointConfig>::const_iterator it_first;
	std::map<std::string, JointConfig>::const_iterator it_second;

	for (it_first = cfg_map_first.begin(), it_second = cfg_map_second.begin();
		it_first != cfg_map_first.end(); 
		++it_first, ++it_second)
	{
		const std::string& joint_name = it_first->first;
		const JointConfig& first_cfg_ = it_first->second;
		const JointConfig& second_cfg = it_second->second;

		JointConfig& new_joint_cfg = new_model_config.getJointConfig(joint_name);

		for (size_t i = 0; i < new_joint_cfg.getDim(); ++i)
		{
			new_joint_cfg[i] = function(first_cfg_[i], second_cfg[i]);
		}
	}

	return new_model_config;
}

ModelConfig ModelConfig::operator+(const ModelConfig& model_config) const
{
	return applyFunction<FCL_REAL>(*this, model_config, (boost::lambda::_1 + boost::lambda::_2) );
}

ModelConfig ModelConfig::operator-(const ModelConfig& model_config) const
{
	return applyFunction<FCL_REAL>(*this, model_config, (boost::lambda::_1 - boost::lambda::_2) );
}

template<typename T>
ModelConfig applyFunction(const ModelConfig& first, boost::function<T (T)> function)
{
	ModelConfig new_model_config(first.getModel() );

	const std::map<std::string, JointConfig>& cfg_map_first = first.getJointCfgsMap();

	std::map<std::string, JointConfig>::const_iterator it_first;

	for (it_first = cfg_map_first.begin(); it_first != cfg_map_first.end(); ++it_first)
	{
		const std::string& joint_name = it_first->first;
		const JointConfig& first_cfg = it_first->second;

		JointConfig& new_joint_cfg = new_model_config.getJointConfig(joint_name);

		for (size_t i = 0; i < new_joint_cfg.getDim(); ++i)
		{
			new_joint_cfg[i] = function(first_cfg[i]);
		}
	}

	return new_model_config;
}

ModelConfig ModelConfig::operator/(const FCL_REAL& number) const
{
	return applyFunction<FCL_REAL>(*this, (boost::lambda::_1 / number) );
}

boost::shared_ptr<ModelConfig> operator+(const boost::shared_ptr<const ModelConfig>& first,
	const boost::shared_ptr<const ModelConfig>& second)
{	
	return boost::make_shared<ModelConfig>( (*first) + (*second) );
}

boost::shared_ptr<ModelConfig> operator-(const boost::shared_ptr<const ModelConfig>& first,
	const boost::shared_ptr<const ModelConfig>& second)
{
	return boost::make_shared<ModelConfig>( (*first) - (*second) );
}

boost::shared_ptr<ModelConfig> operator/(const boost::shared_ptr<const ModelConfig>& model_cfg,
	const FCL_REAL& number)
{
	return boost::make_shared<ModelConfig>( (*model_cfg) / number );
}

}
