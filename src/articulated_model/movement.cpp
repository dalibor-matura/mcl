#include "fcl/articulated_model/movement.h"

#include "fcl/articulated_model/model.h"
#include "fcl/articulated_model/model_config.h"

#include "fcl/articulated_model/joint.h"
#include "fcl/articulated_model/link.h"
#include "fcl/ccd/interpolation/interpolation.h"
#include "fcl/ccd/interpolation/interpolation_factory.h"

#include <algorithm>

#include <boost/assert.hpp>

namespace fcl 
{

Movement::Movement(boost::shared_ptr<const Model> model,
	boost::shared_ptr<const ModelConfig> cfg_start, boost::shared_ptr<const ModelConfig> cfg_end) :
	model_(model),
	cfg_start_(cfg_start),
	cfg_end_(cfg_end),
	time_(0.0),
	interpolations_max_time_scale_(0.0)
{
	init();
}	

void Movement::init()
{
	initJointsInterpolations();
	initChildParentDistanceBounds();
}

void Movement::initJointsInterpolations()
{
	std::vector<boost::shared_ptr<const fcl::Joint> > joints = model_->getJoints();
	std::vector<boost::shared_ptr<const fcl::Joint> >::iterator it;

	std::map<std::string, boost::shared_ptr<Interpolation> > interpolations;

	for (it = joints.begin(); it != joints.end(); ++it)
	{
		boost::shared_ptr<const Joint>& joint = (*it);

		addNewInterpolation(joint, interpolations);
	}	

	setInterpolationsWithMaxTimeScale(interpolations);	
}

void Movement::addNewInterpolation(const boost::shared_ptr<const Joint>& joint,
	std::map<std::string, boost::shared_ptr<Interpolation> >& interpolations)
{
	// for NOW works just for JT_PRISMATIC, JT_REVOLUTE joint types
	BOOST_ASSERT((joint->getJointType() == JT_PRISMATIC) || (joint->getJointType() == JT_REVOLUTE) &&
		"Joint type not supported yet");

	const std::string& joint_name = joint->getName();	
	boost::shared_ptr<Interpolation> joint_interpolation = createInterpolation(joint_name);

	interpolations[joint_name] = joint_interpolation;

	chooseInterpolationMaxTimeScale(joint_interpolation);
}

boost::shared_ptr<Interpolation> Movement::createInterpolation(const std::string& joint_name) const
{
	boost::shared_ptr<const InterpolationData> joint_interpolation_data = 
		model_->getJointInterpolationData(joint_name);

	const FCL_REAL joint_start_value = cfg_start_->getJointConfig(joint_name).getValue(0);
	const FCL_REAL joint_end_value = cfg_end_->getJointConfig(joint_name).getValue(0);

	return InterpolationFactory::instance().create(
		joint_interpolation_data, joint_start_value, joint_end_value);
}

void Movement::chooseInterpolationMaxTimeScale(const boost::shared_ptr<const Interpolation>& interpolation)
{
	if (interpolation->getTimeScale() > interpolations_max_time_scale_)
	{
		interpolations_max_time_scale_ = interpolation->getTimeScale();
	}	
}

void Movement::setInterpolationsWithMaxTimeScale(
	std::map<std::string, boost::shared_ptr<Interpolation> >& interpolations)
{
	std::map<std::string, boost::shared_ptr<Interpolation> >::iterator it;

	for (it = interpolations.begin(); it != interpolations.end(); ++it)
	{
		const std::string& joint_name = it->first;
		boost::shared_ptr<Interpolation>& joint_interp = it->second;

		joint_interp->setMaxTimeScale(getInterpolationMaxTimeScale() );

		joint_interpolation_[joint_name] = joint_interp;
	}
}

FCL_REAL Movement::getInterpolationMaxTimeScale() const
{
	return interpolations_max_time_scale_;
}

void Movement::initChildParentDistanceBounds()
{
	std::vector<boost::shared_ptr<const fcl::Joint> > joints = model_->getJoints();
	std::vector<boost::shared_ptr<const fcl::Joint> >::iterator it;

	for (it = joints.begin(); it != joints.end(); ++it)
	{
		boost::shared_ptr<const Joint>& joint = (*it);

		boost::shared_ptr<Joint> joint_parent = model_->getJointParent(joint);

		if (joint_parent.use_count() == 0) continue;

		child_parent_distance_bound_[joint->getName() ] = 
			calculateChildParentDistanceBound(joint, joint_parent);	
	}	
}

FCL_REAL Movement::calculateChildParentDistanceBound(const boost::shared_ptr<const Joint>& joint,
	const boost::shared_ptr<const Joint>& joint_parent) const
{
	BOOST_ASSERT(joint_parent.use_count() != 0 && "Joint has no parent");
	BOOST_ASSERT(model_->getJointParent(joint) == joint_parent &&
		"Given joint parent isn't set correctly");

	Vec3f translation = joint->getTransformToParent().getTranslation();

	if (isJointRevolute(joint_parent) )
	{
		return translation.length();
	}

	boost::shared_ptr<const Interpolation> parent_interp = getInterpolation(joint_parent);

	// suggest that vector returned by getAxis() method is normalized
	Vec3f position_for_lower_bound = 
		translation + joint_parent->getAxis() * parent_interp->getValueLowerBound();
	Vec3f position_for_upper_bound =
		translation + joint_parent->getAxis() * parent_interp->getValueUpperBound();
	
	FCL_REAL distance_for_lower_bound = position_for_lower_bound.length();
	FCL_REAL distance_for_upper_bound = position_for_upper_bound.length();

	return std::max(distance_for_lower_bound, distance_for_upper_bound);		
}

const boost::shared_ptr<const Interpolation>& Movement::getInterpolation(const std::string& joint_name) const
{
	std::map<std::string, boost::shared_ptr<const Interpolation> >::const_iterator it = joint_interpolation_.find(joint_name);

	BOOST_ASSERT( (it != joint_interpolation_.end() ) && "Joint name is not valid");

	return it->second;
}

const boost::shared_ptr<const Interpolation>& Movement::getInterpolation(const boost::shared_ptr<const Joint>& joint) const
{
	return getInterpolation(joint->getName() );
}

Vec3f Movement::getLinearVelocityBound(const boost::shared_ptr<const Joint>& joint,
	const FCL_REAL& start_time, const FCL_REAL& end_time) const
{
	if (isJointTranslational(joint) )
	{
		// suggest that vector returned by getAxis() method is normalized
		return joint->getAxis() * getInterpolationVelocityBound(joint, start_time, end_time);
	}
	else
	{
		return Vec3f(0.0, 0.0, 0.0);		
	}
}

FCL_REAL Movement::getInterpolationVelocityBound(const boost::shared_ptr<const Joint>& joint,
	const FCL_REAL& start_time, const FCL_REAL& end_time) const
{
	boost::shared_ptr<const Interpolation> interp = getInterpolation(joint);	

	return interp->getVelocityBound(start_time, end_time);
}


FCL_REAL Movement::getAbsoluteLinearVelocityBound(const boost::shared_ptr<const Joint>& joint,
	const FCL_REAL& start_time, const FCL_REAL& end_time) const
{
	if (isJointTranslational(joint) )
	{
		return std::abs(getInterpolationVelocityBound(joint, start_time, end_time));
	}
	else
	{
		return 0.0;		
	}
}

Vec3f Movement::getAngularVelocityBound(const boost::shared_ptr<const Joint>& joint,
	const FCL_REAL& start_time, const FCL_REAL& end_time) const
{
	if (isJointRevolute(joint) )
	{
		// suggest that vector returned by getAxis() method is normalized
		return joint->getAxis() * getInterpolationVelocityBound(joint, start_time, end_time);
	}
	else
	{
		return Vec3f(0.0, 0.0, 0.0);		
	}
}

FCL_REAL Movement::getAbsoluteAngularVelocityBound(const boost::shared_ptr<const Joint>& joint,
	const FCL_REAL& start_time, const FCL_REAL& end_time) const
{
	if (isJointRevolute(joint) )
	{
		return std::abs(getInterpolationVelocityBound(joint, start_time, end_time) );
	}
	else
	{
		return 0.0;
	}
}

bool Movement::isJointRevolute(const boost::shared_ptr<const Joint>& joint) const
{
	JointType joint_type = joint->getJointType();

	switch (joint_type)
	{
	case JT_PRISMATIC:
		return false;
		break;
	case JT_REVOLUTE:
		return true;
		break;
	}

	return false;
}

bool Movement::isJointTranslational(const boost::shared_ptr<const Joint>& joint) const
{
	return !isJointRevolute(joint);
}

FCL_REAL Movement::getChildParentDistanceBound(const boost::shared_ptr<const Joint>& joint,
	const boost::shared_ptr<const Joint>& joint_parent) const
{		
	BOOST_ASSERT(joint_parent.use_count() != 0 && "Joint has no parent");
	BOOST_ASSERT(model_->getJointParent(joint) == joint_parent &&
		"Given joint parent isn't set correctly");

	std::map<std::string, FCL_REAL>::const_iterator it = 
		child_parent_distance_bound_.find(joint->getName() );

	BOOST_ASSERT(it != child_parent_distance_bound_.end() );

	if (it == child_parent_distance_bound_.end() )
	{		
		return 0;
	}

	return it->second;	
}	

boost::shared_ptr<const ModelConfig> Movement::getStartCfg() const
{
	return cfg_start_;
}

boost::shared_ptr<const ModelConfig> Movement::getEndCfg() const
{
	return cfg_start_;
}

Transform3f Movement::getGlobalTransform(const boost::shared_ptr<const Joint>& joint, 
	const FCL_REAL& time) const
{
	boost::shared_ptr<const ModelConfig> model_cfg = getModelConfig(time);

	return model_->getGlobalTransform(joint, model_cfg);
}

Transform3f Movement::getGlobalTransform(const boost::shared_ptr<const Link>& link, 
	const FCL_REAL& time) const
{
	boost::shared_ptr<const ModelConfig> model_cfg = getModelConfig(time);

	return model_->getGlobalTransform(link, model_cfg);
}

boost::shared_ptr<ModelConfig> Movement::getModelConfig(const FCL_REAL& time) const
{
	boost::shared_ptr<ModelConfig> new_model_config(new ModelConfig(model_) );
	
	getModelConfig(time, new_model_config);

	return new_model_config;
}

void Movement::getModelConfig(const FCL_REAL& time, boost::shared_ptr<ModelConfig>& model_config) const
{
	const std::map<std::string, JointConfig>& joint_cfgs_map = model_config->getJointCfgsMap();

	std::map<std::string, JointConfig>::const_iterator it_first;

	for (it_first = joint_cfgs_map.begin(); it_first != joint_cfgs_map.end(); ++it_first)
	{
		const std::string& joint_name = it_first->first;
		const boost::shared_ptr<const Interpolation>& joint_interpolation = getInterpolation(joint_name);

		JointConfig& joint_cfg = model_config->getJointConfig(joint_name);		

		for (size_t i = 0; i < joint_cfg.getDim(); ++i)
		{
			// TODO: create mechanism that set multiple dimensions joint correctly
			joint_cfg[i] = joint_interpolation->getValue(time);
		}
	}
}

FCL_REAL Movement::getDuration() const
{
	return interpolations_max_time_scale_;
}

boost::shared_ptr<const Model> Movement::getModel() const
{
	return model_;
}

}