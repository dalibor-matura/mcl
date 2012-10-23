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
	time_(0.0)
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

	for (it = joints.begin(); it != joints.end(); ++it)
	{
		boost::shared_ptr<const Joint>& joint = (*it);

		// for NOW works just for JT_PRISMATIC, JT_REVOLUTE joint types
		BOOST_ASSERT_MSG((joint->getJointType() == JT_PRISMATIC) || (joint->getJointType() == JT_REVOLUTE),
			"Joint type not supported yet");

		const std::string& joint_name = joint->getName();
		const InterpolationType joint_interpolation_type = model_->getJointInterpolationType(joint_name);

		const FCL_REAL joint_start_value = cfg_start_->getJointConfig(joint_name).getValue(0);
		const FCL_REAL joint_end_value = cfg_end_->getJointConfig(joint_name).getValue(0);

		joint_interpolation_[joint_name] =
			InterpolationFactory::instance().create(joint_interpolation_type, joint_start_value, joint_end_value);
	}	
}

void Movement::initChildParentDistanceBounds()
{
	std::vector<boost::shared_ptr<const fcl::Joint> > joints = model_->getJoints();
	std::vector<boost::shared_ptr<const fcl::Joint> >::iterator it;

	for (it = joints.begin(); it != joints.end(); ++it)
	{
		boost::shared_ptr<const Joint>& joint = (*it);

		boost::shared_ptr<Interpolation> interpolation = getInterpolation(joint);
		boost::shared_ptr<Joint> joint_parent = model_->getJointParent(joint);

		if (joint_parent.use_count() == 0)
		{
			continue;
		}

		child_parent_distance_bound_[joint->getName() ] = 
			calculateChildParentDistanceBound(joint, joint_parent);	
	}	
}

FCL_REAL Movement::calculateChildParentDistanceBound(const boost::shared_ptr<const Joint>& joint,
	const boost::shared_ptr<const Joint>& joint_parent) const
{
	BOOST_ASSERT_MSG(joint_parent.use_count() != 0, "Joint has no parent");
	BOOST_ASSERT_MSG(model_->getJointParent(joint) == joint_parent,
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

boost::shared_ptr<Interpolation> Movement::getInterpolation(const boost::shared_ptr<const Joint>& joint) const
{
	std::string joint_name = joint->getName();

	std::map<std::string, boost::shared_ptr<Interpolation> >::const_iterator it = joint_interpolation_.find(joint_name);

	BOOST_ASSERT_MSG((it != joint_interpolation_.end()), "Joint name is not valid");

	return it->second;
}

Vec3f Movement::getLinearVelocityBound(const boost::shared_ptr<const Joint>& joint,
	const FCL_REAL& time) const
{
	if (isJointTranslational(joint) )
	{
		// suggest that vector returned by getAxis() method is normalized
		return joint->getAxis() * getInterpolationVelocityBound(joint, time);
	}
	else
	{
		return Vec3f(0.0, 0.0, 0.0);		
	}
}

FCL_REAL Movement::getInterpolationVelocityBound(const boost::shared_ptr<const Joint>& joint,
	const FCL_REAL& time) const
{
	boost::shared_ptr<const Interpolation> interp = getInterpolation(joint);	

	return interp->getVelocityBound(time);
}


FCL_REAL Movement::getAbsoluteLinearVelocityBound(const boost::shared_ptr<const Joint>& joint,
	const FCL_REAL& time) const
{
	if (isJointTranslational(joint) )
	{
		return std::abs(getInterpolationVelocityBound(joint, time));
	}
	else
	{
		return 0.0;		
	}
}

Vec3f Movement::getAngularVelocityBound(const boost::shared_ptr<const Joint>& joint,
	const FCL_REAL& time) const
{
	if (isJointRevolute(joint) )
	{
		// suggest that vector returned by getAxis() method is normalized
		return joint->getAxis() * getInterpolationVelocityBound(joint, time);
	}
	else
	{
		return Vec3f(0.0, 0.0, 0.0);		
	}
}

FCL_REAL Movement::getAbsoluteAngularVelocityBound(const boost::shared_ptr<const Joint>& joint,
	const FCL_REAL& time) const
{
	FCL_REAL velocity;

	if (isJointRevolute(joint) )
	{
		return std::abs(getInterpolationVelocityBound(joint, time) );
	}
	else
	{
		return 0.0;
	}

	return velocity;
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
	BOOST_ASSERT_MSG(joint_parent.use_count() != 0, "Joint has no parent");
	BOOST_ASSERT_MSG(model_->getJointParent(joint) == joint_parent,
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

std::vector<boost::shared_ptr<const Joint> >
	Movement::getJointsChainFromLastJoint(const boost::shared_ptr<const Joint>& last_joint) const
{
	std::vector<boost::shared_ptr<const Joint> > joints_chain;

	boost::shared_ptr<const Joint> joint = last_joint;

	while (joint.use_count() != 0)
	{
		joints_chain.push_back(joint);

		joint = model_->getJointParent(joint);
	}

	return joints_chain;
}

Transform3f Movement::getGlobalTransform(const boost::shared_ptr<const Joint>& joint,
	const boost::shared_ptr<const ModelConfig>& model_cfg) const
{
	std::vector<boost::shared_ptr<const Joint> > joints_chain = getJointsChainFromLastJoint(joint);
	std::vector<boost::shared_ptr<const Joint> >::const_iterator it;

	Transform3f global_transform;
	global_transform.setIdentity();

	for (it = joints_chain.begin(); it != joints_chain.end(); ++it)
	{
		const boost::shared_ptr<const Joint>& joint = (*it);
		const JointConfig& joint_cfg = model_cfg->getJointConfig(joint);

		global_transform *= joint->getLocalTransform(joint_cfg);
	}

	return global_transform;
}

Transform3f Movement::getGlobalTransform(const boost::shared_ptr<const Link>& link,
	const boost::shared_ptr<const ModelConfig>& model_cfg) const
{
	return getGlobalTransform(link->getParentJoint(), model_cfg);
}

boost::shared_ptr<ModelConfig> Movement::getModelConfig(const FCL_REAL& time) const
{
	boost::shared_ptr<ModelConfig> model_config(new ModelConfig(model_) );

	return model_config;
}

}