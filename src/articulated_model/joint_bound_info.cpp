#include "fcl/articulated_model/joint_bound_info.h"

#include "fcl/articulated_model/model.h"
#include "fcl/articulated_model/model_config.h"

#include "fcl/articulated_model/joint.h"
#include "fcl/ccd/interpolation/interpolation.h"
#include "fcl/ccd/interpolation/interpolation_factory.h"

#include <boost/assert.hpp>

namespace fcl 
{

JointBoundInfo::JointBoundInfo(boost::shared_ptr<Model> model,
	boost::shared_ptr<const ModelConfig> cfg_start, boost::shared_ptr<const ModelConfig> cfg_end) :

	model_(model),
	cfg_start_(cfg_start),
	cfg_end_(cfg_end),
	time_(0.0)
{
	initJointsInterpolations();
}	

void JointBoundInfo::initJointsInterpolations()
{
	std::vector<boost::shared_ptr<fcl::Joint> > joints = model_->getJoints();
	std::vector<boost::shared_ptr<fcl::Joint> >::iterator it;

	for (it = joints.begin(); it != joints.end(); ++it)
	{
		boost::shared_ptr<Joint>& joint = (*it);

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

Vec3f JointBoundInfo::getLinearVelocityBound(boost::shared_ptr<const Joint>& joint) const
{
	if (isJointTranslational(joint) )
	{
		return joint->getAxis() * getInterpolationVelocityBound(joint);
	}
	else
	{
		return Vec3f(0.0, 0.0, 0.0);		
	}
}


FCL_REAL JointBoundInfo::getAbsoluteLinearVelocityBound(boost::shared_ptr<const Joint>& joint) const
{
	if (isJointTranslational(joint) )
	{
		return std::abs(getInterpolationVelocityBound(joint));
	}
	else
	{
		return 0.0;		
	}
}

FCL_REAL JointBoundInfo::getInterpolationVelocityBound(boost::shared_ptr<const Joint>& joint) const
{
	boost::shared_ptr<const Interpolation> interp = getInterpolation(joint);	

	return interp->getVelocityBound(getCurrentTime() );
}

boost::shared_ptr<Interpolation> JointBoundInfo::getInterpolation(boost::shared_ptr<const Joint>& joint) const
{
	std::string joint_name = joint->getName();

	std::map<std::string, boost::shared_ptr<Interpolation> >::const_iterator it = joint_interpolation_.find(joint_name);

	BOOST_ASSERT_MSG((it != joint_interpolation_.end()), "Joint name is not valid");

	return it->second;
}

Vec3f JointBoundInfo::getAngularVelocityBound(boost::shared_ptr<const Joint>& joint) const
{
	if (isJointRevolute(joint) )
	{
		return joint->getAxis() * getInterpolationVelocityBound(joint);
	}
	else
	{
		return Vec3f(0.0, 0.0, 0.0);		
	}
}

FCL_REAL JointBoundInfo::getAbsoluteAngularVelocityBound(boost::shared_ptr<const Joint>& joint) const
{
	FCL_REAL velocity;

	if (isJointRevolute(joint) )
	{
		return std::abs(getInterpolationVelocityBound(joint) );
	}
	else
	{
		return 0.0;
	}

	return velocity;
}

bool JointBoundInfo::isJointRevolute(boost::shared_ptr<const Joint>& joint) const
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

bool JointBoundInfo::isJointTranslational(boost::shared_ptr<const Joint>& joint) const
{
	return !isJointRevolute(joint);
}

FCL_REAL JointBoundInfo::getVectorLengthBound(boost::shared_ptr<const Joint>& joint_parent, boost::shared_ptr<const Joint>& joint) const
{	
	// TODO: ask Jia Pan what exactly getTransformToParent means
	// Is it transformation from joint's coordinates to parent's coordinates
	// or it is transformation from parent's coordinates to joint's coordinates?
	// I suggest the second option in spite of the method's name.
	Vec3f vec = joint->getTransformToParent().getTranslation();

	if (isJointTranslational(joint_parent) )
	{
		boost::shared_ptr<const Interpolation> parent_interp = getInterpolation(joint_parent);

		Vec3f movement = joint_parent->getAxis() * parent_interp->getMovementLengthBound(getCurrentTime() );

		vec += movement;
	}	

	return vec.length();
}

void JointBoundInfo::setCurrentTime(const FCL_REAL& time)
{
	time_ = time;
}

FCL_REAL JointBoundInfo::getCurrentTime() const
{
	return time_;
}	

}