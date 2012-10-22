#ifndef JOINT_BOUND_H
#define JOINT_BOUND_H

#include "fcl/data_types.h"
#include "fcl/math/vec_3f.h"

#include <map>

#include <boost/shared_ptr.hpp>

namespace fcl {

class Model;
class ModelConfig;
class Joint;
class Interpolation;

class JointBoundInfo
{
public:
	JointBoundInfo(boost::shared_ptr<Model> model,
		boost::shared_ptr<const ModelConfig> cfg_start, boost::shared_ptr<const ModelConfig> cfg_end);

	void setCurrentTime(const FCL_REAL& time);
	FCL_REAL getCurrentTime() const;

	/* setCurrentTime must be called before any other method is called */

	Vec3f getLinearVelocityBound(const boost::shared_ptr<const Joint>& joint) const;
	FCL_REAL getAbsoluteLinearVelocityBound(const boost::shared_ptr<const Joint>& joint) const;

	Vec3f getAngularVelocityBound(const boost::shared_ptr<const Joint>& joint) const;	
	FCL_REAL getAbsoluteAngularVelocityBound(const boost::shared_ptr<const Joint>& joint) const;	

	FCL_REAL getVectorLengthBound(const boost::shared_ptr<const Joint>& joint_parent, 
		const boost::shared_ptr<const Joint>& joint) const;	

private:
	void initJointsInterpolations();

	FCL_REAL getInterpolationVelocityBound(const boost::shared_ptr<const Joint>& joint) const;
	boost::shared_ptr<Interpolation> getInterpolation(const boost::shared_ptr<const Joint>& joint) const;	

	bool isJointRevolute(const boost::shared_ptr<const Joint>& joint) const;
	bool isJointTranslational(const boost::shared_ptr<const Joint>& joint) const;		

private:
	FCL_REAL time_;

	boost::shared_ptr<Model> model_;
	boost::shared_ptr<const ModelConfig> cfg_start_;
	boost::shared_ptr<const ModelConfig> cfg_end_;

	std::map<std::string, boost::shared_ptr<Interpolation> > joint_interpolation_;
};

}

#endif