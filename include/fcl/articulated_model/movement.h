#ifndef JOINT_BOUND_H
#define JOINT_BOUND_H

#include "fcl/data_types.h"
#include "fcl/math/vec_3f.h"
#include "fcl/math/transform.h"

#include <map>
#include <vector>

#include <boost/shared_ptr.hpp>

namespace fcl {

class Model;
class ModelConfig;
class Joint;
class Link;
class Interpolation;

class Movement
{
public:
	Movement(boost::shared_ptr<const Model> model,
		boost::shared_ptr<const ModelConfig> cfg_start, boost::shared_ptr<const ModelConfig> cfg_end);

	boost::shared_ptr<const ModelConfig> getStartCfg() const;
	boost::shared_ptr<const ModelConfig> getEndCfg() const;

	Transform3f getGlobalTransform(const boost::shared_ptr<const Joint>& joint, const FCL_REAL& time) const;
	Transform3f getGlobalTransform(const boost::shared_ptr<const Joint>& joint,
		const boost::shared_ptr<const ModelConfig>& model_cfg) const;	

	Transform3f getGlobalTransform(const boost::shared_ptr<const Link>& link, const FCL_REAL& time) const;
	Transform3f getGlobalTransform(const boost::shared_ptr<const Link>& link,
		const boost::shared_ptr<const ModelConfig>& model_cfg) const;

	boost::shared_ptr<ModelConfig> getModelConfig(const FCL_REAL& time) const;

	// order of joints in returned vector is from last one to root joint
	std::vector<boost::shared_ptr<const Joint> >
		getJointsChainFromLastJoint(const boost::shared_ptr<const Joint>& last_joint) const;

	/* setCurrentTime must be called before any other method is called */

	Vec3f getLinearVelocityBound(const boost::shared_ptr<const Joint>& joint,
		const FCL_REAL& time) const;
	FCL_REAL getAbsoluteLinearVelocityBound(const boost::shared_ptr<const Joint>& joint,
		const FCL_REAL& time) const;

	Vec3f getAngularVelocityBound(const boost::shared_ptr<const Joint>& joint,
		const FCL_REAL& time) const;	
	FCL_REAL getAbsoluteAngularVelocityBound(const boost::shared_ptr<const Joint>& joint,
		const FCL_REAL& time) const;	

	FCL_REAL getChildParentDistanceBound(const boost::shared_ptr<const Joint>& joint,
		const boost::shared_ptr<const Joint>& joint_parent) const;	

private:
	void init();

	void initJointsInterpolations();

	void initChildParentDistanceBounds();

	FCL_REAL calculateChildParentDistanceBound(const boost::shared_ptr<const Joint>& joint,
		const boost::shared_ptr<const Joint>& joint_parent) const;

	FCL_REAL getInterpolationVelocityBound(const boost::shared_ptr<const Joint>& joint,
		const FCL_REAL& time) const;

	const boost::shared_ptr<const Interpolation>& getInterpolation(const std::string& joint_name) const;	
	const boost::shared_ptr<const Interpolation>& getInterpolation(const boost::shared_ptr<const Joint>& joint) const;	

	bool isJointRevolute(const boost::shared_ptr<const Joint>& joint) const;
	bool isJointTranslational(const boost::shared_ptr<const Joint>& joint) const;		

private:
	FCL_REAL time_;

	boost::shared_ptr<const Model> model_;
	boost::shared_ptr<const ModelConfig> cfg_start_;
	boost::shared_ptr<const ModelConfig> cfg_end_;

	std::map<std::string, boost::shared_ptr<const Interpolation> > joint_interpolation_;
	std::map<std::string, FCL_REAL> child_parent_distance_bound_;
};

}

#endif