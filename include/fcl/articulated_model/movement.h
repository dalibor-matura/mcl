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
		boost::shared_ptr<const ModelConfig> cfg_start, 
		boost::shared_ptr<const ModelConfig> cfg_end);

	boost::shared_ptr<const ModelConfig> getStartCfg() const;
	boost::shared_ptr<const ModelConfig> getEndCfg() const;

	Transform3f getGlobalTransform(const boost::shared_ptr<const Joint>& joint, 
		const FCL_REAL& time) const;		

	Transform3f getGlobalTransform(const boost::shared_ptr<const Link>& link, 
		const FCL_REAL& time) const;

	boost::shared_ptr<ModelConfig> getModelConfig(const FCL_REAL& time) const;
	void getModelConfig(const FCL_REAL& time, boost::shared_ptr<ModelConfig>& model_config) const;	

	/* setCurrentTime must be called before any other method is called */

	Vec3f getLinearVelocityBound(const boost::shared_ptr<const Joint>& joint,
		const FCL_REAL& start_time, const FCL_REAL& end_time) const;
	FCL_REAL getAbsoluteLinearVelocityBound(const boost::shared_ptr<const Joint>& joint,
		const FCL_REAL& start_time, const FCL_REAL& end_time) const;

	Vec3f getAngularVelocityBound(const boost::shared_ptr<const Joint>& joint,
		const FCL_REAL& start_time, const FCL_REAL& end_time) const;	
	FCL_REAL getAbsoluteAngularVelocityBound(const boost::shared_ptr<const Joint>& joint,
		const FCL_REAL& start_time, const FCL_REAL& end_time) const;	

	FCL_REAL getChildParentDistanceBound(const boost::shared_ptr<const Joint>& joint,
		const boost::shared_ptr<const Joint>& joint_parent) const;	

	FCL_REAL getDuration() const;

	boost::shared_ptr<const Model> getModel() const;

private:
	void init();

	void initJointsInterpolations();

	void addNewInterpolation(const boost::shared_ptr<const Joint>& joint,
		std::map<std::string, boost::shared_ptr<Interpolation> >& interpolations);

	boost::shared_ptr<Interpolation> createInterpolation(const std::string& joint_name) const;

	void chooseInterpolationMaxTimeScale(const boost::shared_ptr<const Interpolation>& interpolation);
	FCL_REAL getInterpolationMaxTimeScale() const;	

	void setInterpolationsWithMaxTimeScale(
		std::map<std::string, boost::shared_ptr<Interpolation> >& interpolations);

	void initChildParentDistanceBounds();

	FCL_REAL calculateChildParentDistanceBound(const boost::shared_ptr<const Joint>& joint,
		const boost::shared_ptr<const Joint>& joint_parent) const;

	FCL_REAL getInterpolationVelocityBound(const boost::shared_ptr<const Joint>& joint,
		const FCL_REAL& start_time, const FCL_REAL& end_time) const;

	const boost::shared_ptr<const Interpolation>& getInterpolation(const std::string& joint_name) const;	
	const boost::shared_ptr<const Interpolation>& getInterpolation(const boost::shared_ptr<const Joint>& joint) const;	

	bool isJointRevolute(const boost::shared_ptr<const Joint>& joint) const;
	bool isJointTranslational(const boost::shared_ptr<const Joint>& joint) const;	

private:
	FCL_REAL time_;

	boost::shared_ptr<const Model> model_;
	boost::shared_ptr<const ModelConfig> cfg_start_;
	boost::shared_ptr<const ModelConfig> cfg_end_;

	FCL_REAL interpolations_max_time_scale_;

	std::map<std::string, boost::shared_ptr<const Interpolation> > joint_interpolation_;
	std::map<std::string, FCL_REAL> child_parent_distance_bound_;
};

}

#endif