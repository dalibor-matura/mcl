#ifndef FCL_MODEL_BOUND_H
#define FCL_MODEL_BOUND_H

#include "fcl/data_types.h"
#include "fcl/math/vec_3f.h"
#include "fcl/math/transform.h"

#include <boost/shared_ptr.hpp>

#include <string>
#include <vector>
#include <map>

namespace fcl 
{

class Model;
class Movement;
class Link;
class Joint;
class JointBoundInfo;

class LinkBound 
{
public:
	LinkBound(boost::shared_ptr<const Model> model, boost::shared_ptr<const Movement> movement,
		boost::shared_ptr<const Link> bounded_link);

	// direction must be normalized
	FCL_REAL getMotionBound(const FCL_REAL& time, const Vec3f& direction, 
		const FCL_REAL max_distance_from_joint_center = 0);	

	FCL_REAL getNonDirectionalMotionBound(const FCL_REAL& time, 
		const FCL_REAL max_distance_from_joint_center = 0);	

	Transform3f getBoundedLinkGlobalTransform(const FCL_REAL& time) const;

	boost::shared_ptr<const Link> getBoundedLink() const;

private:	
	void initJointsChain();
	const std::vector<boost::shared_ptr<const Joint> >& getJointsChain() const;

	// get parent joint of bounded link
	boost::shared_ptr<Joint> getLastJoint() const;					

	// joints are ordered from last joint to root joint
	FCL_REAL getJointsChainMotionBound() const;

	// returns motion bound of the joint in its parent's frame 
	FCL_REAL getMotionBoundInParentFrame(const boost::shared_ptr<const Joint>& joint) const;
	FCL_REAL getDirectionalMotionBoundInParentFrame(const boost::shared_ptr<const Joint>& joint) const;
	FCL_REAL getSimpleMotionBoundInParentFrame(const boost::shared_ptr<const Joint>& joint) const;

	FCL_REAL getObjectMotionBoundInLastJointFrame(const FCL_REAL max_distance_from_joint_center) const;

	bool isRoot(const boost::shared_ptr<const Joint>& joint) const;	

	FCL_REAL getAccumulatedAngularBound(const boost::shared_ptr<const Joint>& joint, bool is_directional = false) const;	

	void setCurrentDirection(const Vec3f& direction);
	Vec3f getCurrentDirection() const;

	bool isCurrentDirectionValid() const;

	void setCurrentTime(const FCL_REAL& time);
	FCL_REAL getCurrentTime() const;

	void resetAngularBoundAccumulation();
	void addAngularBoundAccumulation(const FCL_REAL& accumulation) const;
	FCL_REAL getAngularBoundAccumulation() const;	

private:		
	Vec3f direction_;
	FCL_REAL time_;
	FCL_REAL max_distance_; // distance from center of last joint to the furthermost point of the collision object	

	boost::shared_ptr<const Model> model_;
	boost::shared_ptr<const Link> bounded_link_;
	boost::shared_ptr<const Movement> movement_;

	std::vector<boost::shared_ptr<const Joint> > joints_chain_;

	// TODO: find out better solution then using of mutable
	// Maybe could be changed together with adding cache mechanism for all calculations
	mutable FCL_REAL accumulated_angular_bound_;
};

}

#endif 