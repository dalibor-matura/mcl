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

class LinkBound 
{
public:
	LinkBound(boost::shared_ptr<const Movement> movement, 
		boost::shared_ptr<const Link> bounded_link);

	// direction must be normalized
	FCL_REAL getMotionBound(FCL_REAL start_time, FCL_REAL end_time,
		const Vec3f& direction, const FCL_REAL max_distance_from_joint_center = 0);	

	FCL_REAL getNonDirectionalMotionBound(const FCL_REAL& start_time, 
		const FCL_REAL& end_time, const FCL_REAL max_distance_from_joint_center = 0);	

	Transform3f getBoundedLinkGlobalTransform(const FCL_REAL& time) const;

	boost::shared_ptr<const Link> getBoundedLink() const;

	// get parent joint of bounded link
	boost::shared_ptr<Joint> getLastJoint() const;

private:	
	void initJointsChain();
	const std::vector<boost::shared_ptr<const Joint> >& getJointsChain() const;					

	void sortTimes(FCL_REAL& start_time, FCL_REAL& end_time) const;

	// joints are ordered from last joint to root joint
	FCL_REAL getJointsChainMotionBound() const;

	// returns motion bound of the joint in its parent's frame 
	FCL_REAL getMotionBoundInParentFrame(const boost::shared_ptr<const Joint>& joint) const;
	FCL_REAL getDirectionalMotionBoundInParentFrame(const boost::shared_ptr<const Joint>& joint) const;
	FCL_REAL getSimpleMotionBoundInParentFrame(const boost::shared_ptr<const Joint>& joint) const;

	FCL_REAL getObjectMotionBoundInLastJointFrame(const FCL_REAL max_distance_from_joint_center) const;

	bool isRoot(const boost::shared_ptr<const Joint>& joint) const;	

	FCL_REAL getAccumulatedAngularBound(const boost::shared_ptr<const Joint>& joint, bool is_directional = false) const;	
	FCL_REAL getAccumulatedAngularBoundForLastJointFrame(const boost::shared_ptr<const Joint>& joint) const;

	void setCurrentDirection(const Vec3f& direction);
	Vec3f getCurrentDirection() const;

	bool isCurrentDirectionValid() const;

	void setStartTime(const FCL_REAL& time);
	FCL_REAL getStartTime() const;

	void setEndTime(const FCL_REAL& time);
	FCL_REAL getEndTime() const;

	void setLastStartTime(const FCL_REAL& time);
	void setLastEndTime(const FCL_REAL& time);

	void setLastDirection(const Vec3f& direction);

	bool areTimesCashed() const;

	void resetAngularBoundAccumulation();
	void addAngularBoundAccumulation(const FCL_REAL& accumulation) const;
	FCL_REAL getAngularBoundAccumulation() const;	

private:		
	Vec3f direction_;

	FCL_REAL start_time_;
	FCL_REAL end_time_;	

	boost::shared_ptr<const Model> model_;
	boost::shared_ptr<const Link> bounded_link_;
	boost::shared_ptr<const Movement> movement_;

	std::vector<boost::shared_ptr<const Joint> > joints_chain_;

	// TODO: find out better solution then using of mutable
	// Maybe could be changed together with adding cache mechanism for all calculations
	mutable FCL_REAL accumulated_angular_bound_;

	FCL_REAL last_start_time_;
	FCL_REAL last_end_time_;

	Vec3f last_direction_;

	bool start_time_cached_;
	bool end_time_cached_;

	bool direction_cached_;

	FCL_REAL cashed_joints_chain_motion_bound_;
};

}

#endif 