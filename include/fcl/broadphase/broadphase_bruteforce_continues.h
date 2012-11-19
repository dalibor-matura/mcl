#ifndef FCL_BROAD_PHASE_BRUTE_FORCE_CONTINUES_H
#define FCL_BROAD_PHASE_BRUTE_FORCE_CONTINUES_H

#include "fcl/broadphase/broadphase.h"
#include <list>

namespace fcl
{

/// @brief Brute force N-body collision manager
class NaiveContinuesCollisionManager : public BroadPhaseContinuousCollisionManager
{
public:
  NaiveContinuesCollisionManager() {}

  /// @brief add objects to the manager
  void registerObjects(const std::vector<ContinuousCollisionObject*>& other_objs);

  /// @brief add one object to the manager
  void registerObject(ContinuousCollisionObject* obj);

  /// @brief remove one object from the manager
  void unregisterObject(ContinuousCollisionObject* obj);

  /// @brief initialize the manager, related with the specific type of manager
  void setup();

  /// @brief update the condition of manager
  void update();

  /// @brief clear the manager
  void clear();

  /// @brief return the objects managed by the manager
  void getObjects(std::vector<ContinuousCollisionObject*>& objs) const;

  /// @brief perform collision test between one object and all the objects belonging to the manager
  void collide(ContinuousCollisionObject* obj, void* cdata, ContinuousCollisionCallBack callback) const;

  /// @brief perform distance computation between one object and all the objects belonging to the manager
  void distance(ContinuousCollisionObject* obj, void* cdata, ContinuousDistanceCallBack callback) const;

  /// @brief perform collision test for the objects belonging to the manager (i.e., N^2 self collision)
  void collide(void* cdata, ContinuousCollisionCallBack callback) const;

  /// @brief perform distance test for the objects belonging to the manager (i.e., N^2 self distance)
  void distance(void* cdata, ContinuousDistanceCallBack callback) const;

  /// @brief perform collision test with objects belonging to another manager
  void collide(BroadPhaseContinuousCollisionManager* other_manager, void* cdata, ContinuousCollisionCallBack callback) const;

  /// @brief perform distance test with objects belonging to another manager
  void distance(BroadPhaseContinuousCollisionManager* other_manager, void* cdata, ContinuousDistanceCallBack callback) const;

  /// @brief whether the manager is empty
  bool empty() const;
  
  /// @brief the number of objects managed by the manager
  inline size_t size() const { return objs.size(); }

protected:

  /// @brief objects belonging to the manager are stored in a list structure
  std::list<ContinuousCollisionObject*> objs;
};


}

#endif
