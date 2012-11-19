#include "fcl/broadphase/broadphase_bruteforce_continues.h"
#include <limits>

namespace fcl
{

void NaiveContinuesCollisionManager::registerObjects(const std::vector<ContinuousCollisionObject*>& other_objs)
{
  std::copy(other_objs.begin(), other_objs.end(), std::back_inserter(objs));
}

void NaiveContinuesCollisionManager::unregisterObject(ContinuousCollisionObject* obj)
{
  objs.remove(obj);
}

void NaiveContinuesCollisionManager::registerObject(ContinuousCollisionObject* obj)
{
  objs.push_back(obj);
}

void NaiveContinuesCollisionManager::setup()
{

}

void NaiveContinuesCollisionManager::update()
{

}

void NaiveContinuesCollisionManager::clear()
{
  objs.clear();
}

void NaiveContinuesCollisionManager::getObjects(std::vector<ContinuousCollisionObject*>& objs_) const
{
  objs_.resize(objs.size());
  std::copy(objs.begin(), objs.end(), objs_.begin());
}

void NaiveContinuesCollisionManager::collide(ContinuousCollisionObject* obj, void* cdata, ContinuousCollisionCallBack callback) const
{
  if(size() == 0) return;

  for(std::list<ContinuousCollisionObject*>::const_iterator it = objs.begin(), end = objs.end(); it != end; ++it)
  {
    if(callback(obj, *it, cdata))
      return;
  }
}

void NaiveContinuesCollisionManager::distance(ContinuousCollisionObject* obj, void* cdata, ContinuousDistanceCallBack callback) const
{
  if(size() == 0) return;

  FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
  for(std::list<ContinuousCollisionObject*>::const_iterator it = objs.begin(), end = objs.end(); 
      it != end; ++it)
  {
    if(obj->getAABB().distance((*it)->getAABB()) < min_dist)
    {
      if(callback(obj, *it, cdata, min_dist))
        return;
    }
  }
}

void NaiveContinuesCollisionManager::collide(void* cdata, ContinuousCollisionCallBack callback) const
{
  if(size() == 0) return;

  for(std::list<ContinuousCollisionObject*>::const_iterator it1 = objs.begin(), end = objs.end(); 
      it1 != end; ++it1)
  {
    std::list<ContinuousCollisionObject*>::const_iterator it2 = it1; it2++;
    for(; it2 != end; ++it2)
    {
      if((*it1)->getAABB().overlap((*it2)->getAABB()))
        if(callback(*it1, *it2, cdata))
          return;
    }
  }
}

void NaiveContinuesCollisionManager::distance(void* cdata, ContinuousDistanceCallBack callback) const
{
  if(size() == 0) return;
  
  FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
  for(std::list<ContinuousCollisionObject*>::const_iterator it1 = objs.begin(), end = objs.end(); it1 != end; ++it1)
  {
    std::list<ContinuousCollisionObject*>::const_iterator it2 = it1; it2++;
    for(; it2 != end; ++it2)
    {
      if((*it1)->getAABB().distance((*it2)->getAABB()) < min_dist)
      {
        if(callback(*it1, *it2, cdata, min_dist))
          return;
      }
    }
  }
}

void NaiveContinuesCollisionManager::collide(BroadPhaseContinuousCollisionManager* other_manager_, void* cdata, ContinuousCollisionCallBack callback) const
{
  NaiveContinuesCollisionManager* other_manager = static_cast<NaiveContinuesCollisionManager*>(other_manager_);
  
  if((size() == 0) || (other_manager->size() == 0)) return;

  if(this == other_manager) 
  {
    collide(cdata, callback);
    return;
  }

  for(std::list<ContinuousCollisionObject*>::const_iterator it1 = objs.begin(), end1 = objs.end(); it1 != end1; ++it1)
  {
    for(std::list<ContinuousCollisionObject*>::const_iterator it2 = other_manager->objs.begin(), end2 = other_manager->objs.end(); it2 != end2; ++it2)
    {
      if((*it1)->getAABB().overlap((*it2)->getAABB()))
        if(callback((*it1), (*it2), cdata))
          return;
    }
  }
}

void NaiveContinuesCollisionManager::distance(BroadPhaseContinuousCollisionManager* other_manager_, void* cdata, ContinuousDistanceCallBack callback) const
{
  NaiveContinuesCollisionManager* other_manager = static_cast<NaiveContinuesCollisionManager*>(other_manager_);

  if((size() == 0) || (other_manager->size() == 0)) return;

  if(this == other_manager)
  {
    distance(cdata, callback);
    return;
  }
  
  FCL_REAL min_dist = std::numeric_limits<FCL_REAL>::max();
  for(std::list<ContinuousCollisionObject*>::const_iterator it1 = objs.begin(), end1 = objs.end(); it1 != end1; ++it1)
  {
    for(std::list<ContinuousCollisionObject*>::const_iterator it2 = other_manager->objs.begin(), end2 = other_manager->objs.end(); it2 != end2; ++it2)
    {
      if((*it1)->getAABB().distance((*it2)->getAABB()) < min_dist)
      {
        if(callback(*it1, *it2, cdata, min_dist))
          return;
      }
    }
  }
}

bool NaiveContinuesCollisionManager::empty() const
{
  return objs.empty();
}


}
