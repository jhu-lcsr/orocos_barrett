#ifndef __ORO_BARRETT_INTERFACE_UTILS_H__
#define __ORO_BARRETT_INTERFACE_UTILS_H__

#include <kdl/tree.hpp>

namespace oro_barrett_interface {

  /** Get KDL subtree **/
  void getSubtree(
      const KDL::Tree &tree,
      KDL::SegmentMap::const_iterator subroot,
      KDL::Tree &subtree);

}

#endif // ifndef __ORO_BARRETT_INTERFACE_UTILS_H__
