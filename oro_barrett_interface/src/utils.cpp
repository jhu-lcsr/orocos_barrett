
#include <oro_barrett_interface/utils.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>

#include <rtt/Logger.hpp>

namespace oro_barrett_interface {

  void getSubtree(
      const KDL::Tree &tree,
      KDL::SegmentMap::const_iterator subroot,
      KDL::Tree &subtree)
  {
    const std::string element_name = subroot->first;
    const KDL::TreeElement element = subroot->second;

    // Add the children segments to the subtree
    std::vector<KDL::SegmentMap::const_iterator>::const_iterator it;
    for(it = element.children.begin();
        it != element.children.end();
        ++it)
    {
      const std::string child_name = (*it)->first;
      const KDL::TreeElement child = (*it)->second;

      RTT::log(RTT::Debug)
        << "Adding segment " <<child_name
        << " to parent "<< element_name
        << " with transform: "
        << RTT::endlog();

      // Add this segment to the subtree
      subtree.addSegment(child.segment, element_name);
      getSubtree(tree, *it, subtree);
    }
  }

}
