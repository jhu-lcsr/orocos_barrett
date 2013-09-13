#ifndef __ORO_BARRETT_INTERFACE_HAND_DEVICE_H
#define __ORO_BARRETT_INTERFACE_HAND_DEVICE_H

namespace oro_barrett_interface {

  struct HandDevice {
    // Configuration
    std::vector<std::string> joint_names;
    Eigen::Vector4d resolver_ranges;
  };

}

#endif // ifndef __ORO_BARRETT_INTERFACE_HAND_DEVICE_H
