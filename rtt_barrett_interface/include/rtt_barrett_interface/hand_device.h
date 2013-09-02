#ifndef __RTT_BARRETT_INTERFACE_HAND_DEVICE_H
#define __RTT_BARRETT_INTERFACE_HAND_DEVICE_H

namespace rtt_barrett_interface {

  struct HandDevice {
    // Configuration
    std::vector<std::string> joint_names;
    Eigen::Vector4d resolver_ranges;
  };

}

#endif // ifndef __RTT_BARRETT_INTERFACE_HAND_DEVICE_H

