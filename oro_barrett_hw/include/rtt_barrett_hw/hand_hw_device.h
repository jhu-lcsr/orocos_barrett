#ifndef __ORO_BARRETT_HW_HAND_HW_DEVICE
#define __ORO_BARRETT_HW_HAND_HW_DEVICE

#include <barrett/systems.h>

#include <oro_barrett_interface/hand_device.h>

namespace oro_barrett_hw {
  // State structure for a Hand
  struct HandHWDevice 
  {
    boost::shared_ptr<barrett::Hand> interface;
  };

}

#endif // ifnedf __ORO_BARRETT_HW_HAND_DEVICE
