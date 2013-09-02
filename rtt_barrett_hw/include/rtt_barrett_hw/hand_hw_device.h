#ifndef __RTT_BARRETT_HW_HAND_HW_DEVICE
#define __RTT_BARRETT_HW_HAND_HW_DEVICE

#include <barrett/systems.h>

namespace rtt_barrett_hw {
  // State structure for a Hand
  struct HandHWDevice 
  {
    boost::shared_ptr<barrett::Hand> interface;
  };

}

#endif // ifnedf __RTT_BARRETT_HW_HAND_DEVICE
