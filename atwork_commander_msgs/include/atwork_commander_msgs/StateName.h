#pragma once

#include <atwork_commander_msgs/msg/refbox_state.hpp>

namespace atwork_commander_msgs {

inline const char* stateName(uint32_t state)
{
  switch (state) {
    case atwork_commander_msgs::msg::RefboxState::FAILURE:
      return "FAILURE";

    case atwork_commander_msgs::msg::RefboxState::IDLE:
      return "IDLE";

    case atwork_commander_msgs::msg::RefboxState::READY:
      return "READY";

    case atwork_commander_msgs::msg::RefboxState::PREPARATION:
      return "PREPARATION";

    case atwork_commander_msgs::msg::RefboxState::EXECUTION:
      return "EXECUTION";

    default:
      return "UNKNOWN";
  }
}

} // namespace atwork_commander_msgs
