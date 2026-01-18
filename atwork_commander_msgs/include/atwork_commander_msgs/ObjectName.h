#pragma once

#include <atwork_commander_msgs/msg/object.hpp>

namespace atwork_commander_msgs {

inline const char* objectName(uint16_t typeID)
{
  switch (typeID) {
    case Object::EMPTY:           return "NOTHING";
    case Object::F20_20_B:         return "F20_20_B";
    case Object::F20_20_G:         return "F20_20_G";
    case Object::S40_40_B:         return "S40_40_B";
    case Object::S40_40_G:         return "S40_40_G";
    case Object::M20_100:          return "M20_100";
    case Object::M20:              return "M20";
    case Object::M30:              return "M30";
    case Object::R20:              return "R20";
    case Object::BEARING_BOX:      return "BEARING_BOX";
    case Object::BEARING:          return "BEARING";
    case Object::AXIS:             return "AXIS";
    case Object::DISTANCE_TUBE:    return "DISTANCE_TUBE";
    case Object::MOTOR:            return "MOTOR";

    /* ROS2 renamed constants */
    case Object::AXIS2:             return "AXIS2";
    case Object::BEARING2:          return "BEARING2";
    case Object::HOUSING:            return "HOUSING";
    case Object::MOTOR2:             return "MOTOR2";
    case Object::SPACER:             return "SPACER";
    case Object::SCREWDRIVER:        return "SCREWDRIVER";
    case Object::WRENCH:             return "WRENCH";
    case Object::DRILL:              return "DRILL";
    case Object::ALLENKEY:           return "ALLENKEY";

    case Object::CONTAINER_RED:      return "CONTAINER_RED";
    case Object::CONTAINER_BLUE:     return "CONTAINER_BLUE";

    case Object::F20_20_H:           return "F20_20_H";
    case Object::F20_20_V:           return "F20_20_V";
    case Object::F20_20_F:           return "F20_20_F";

    case Object::S40_40_H:           return "S40_40_H";
    case Object::S40_40_V:           return "S40_40_V";
    case Object::S40_40_F:           return "S40_40_F";

    case Object::M20_H:              return "M20_H";
    case Object::M20_V:              return "M20_V";
    case Object::M20_F:              return "M20_F";

    case Object::M20_100_H:           return "M20_100_H";
    case Object::M20_100_V:           return "M20_100_V";
    case Object::M20_100_F:           return "M20_100_F";

    case Object::M30_H:               return "M30_H";
    case Object::M30_V:               return "M30_V";
    case Object::M30_F:               return "M30_F";

    case Object::R20_H:               return "R20_H";
    case Object::R20_V:               return "R20_V";
    case Object::R20_F:               return "R20_F";

    default:
      return "UNKNOWN";
  }
}

} // namespace atwork_commander_msgs
