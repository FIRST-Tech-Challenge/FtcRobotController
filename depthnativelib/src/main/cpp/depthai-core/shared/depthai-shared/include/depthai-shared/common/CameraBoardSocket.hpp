#pragma once
#include <cstdint>
namespace dai {
/**
 * Which Camera socket to use.
 *
 * AUTO denotes that the decision will be made by device
 */
enum class CameraBoardSocket : int32_t {
    AUTO = -1,
    CAM_A,
    CAM_B,
    CAM_C,
    CAM_D,
    CAM_E,
    CAM_F,
    CAM_G,
    CAM_H,
    CAM_I,
    CAM_J,
    // Deprecated naming
    RGB [[deprecated]] = CAM_A,
    CENTER [[deprecated]] = CAM_A,
    LEFT [[deprecated]] = CAM_B,
    RIGHT [[deprecated]] = CAM_C,
};

}  // namespace dai
