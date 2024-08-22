#pragma once
#include <cstdint>

namespace dai {
/**
 * Camera sensor image orientation / pixel readout.
 * This exposes direct sensor settings. 90 or 270 degrees rotation is not available.
 *
 * AUTO denotes that the decision will be made by device (e.g. on OAK-1/megaAI: ROTATE_180_DEG).
 */
enum class CameraImageOrientation : int32_t { AUTO = -1, NORMAL, HORIZONTAL_MIRROR, VERTICAL_FLIP, ROTATE_180_DEG };

}  // namespace dai
