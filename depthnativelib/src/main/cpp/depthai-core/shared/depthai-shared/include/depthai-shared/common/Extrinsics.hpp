#pragma once

#include <vector>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/Point3f.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/// Extrinsics structure
struct Extrinsics {
    std::vector<std::vector<float>> rotationMatrix;
    /**
     *  (x, y, z) pose of destCameraSocket w.r.t currentCameraSocket obtained through calibration
     */
    Point3f translation;
    /**
     *  (x, y, z) pose of destCameraSocket w.r.t currentCameraSocket measured through CAD design
     */
    Point3f specTranslation;
    CameraBoardSocket toCameraSocket = CameraBoardSocket::AUTO;
    DEPTHAI_SERIALIZE(Extrinsics, rotationMatrix, translation, specTranslation, toCameraSocket);
};

}  // namespace dai