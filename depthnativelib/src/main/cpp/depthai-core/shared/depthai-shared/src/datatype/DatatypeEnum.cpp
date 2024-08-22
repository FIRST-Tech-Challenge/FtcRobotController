#include "depthai-shared/datatype/DatatypeEnum.hpp"

#include <functional>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace dai {

const std::unordered_map<DatatypeEnum, std::vector<DatatypeEnum>> hierarchy = {
    {DatatypeEnum::Buffer,
     {
         DatatypeEnum::ImgFrame,
         DatatypeEnum::NNData,
         DatatypeEnum::ImageManipConfig,
         DatatypeEnum::CameraControl,
         DatatypeEnum::ImgDetections,
         DatatypeEnum::SpatialImgDetections,
         DatatypeEnum::SystemInformation,
         DatatypeEnum::SpatialLocationCalculatorConfig,
         DatatypeEnum::SpatialLocationCalculatorData,
         DatatypeEnum::EdgeDetectorConfig,
         DatatypeEnum::Tracklets,
         DatatypeEnum::IMUData,
         DatatypeEnum::StereoDepthConfig,
         DatatypeEnum::FeatureTrackerConfig,
         DatatypeEnum::ToFConfig,
         DatatypeEnum::TrackedFeatures,
         DatatypeEnum::AprilTagConfig,
         DatatypeEnum::AprilTags,
     }},
    {DatatypeEnum::ImgFrame, {}},
    {DatatypeEnum::NNData, {}},
    {DatatypeEnum::ImageManipConfig, {}},
    {DatatypeEnum::CameraControl, {}},
    {DatatypeEnum::ImgDetections, {DatatypeEnum::SpatialImgDetections}},
    {DatatypeEnum::SpatialImgDetections, {}},
    {DatatypeEnum::SystemInformation, {}},
    {DatatypeEnum::SpatialLocationCalculatorConfig, {}},
    {DatatypeEnum::SpatialLocationCalculatorData, {}},
    {DatatypeEnum::EdgeDetectorConfig, {}},
    {DatatypeEnum::Tracklets, {}},
    {DatatypeEnum::IMUData, {}},
    {DatatypeEnum::StereoDepthConfig, {}},
    {DatatypeEnum::FeatureTrackerConfig, {}},
    {DatatypeEnum::ToFConfig, {}},
    {DatatypeEnum::TrackedFeatures, {}},
    {DatatypeEnum::AprilTagConfig, {}},
    {DatatypeEnum::AprilTags, {}},
};

bool isDatatypeSubclassOf(DatatypeEnum parent, DatatypeEnum children) {
    for(const auto& d : hierarchy.at(parent)) {
        if(d == children) return true;
        if(isDatatypeSubclassOf(d, children)) return true;
    }
    return false;
}

}  // namespace dai
