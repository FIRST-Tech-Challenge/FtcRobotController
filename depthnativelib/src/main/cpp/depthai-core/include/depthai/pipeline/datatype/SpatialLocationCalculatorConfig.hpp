#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawSpatialLocationCalculatorConfig.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * SpatialLocationCalculatorConfig message. Carries ROI (region of interest) and threshold for depth calculation
 */
class SpatialLocationCalculatorConfig : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawSpatialLocationCalculatorConfig& cfg;

   public:
    /**
     * Construct SpatialLocationCalculatorConfig message.
     */
    SpatialLocationCalculatorConfig();
    explicit SpatialLocationCalculatorConfig(std::shared_ptr<RawSpatialLocationCalculatorConfig> ptr);
    virtual ~SpatialLocationCalculatorConfig() = default;

    /**
     * Set a vector of ROIs as configuration data.
     * @param ROIs Vector of configuration parameters for ROIs (region of interests)
     */
    void setROIs(std::vector<SpatialLocationCalculatorConfigData> ROIs);
    /**
     * Add a new ROI to configuration data.
     * @param roi Configuration parameters for ROI (region of interest)
     */
    void addROI(SpatialLocationCalculatorConfigData& ROI);

    /**
     * Retrieve configuration data for SpatialLocationCalculator
     * @returns Vector of configuration parameters for ROIs (region of interests)
     */
    std::vector<SpatialLocationCalculatorConfigData> getConfigData() const;

    /**
     * Set explicit configuration.
     * @param config Explicit configuration
     */
    SpatialLocationCalculatorConfig& set(dai::RawSpatialLocationCalculatorConfig config);

    /**
     * Retrieve configuration data for SpatialLocationCalculator.
     * @returns config for SpatialLocationCalculator
     */
    dai::RawSpatialLocationCalculatorConfig get() const;
};

}  // namespace dai
