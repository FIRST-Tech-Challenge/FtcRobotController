#pragma once

// standard
#include <map>
#include <memory>
#include <unordered_set>
#include <vector>

// project
#include "AssetManager.hpp"
#include "Node.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/openvino/OpenVINO.hpp"

// shared
#include "depthai-shared/device/BoardConfig.hpp"
#include "depthai-shared/pipeline/PipelineSchema.hpp"
#include "depthai-shared/properties/GlobalProperties.hpp"

namespace dai {

class PipelineImpl {
    friend class Pipeline;
    friend class Node;

   public:
    PipelineImpl() = default;
    PipelineImpl(const PipelineImpl&) = default;

   private:
    // static functions
    static bool isSamePipeline(const Node::Output& out, const Node::Input& in);
    static bool canConnect(const Node::Output& out, const Node::Input& in);

    // Functions
    Node::Id getNextUniqueId();
    PipelineSchema getPipelineSchema(SerializationType type = DEFAULT_SERIALIZATION_TYPE) const;
    tl::optional<OpenVINO::Version> getPipelineOpenVINOVersion() const;
    bool isOpenVINOVersionCompatible(OpenVINO::Version version) const;
    Device::Config getDeviceConfig() const;
    void setCameraTuningBlobPath(const dai::Path& path);
    void setXLinkChunkSize(int sizeBytes);
    void setSippBufferSize(int sizeBytes);
    void setSippDmaBufferSize(int sizeBytes);
    void setBoardConfig(BoardConfig board);
    BoardConfig getBoardConfig() const;

    // Access to nodes
    std::vector<std::shared_ptr<const Node>> getAllNodes() const;
    std::vector<std::shared_ptr<Node>> getAllNodes();
    std::shared_ptr<const Node> getNode(Node::Id id) const;
    std::shared_ptr<Node> getNode(Node::Id id);

    void serialize(PipelineSchema& schema, Assets& assets, std::vector<std::uint8_t>& assetStorage, SerializationType type = DEFAULT_SERIALIZATION_TYPE) const;
    nlohmann::json serializeToJson() const;
    void remove(std::shared_ptr<Node> node);

    std::vector<Node::Connection> getConnections() const;
    void link(const Node::Output& out, const Node::Input& in);
    void unlink(const Node::Output& out, const Node::Input& in);
    void setCalibrationData(CalibrationHandler calibrationDataHandler);
    CalibrationHandler getCalibrationData() const;

    // Must be incremented and unique for each node
    Node::Id latestId = 0;
    // Pipeline asset manager
    AssetManager assetManager;
    // Optionally forced version
    tl::optional<OpenVINO::Version> forceRequiredOpenVINOVersion;
    // Global pipeline properties
    GlobalProperties globalProperties;
    // Optimized for adding, searching and removing connections
    using NodeMap = std::unordered_map<Node::Id, std::shared_ptr<Node>>;
    NodeMap nodeMap;
    using NodeConnectionMap = std::unordered_map<Node::Id, std::unordered_set<Node::Connection>>;
    // Connection map, NodeId represents id of node connected TO (input)
    NodeConnectionMap nodeConnectionMap;
    // Board configuration
    BoardConfig board;

    // Template create function
    template <class N>
    std::shared_ptr<N> create(const std::shared_ptr<PipelineImpl>& itself) {
        // Check that passed type 'N' is subclass of Node
        static_assert(std::is_base_of<Node, N>::value, "Specified class is not a subclass of Node");
        // Get unique id for this new node
        auto id = getNextUniqueId();
        // Create and store the node in the map
        auto node = std::make_shared<N>(itself, id);
        nodeMap[id] = node;
        // Return shared pointer to this node
        return node;
    }
};

/**
 * @brief Represents the pipeline, set of nodes and connections between them
 */
class Pipeline {
    std::shared_ptr<PipelineImpl> pimpl;
    PipelineImpl* impl() {
        return pimpl.get();
    }
    const PipelineImpl* impl() const {
        return pimpl.get();
    }

   public:
    /**
     * Constructs a new pipeline
     */
    Pipeline();
    explicit Pipeline(const std::shared_ptr<PipelineImpl>& pimpl);

    /// Clone the pipeline (Creates a copy)
    Pipeline clone() const;

    /**
     * @returns Global properties of current pipeline
     */
    GlobalProperties getGlobalProperties() const;

    /**
     * @returns Pipeline schema
     */
    PipelineSchema getPipelineSchema(SerializationType type = DEFAULT_SERIALIZATION_TYPE) const;

    // void loadAssets(AssetManager& assetManager);
    void serialize(PipelineSchema& schema, Assets& assets, std::vector<std::uint8_t>& assetStorage) const {
        impl()->serialize(schema, assets, assetStorage);
    }

    /// Returns whole pipeline represented as JSON
    nlohmann::json serializeToJson() const {
        return impl()->serializeToJson();
    }

    /**
     * Adds a node to pipeline.
     *
     * Node is specified by template argument N
     */
    template <class N>
    std::shared_ptr<N> create() {
        return impl()->create<N>(pimpl);
    }

    /// Removes a node from pipeline
    void remove(std::shared_ptr<Node> node) {
        impl()->remove(node);
    }

    /// Get a vector of all nodes
    std::vector<std::shared_ptr<const Node>> getAllNodes() const {
        return impl()->getAllNodes();
    }
    /// Get a vector of all nodes
    std::vector<std::shared_ptr<Node>> getAllNodes() {
        return impl()->getAllNodes();
    }

    /// Get node with id if it exists, nullptr otherwise
    std::shared_ptr<const Node> getNode(Node::Id id) const {
        return impl()->getNode(id);
    }
    /// Get node with id if it exists, nullptr otherwise
    std::shared_ptr<Node> getNode(Node::Id id) {
        return impl()->getNode(id);
    }

    /// Get all connections
    std::vector<Node::Connection> getConnections() const {
        return impl()->getConnections();
    }

    using NodeConnectionMap = PipelineImpl::NodeConnectionMap;
    /// Get a reference to internal connection representation
    const NodeConnectionMap& getConnectionMap() const {
        return impl()->nodeConnectionMap;
    }

    using NodeMap = PipelineImpl::NodeMap;
    /// Get a reference to internal node map
    const NodeMap& getNodeMap() const {
        return impl()->nodeMap;
    }

    /**
     * Link output to an input. Both nodes must be on the same pipeline
     *
     * Throws an error if they aren't or cannot be connected
     *
     * @param out Nodes output to connect from
     * @param in Nodes input to connect to
     */
    void link(const Node::Output& out, const Node::Input& in) {
        impl()->link(out, in);
    }

    /**
     * Unlink output from an input.
     *
     * Throws an error if link doesn't exists
     *
     * @param out Nodes output to unlink from
     * @param in Nodes input to unlink to
     */
    void unlink(const Node::Output& out, const Node::Input& in) {
        impl()->unlink(out, in);
    }

    /// Get pipelines AssetManager as reference
    const AssetManager& getAssetManager() const {
        return impl()->assetManager;
    }

    /// Get pipelines AssetManager as reference
    AssetManager& getAssetManager() {
        return impl()->assetManager;
    }

    /// Set a specific OpenVINO version to use with this pipeline
    void setOpenVINOVersion(OpenVINO::Version version) {
        impl()->forceRequiredOpenVINOVersion = version;
    }

    /**
     * Sets the calibration in pipeline which overrides the calibration data in eeprom
     *
     * @param calibrationDataHandler CalibrationHandler object which is loaded with calibration information.
     */
    void setCalibrationData(CalibrationHandler calibrationDataHandler) {
        impl()->setCalibrationData(calibrationDataHandler);
    }

    /**
     * gets the calibration data which is set through pipeline
     *
     * @return the calibrationHandler with calib data in the pipeline
     */
    CalibrationHandler getCalibrationData() const {
        return impl()->getCalibrationData();
    }

    /// Get possible OpenVINO version to run this pipeline
    OpenVINO::Version getOpenVINOVersion() const {
        return impl()->getPipelineOpenVINOVersion().value_or(OpenVINO::DEFAULT_VERSION);
    }

    /// Get required OpenVINO version to run this pipeline. Can be none
    tl::optional<OpenVINO::Version> getRequiredOpenVINOVersion() const {
        return impl()->getPipelineOpenVINOVersion();
    }

    /// Set a camera IQ (Image Quality) tuning blob, used for all cameras
    void setCameraTuningBlobPath(const dai::Path& path) {
        impl()->setCameraTuningBlobPath(path);
    }

    /**
     * Set chunk size for splitting device-sent XLink packets, in bytes. A larger value could
     * increase performance, with 0 disabling chunking. A negative value won't modify the
     * device defaults - configured per protocol, currently 64*1024 for both USB and Ethernet.
     */
    void setXLinkChunkSize(int sizeBytes) {
        impl()->setXLinkChunkSize(sizeBytes);
    }

    /**
     * SIPP (Signal Image Processing Pipeline) internal memory pool.
     * SIPP is a framework used to schedule HW filters, e.g. ISP, Warp, Median filter etc.
     * Changing the size of this pool is meant for advanced use cases, pushing the limits of the HW.
     * By default memory is allocated in high speed CMX memory. Setting to 0 will allocate in DDR 256 kilobytes.
     * Units are bytes.
     */
    void setSippBufferSize(int sizeBytes) {
        impl()->setSippBufferSize(sizeBytes);
    }

    /**
     * SIPP (Signal Image Processing Pipeline) internal DMA memory pool.
     * SIPP is a framework used to schedule HW filters, e.g. ISP, Warp, Median filter etc.
     * Changing the size of this pool is meant for advanced use cases, pushing the limits of the HW.
     * Memory is allocated in high speed CMX memory
     * Units are bytes.
     */
    void setSippDmaBufferSize(int sizeBytes) {
        impl()->setSippDmaBufferSize(sizeBytes);
    }

    /// Checks whether a given OpenVINO version is compatible with the pipeline
    bool isOpenVINOVersionCompatible(OpenVINO::Version version) const {
        return impl()->isOpenVINOVersionCompatible(version);
    }

    /// Sets board configuration
    void setBoardConfig(BoardConfig board) {
        impl()->setBoardConfig(board);
    }

    /// Gets board configuration
    BoardConfig getBoardConfig() const {
        return impl()->getBoardConfig();
    }

    /// Get device configuration needed for this pipeline
    Device::Config getDeviceConfig() const {
        return impl()->getDeviceConfig();
    }
};

}  // namespace dai
