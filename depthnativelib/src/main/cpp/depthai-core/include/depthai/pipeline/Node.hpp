#pragma once

#include <algorithm>
#include <memory>
#include <set>
#include <string>
#include <tuple>

// project
#include "depthai/openvino/OpenVINO.hpp"
#include "depthai/pipeline/AssetManager.hpp"
#include "depthai/utility/copyable_unique_ptr.hpp"

// depthai-shared
#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/properties/Properties.hpp"

// libraries
#include "tl/optional.hpp"

namespace dai {
// fwd declare Pipeline
class Pipeline;
class PipelineImpl;

/**
 * @brief Abstract Node
 */
class Node {
    friend class Pipeline;
    friend class PipelineImpl;

   public:
    /// Node identificator. Unique for every node on a single Pipeline
    using Id = std::int64_t;
    struct Connection;
    // fwd declare classes
    class Input;
    class Output;
    class InputMap;
    class OutputMap;

   protected:
    std::unordered_map<std::string, Output*> outputRefs;
    std::unordered_map<std::string, Input*> inputRefs;

    std::unordered_map<std::string, OutputMap*> outputMapRefs;
    std::unordered_map<std::string, InputMap*> inputMapRefs;

    // helpers for setting refs
    void setOutputRefs(std::initializer_list<Output*> l);
    void setOutputRefs(Output* outRef);
    void setInputRefs(std::initializer_list<Input*> l);
    void setInputRefs(Input* inRef);
    void setOutputMapRefs(std::initializer_list<OutputMap*> l);
    void setOutputMapRefs(OutputMap* outMapRef);
    void setInputMapRefs(std::initializer_list<InputMap*> l);
    void setInputMapRefs(InputMap* inMapRef);

   public:
    struct DatatypeHierarchy {
        DatatypeHierarchy(DatatypeEnum d, bool c) : datatype(d), descendants(c) {}
        DatatypeEnum datatype;
        bool descendants;
    };

    class Output {
        Node& parent;

       public:
        enum class Type { MSender, SSender };
        std::string group = "";
        std::string name;
        Type type;
        // Which types and do descendants count as well?
        std::vector<DatatypeHierarchy> possibleDatatypes;
        Output(Node& par, std::string n, Type t, std::vector<DatatypeHierarchy> types)
            : parent(par), name(std::move(n)), type(t), possibleDatatypes(std::move(types)) {}
        Output(Node& par, std::string group, std::string n, Type t, std::vector<DatatypeHierarchy> types)
            : parent(par), group(std::move(group)), name(std::move(n)), type(t), possibleDatatypes(std::move(types)) {}

        Node& getParent() {
            return parent;
        }
        const Node& getParent() const {
            return parent;
        }

        /// Output to string representation
        std::string toString() const;

        /**
         * Check if this output and given input are on the same pipeline.
         * @see canConnect for checking if connection is possible
         * @returns True if output and input are on the same pipeline
         */
        bool isSamePipeline(const Input& in);

        /**
         * Check if connection is possible
         * @param in Input to connect to
         * @returns True if connection is possible, false otherwise
         */
        bool canConnect(const Input& in);

        /**
         * Retrieve all connections from this output
         * @returns Vector of connections
         */
        std::vector<Connection> getConnections();

        /**
         * Link current output to input.
         *
         * Throws an error if this output cannot be linked to given input,
         * or if they are already linked
         *
         * @param in Input to link to
         */
        void link(const Input& in);

        /**
         * Unlink a previously linked connection
         *
         * Throws an error if not linked.
         *
         * @param in Input from which to unlink from
         */
        void unlink(const Input& in);
    };

    /**
     * Output map which keeps track of extra outputs assigned to a node
     * Extends std::unordered_map<std::string, dai::Node::Output>
     */
    class OutputMap : public std::unordered_map<std::string, Output> {
        Output defaultOutput;

       public:
        std::string name;
        OutputMap(std::string name, Output defaultOutput);
        OutputMap(Output defaultOutput);
        /// Create or modify an input
        Output& operator[](const std::string& key);
    };

    class Input {
        Node& parent;

       public:
        enum class Type { SReceiver, MReceiver };
        std::string group = "";
        std::string name;
        Type type;
        bool defaultBlocking{true};
        int defaultQueueSize{8};
        tl::optional<bool> blocking;
        tl::optional<int> queueSize;
        // Options - more information about the input
        tl::optional<bool> waitForMessage;
        bool defaultWaitForMessage{false};
        friend class Output;
        std::vector<DatatypeHierarchy> possibleDatatypes;

        /// Constructs Input with default blocking and queueSize options
        Input(Node& par, std::string n, Type t, std::vector<DatatypeHierarchy> types)
            : parent(par), name(std::move(n)), type(t), possibleDatatypes(std::move(types)) {}

        /// Constructs Input with specified blocking and queueSize options
        Input(Node& par, std::string n, Type t, bool blocking, int queueSize, std::vector<DatatypeHierarchy> types)
            : parent(par), name(std::move(n)), type(t), defaultBlocking(blocking), defaultQueueSize(queueSize), possibleDatatypes(std::move(types)) {}

        /// Constructs Input with specified blocking and queueSize as well as additional options
        Input(Node& par, std::string n, Type t, bool blocking, int queueSize, bool waitForMessage, std::vector<DatatypeHierarchy> types)
            : parent(par),
              name(std::move(n)),
              type(t),
              defaultBlocking(blocking),
              defaultQueueSize(queueSize),
              defaultWaitForMessage(waitForMessage),
              possibleDatatypes(std::move(types)) {}

        /// Constructs Input with specified blocking and queueSize as well as additional options
        Input(Node& par, std::string group, std::string n, Type t, bool blocking, int queueSize, bool waitForMessage, std::vector<DatatypeHierarchy> types)
            : parent(par),
              group(std::move(group)),
              name(std::move(n)),
              type(t),
              defaultBlocking(blocking),
              defaultQueueSize(queueSize),
              defaultWaitForMessage(waitForMessage),
              possibleDatatypes(std::move(types)) {}

        Node& getParent() {
            return parent;
        }
        const Node& getParent() const {
            return parent;
        }

        /// Input to string representation
        std::string toString() const;

        /**
         * Overrides default input queue behavior.
         * @param blocking True blocking, false overwriting
         */
        void setBlocking(bool blocking);

        /**
         * Get input queue behavior
         * @returns True blocking, false overwriting
         */
        bool getBlocking() const;

        /**
         * Overrides default input queue size.
         * If queue size fills up, behavior depends on `blocking` attribute
         * @param size Maximum input queue size
         */
        void setQueueSize(int size);

        /**
         * Get input queue size.
         * @returns Maximum input queue size
         */
        int getQueueSize() const;

        /**
         * Overrides default wait for message behavior.
         * Applicable for nodes with multiple inputs.
         * Specifies behavior whether to wait for this input when a Node processes certain data or not.
         * @param waitForMessage Whether to wait for message to arrive to this input or not
         */
        void setWaitForMessage(bool waitForMessage);

        /**
         * Get behavior whether to wait for this input when a Node processes certain data or not
         * @returns Whether to wait for message to arrive to this input or not
         */
        bool getWaitForMessage() const;

        /**
         * Equivalent to setWaitForMessage but with inverted logic.
         */
        void setReusePreviousMessage(bool reusePreviousMessage);

        /**
         * Equivalent to getWaitForMessage but with inverted logic.
         */
        bool getReusePreviousMessage() const;
    };

    /**
     * Input map which keeps track of inputs assigned to a node
     * Extends std::unordered_map<std::string, dai::Node::Input>
     */
    class InputMap : public std::unordered_map<std::string, Input> {
        Input defaultInput;

       public:
        std::string name;
        InputMap(Input defaultInput);
        InputMap(std::string name, Input defaultInput);
        /// Create or modify an input
        Input& operator[](const std::string& key);
    };

    /// Connection between an Input and Output
    struct Connection {
        friend struct std::hash<Connection>;
        Connection(Output out, Input in);
        Id outputId;
        std::string outputName;
        std::string outputGroup;
        Id inputId;
        std::string inputName;
        std::string inputGroup;
        bool operator==(const Connection& rhs) const;
    };

   protected:
    // when Pipeline tries to serialize and construct on remote, it will check if all connected nodes are on same pipeline
    std::weak_ptr<PipelineImpl> parent;

   public:
    /// Id of node
    const Id id;

   protected:
    AssetManager assetManager;

    virtual Properties& getProperties();
    virtual tl::optional<OpenVINO::Version> getRequiredOpenVINOVersion();
    copyable_unique_ptr<Properties> propertiesHolder;

   public:
    // Underlying properties
    Properties& properties;

    // access
    Pipeline getParentPipeline();
    const Pipeline getParentPipeline() const;

    /// Deep copy the node
    virtual std::unique_ptr<Node> clone() const = 0;

    /// Retrieves nodes name
    virtual const char* getName() const = 0;

    /// Retrieves all nodes outputs
    std::vector<Output> getOutputs();

    /// Retrieves all nodes inputs
    std::vector<Input> getInputs();

    /// Retrieves reference to node outputs
    std::vector<Output*> getOutputRefs();

    /// Retrieves reference to node outputs
    std::vector<const Output*> getOutputRefs() const;

    /// Retrieves reference to node inputs
    std::vector<Input*> getInputRefs();

    /// Retrieves reference to node inputs
    std::vector<const Input*> getInputRefs() const;

    /// Constructs Node
    Node(const std::shared_ptr<PipelineImpl>& p, Id nodeId, std::unique_ptr<Properties> props);
    virtual ~Node() = default;

    /// Get node AssetManager as a const reference
    const AssetManager& getAssetManager() const;

    /// Get node AssetManager as a reference
    AssetManager& getAssetManager();
};

// Node CRTP class
template <typename Base, typename Derived, typename Props>
class NodeCRTP : public Base {
   public:
    using Properties = Props;
    /// Underlying properties
    Properties& properties;
    const char* getName() const override {
        return Derived::NAME;
    };
    std::unique_ptr<Node> clone() const override {
        return std::make_unique<Derived>(static_cast<const Derived&>(*this));
    };

   private:
    NodeCRTP(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
        : Base(par, nodeId, std::move(props)), properties(static_cast<Properties&>(Node::properties)) {}
    NodeCRTP(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : NodeCRTP(par, nodeId, std::make_unique<Props>()) {}
    friend Derived;
    friend Base;
    friend class PipelineImpl;
};

}  // namespace dai

// Specialization of std::hash for Node::Connection
namespace std {
template <>
struct hash<dai::Node::Connection> {
    size_t operator()(const dai::Node::Connection& obj) const {
        size_t seed = 0;
        std::hash<dai::Node::Id> hId;
        std::hash<std::string> hStr;
        seed ^= hId(obj.outputId) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= hStr(obj.outputName) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= hId(obj.inputId) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= hStr(obj.outputName) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

}  // namespace std
