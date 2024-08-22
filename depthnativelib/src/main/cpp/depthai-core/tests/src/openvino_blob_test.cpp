#include <catch2/catch_all.hpp>

// std
#include <atomic>
#include <fstream>
#include <iostream>

// Include depthai library
#include <depthai/depthai.hpp>

// setBlob

void checkBlob(dai::OpenVINO::Blob& blob) {
    printf("Blob: ");
    for(const auto& in : blob.networkInputs) {
        std::string name = in.first;
        auto tensor = in.second;
        printf("'%s - dims: %d - order: %04x - type: %d' ", name.c_str(), tensor.numDimensions, tensor.order, tensor.dataType);
    }
    for(const auto& out : blob.networkOutputs) {
        std::string name = out.first;
        auto tensor = out.second;
        printf("'%s - dims: %d - order: %04x - type: %d' ", name.c_str(), tensor.numDimensions, tensor.order, tensor.dataType);
    }
    printf("(%u %u %u %u)\n", blob.stageCount, blob.numShaves, blob.numSlices, blob.version);

    REQUIRE(blob.networkInputs.size() == 1);
    REQUIRE(blob.networkInputs.at("0").numDimensions == 4);
    REQUIRE(blob.networkInputs.at("0").order == dai::TensorInfo::StorageOrder::NCHW);
    // REQUIRE(blob.networkInputs.at("0").dataType == dai::TensorInfo::DataType::U8F);

    REQUIRE(blob.networkOutputs.size() == 1);
    REQUIRE(blob.networkOutputs.at("14").numDimensions == 4);
    REQUIRE(blob.networkOutputs.at("14").order == dai::TensorInfo::StorageOrder::NCHW);
    REQUIRE(blob.networkOutputs.at("14").dataType == dai::TensorInfo::DataType::FP16);
}

TEST_CASE("OpenVINO 2020.3 setBlob") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();

    dai::OpenVINO::Blob blob(OPENVINO_2020_3_BLOB_PATH);
    REQUIRE(blob.version == dai::OpenVINO::VERSION_2020_3);
    checkBlob(blob);
    nn->setBlob(std::move(blob));

    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2020_3);
    REQUIRE_THROWS_AS(dai::Device(p), std::runtime_error);
}

TEST_CASE("OpenVINO 2020.4 setBlob") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();

    dai::OpenVINO::Blob blob(OPENVINO_2020_4_BLOB_PATH);
    REQUIRE(blob.version == dai::OpenVINO::VERSION_2020_4);
    checkBlob(blob);
    nn->setBlob(std::move(blob));

    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2020_4);
    dai::Device d(p);
}

TEST_CASE("OpenVINO 2021.1 setBlob") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();

    dai::OpenVINO::Blob blob(OPENVINO_2021_1_BLOB_PATH);
    REQUIRE(blob.version == dai::OpenVINO::VERSION_2021_1);
    checkBlob(blob);
    nn->setBlob(std::move(blob));

    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2021_1);
    dai::Device d(p);
}

TEST_CASE("OpenVINO 2021.2 setBlob") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();

    dai::OpenVINO::Blob blob(OPENVINO_2021_2_BLOB_PATH);
    REQUIRE(blob.version == dai::OpenVINO::VERSION_2021_2);
    checkBlob(blob);
    nn->setBlob(std::move(blob));

    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2021_2);
    dai::Device d(p);
}

TEST_CASE("OpenVINO 2021.3 setBlob") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();

    dai::OpenVINO::Blob blob(OPENVINO_2021_3_BLOB_PATH);
    REQUIRE(blob.version == dai::OpenVINO::VERSION_2021_3);
    checkBlob(blob);
    nn->setBlob(std::move(blob));

    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2021_3);
    dai::Device d(p);
}

TEST_CASE("OpenVINO 2021.4 setBlob") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();

    dai::OpenVINO::Blob blob(OPENVINO_2021_4_BLOB_PATH);
    REQUIRE(blob.version == dai::OpenVINO::VERSION_2021_4);
    checkBlob(blob);
    nn->setBlob(std::move(blob));

    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2021_4);
    dai::Device d(p);
}

TEST_CASE("OpenVINO 2022.1 setBlob") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();

    dai::OpenVINO::Blob blob(OPENVINO_2022_1_BLOB_PATH);
    REQUIRE(blob.version == dai::OpenVINO::VERSION_2022_1);
    checkBlob(blob);
    nn->setBlob(std::move(blob));

    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2022_1);
    dai::Device d(p);
}

// setBlobPath

TEST_CASE("OpenVINO 2020.3 blob") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();
    nn->setBlobPath(OPENVINO_2020_3_BLOB_PATH);
    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2020_3);
    REQUIRE_THROWS_AS(dai::Device(p), std::runtime_error);
}

TEST_CASE("OpenVINO 2020.4 blob") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();
    nn->setBlobPath(OPENVINO_2020_4_BLOB_PATH);
    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2020_4);
    dai::Device d(p);
}

TEST_CASE("OpenVINO 2021.1 blob") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();
    nn->setBlobPath(OPENVINO_2021_1_BLOB_PATH);
    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2021_1);
    dai::Device d(p);
}

TEST_CASE("OpenVINO 2021.2 blob") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();
    nn->setBlobPath(OPENVINO_2021_2_BLOB_PATH);
    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2021_2);
    dai::Device d(p);
}

TEST_CASE("OpenVINO 2021.3 blob") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();
    nn->setBlobPath(OPENVINO_2021_3_BLOB_PATH);
    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2021_3);
    dai::Device d(p);
}

TEST_CASE("OpenVINO 2021.4 blob") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();
    nn->setBlobPath(OPENVINO_2021_4_BLOB_PATH);
    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2021_4);
    dai::Device d(p);
}

TEST_CASE("OpenVINO 2022.1 blob") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();
    nn->setBlobPath(OPENVINO_2022_1_BLOB_PATH);
    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2022_1);
    dai::Device d(p);
}

// Check if an exception is thrown if blob is corrupted
TEST_CASE("OpenVINO corrupted blob") {
    std::ifstream stream(OPENVINO_2021_4_BLOB_PATH, std::ios::in | std::ios::binary);
    std::vector<std::uint8_t> blobData(std::istreambuf_iterator<char>(stream), {});

    // Corrupt blob by removing half the file size
    blobData.resize(blobData.size() / 2);

    REQUIRE_THROWS(dai::OpenVINO::Blob(blobData));
}

// TEST UNIVERSAL FW

TEST_CASE("OpenVINO 2020.4 blob, test with universal FW") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();
    nn->setBlobPath(OPENVINO_2020_4_BLOB_PATH);
    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2020_4);
    p.setOpenVINOVersion(dai::OpenVINO::VERSION_UNIVERSAL);
    dai::Device d(p);
}

TEST_CASE("OpenVINO 2021.1 blob, test with universal FW") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();
    nn->setBlobPath(OPENVINO_2021_1_BLOB_PATH);
    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2021_1);
    p.setOpenVINOVersion(dai::OpenVINO::VERSION_UNIVERSAL);
    dai::Device d(p);
}

TEST_CASE("OpenVINO 2021.2 blob, test with universal FW") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();
    nn->setBlobPath(OPENVINO_2021_2_BLOB_PATH);
    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2021_2);
    p.setOpenVINOVersion(dai::OpenVINO::VERSION_UNIVERSAL);
    dai::Device d(p);
}

TEST_CASE("OpenVINO 2021.3 blob, test with universal FW") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();
    nn->setBlobPath(OPENVINO_2021_3_BLOB_PATH);
    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2021_3);
    p.setOpenVINOVersion(dai::OpenVINO::VERSION_UNIVERSAL);
    dai::Device d(p);
}

TEST_CASE("OpenVINO 2021.4 blob, test with universal FW") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();
    nn->setBlobPath(OPENVINO_2021_4_BLOB_PATH);
    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2021_4);
    p.setOpenVINOVersion(dai::OpenVINO::VERSION_UNIVERSAL);
    dai::Device d(p);
}

TEST_CASE("OpenVINO 2022.1 blob, test with universal FW") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();
    nn->setBlobPath(OPENVINO_2022_1_BLOB_PATH);
    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2022_1);
    p.setOpenVINOVersion(dai::OpenVINO::VERSION_UNIVERSAL);
    dai::Device d(p);
}
