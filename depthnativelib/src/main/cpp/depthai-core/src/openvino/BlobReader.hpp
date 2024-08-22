// Modified for internal use of depthai-core library
// Luxonis - December 2020

// Copyright (C) 2018-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

#pragma once

#include <vector>
#include <utility>

#include "BlobFormat.hpp"
#include "depthai-shared/common/TensorInfo.hpp"

namespace dai {

class BlobReader {
public:
    BlobReader() = default;

    void parse(const std::vector<std::uint8_t>& blob);

    const std::unordered_map<std::string, TensorInfo>& getNetworkInputs() const { return networkInputs; }
    const std::unordered_map<std::string, TensorInfo>& getNetworkOutputs() const { return networkOutputs; }

    uint32_t getStageCount() const { return blobHeader.stages_count; }

    uint32_t getMagicNumber() const { return blobHeader.magic_number; }

    uint32_t getVersionMajor() const { return blobHeader.blob_ver_major; }
    uint32_t getVersionMinor() const { return blobHeader.blob_ver_minor; }

    uint32_t getNumberOfShaves() const { return blobHeader.number_of_shaves; }
    uint32_t getNumberOfSlices() const { return blobHeader.number_of_cmx_slices; }

    std::pair<const std::uint8_t*, size_t> getHeader() const { return {pBlob, sizeof(ElfN_Ehdr) + sizeof(mv_blob_header)};}

private:
    const std::uint8_t* pBlob = nullptr;

    mv_blob_header blobHeader = {};

    constexpr static std::uint32_t BLOB_MAGIC_NUMBER = 9709;

    std::unordered_map<std::string, TensorInfo> networkInputs;
    std::unordered_map<std::string, TensorInfo> networkOutputs;

};

}  // namespace dai
