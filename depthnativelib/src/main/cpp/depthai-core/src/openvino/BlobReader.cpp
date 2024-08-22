// Modified for internal use of depthai-core library
// Luxonis - December 2020

// Copyright (C) 2018-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
//

#include "BlobReader.hpp"

#include <spdlog/fmt/fmt.h>

#include <cassert>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "BlobFormat.hpp"

// TODO(themarpe)
// #include <vpu/model/data.hpp>

namespace dai {

namespace {

template <typename T>
T readFromBlob(const std::vector<std::uint8_t>& blob, uint32_t& offset) {
    if(offset + sizeof(T) > blob.size()) {
        throw std::length_error("BlobReader error: Filesize is less than blob specifies. Likely corrupted");
    }

    auto srcPtr = blob.data() + offset;
    offset += sizeof(T);

    return *reinterpret_cast<const T*>(srcPtr);
}

bool isIOShapeName(std::string name) {
    return name.find("@shape") != std::string::npos;
}

}  // namespace

void BlobReader::parse(const std::vector<std::uint8_t>& blob) {
    if(blob.empty() || blob.size() < sizeof(ElfN_Ehdr) + sizeof(mv_blob_header)) {
        throw std::logic_error("BlobReader error: Blob is empty");
    }

    pBlob = blob.data();

    blobHeader = *reinterpret_cast<const mv_blob_header*>(blob.data() + sizeof(ElfN_Ehdr));

    if(blobHeader.magic_number != BLOB_MAGIC_NUMBER) {
        throw std::logic_error("BlobReader error: File does not seem to be a supported neural network blob");
    }

    if(blob.size() < blobHeader.file_size) {
        throw std::length_error("BlobReader error: Filesize is less than blob specifies. Likely corrupted");
    }

    const auto readIO = [this, &blob](uint32_t& ioSectionOffset, uint32_t idx) {
        auto ioIdx = readFromBlob<uint32_t>(blob, ioSectionOffset);
        if(ioIdx != idx) {
            throw std::runtime_error(
                fmt::format("BlobReader failed on I/O processing, its' ioIdx parameter (which is {}) is "
                            "different from its' processing order (which is {})",
                            ioIdx,
                            idx));
        }

        auto ioBufferOffset = readFromBlob<int32_t>(blob, ioSectionOffset);

        auto nameLength = readFromBlob<uint32_t>(blob, ioSectionOffset);
        std::string ioName(nameLength, 0);
        for(auto& c : ioName) {
            c = readFromBlob<char>(blob, ioSectionOffset);
        }

        // Truncate zeros
        ioName = ioName.c_str();

        auto dataType = static_cast<TensorInfo::DataType>(readFromBlob<int32_t>(blob, ioSectionOffset));
        auto orderCode = static_cast<TensorInfo::StorageOrder>(readFromBlob<uint32_t>(blob, ioSectionOffset));

        auto numDims = readFromBlob<uint32_t>(blob, ioSectionOffset);

        // ignore
        readFromBlob<int32_t>(blob, ioSectionOffset);

        auto dimsOffset = blobHeader.const_data_section_offset + readFromBlob<uint32_t>(blob, ioSectionOffset);

        // Skip strides' location and offset
        ioSectionOffset += 2 * sizeof(uint32_t);

        std::vector<unsigned> dims;
        for(unsigned i = 0; i < numDims; ++i) {
            dims.push_back(readFromBlob<uint32_t>(blob, dimsOffset));
        }

        TensorInfo io;
        io.numDimensions = numDims;
        io.dims = dims;
        io.name = ioName;
        io.offset = ioBufferOffset;
        io.order = orderCode;
        io.dataType = dataType;

        return io;
    };

    auto inputInfoSecOffset = blobHeader.input_info_section_offset;
    for(uint32_t i = 0; i < blobHeader.inputs_count; i++) {
        const auto processedInput = readIO(inputInfoSecOffset, i);
        if(!isIOShapeName(processedInput.name)) {
            // Add to inputs
            networkInputs[processedInput.name] = processedInput;
        }
    }

    auto outputInfoSecOffset = blobHeader.output_info_section_offset;
    for(uint32_t i = 0; i < blobHeader.outputs_count; i++) {
        const auto processedOutput = readIO(outputInfoSecOffset, i);
        if(!isIOShapeName(processedOutput.name)) {
            // Add to inputs
            networkOutputs[processedOutput.name] = processedOutput;
        }
    }
}

}  // namespace dai