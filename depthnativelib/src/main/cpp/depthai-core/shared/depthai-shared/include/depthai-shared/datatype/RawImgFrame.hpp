#pragma once

#include <unordered_map>

#include "depthai-shared/common/FrameEvent.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/// RawImgFrame structure
struct RawImgFrame : public RawBuffer {
    enum class Type {
        YUV422i,    // interleaved 8 bit
        YUV444p,    // planar 4:4:4 format
        YUV420p,    // planar 4:2:0 format
        YUV422p,    // planar 8 bit
        YUV400p,    // 8-bit greyscale
        RGBA8888,   // RGBA interleaved stored in 32 bit word
        RGB161616,  // Planar 16 bit RGB data
        RGB888p,    // Planar 8 bit RGB data
        BGR888p,    // Planar 8 bit BGR data
        RGB888i,    // Interleaved 8 bit RGB data
        BGR888i,    // Interleaved 8 bit BGR data
        LUT2,       // 1 bit  per pixel, Lookup table (used for graphics layers)
        LUT4,       // 2 bits per pixel, Lookup table (used for graphics layers)
        LUT16,      // 4 bits per pixel, Lookup table (used for graphics layers)
        RAW16,      // save any raw type (8, 10, 12bit) on 16 bits
        RAW14,      // 14bit value in 16bit storage
        RAW12,      // 12bit value in 16bit storage
        RAW10,      // 10bit value in 16bit storage
        RAW8,
        PACK10,  // SIPP 10bit packed format
        PACK12,  // SIPP 12bit packed format
        YUV444i,
        NV12,
        NV21,
        BITSTREAM,  // used for video encoder bitstream
        HDR,
        RGBF16F16F16p,  // Planar FP16 RGB data
        BGRF16F16F16p,  // Planar FP16 BGR data
        RGBF16F16F16i,  // Interleaved FP16 RGB data
        BGRF16F16F16i,  // Interleaved FP16 BGR data
        GRAY8,          // 8 bit grayscale (1 plane)
        GRAYF16,        // FP16 grayscale (normalized)
        NONE
    };

    static constexpr int typeToBpp(Type type) {
        switch(type) {
            case Type::YUV422i:
                return 1;
                break;
            case Type::YUV444p:
                return 1;
                break;
            case Type::YUV420p:
                return 1;
                break;
            case Type::YUV422p:
                return 1;
                break;
            case Type::YUV400p:
                return 1;
                break;
            case Type::RGBA8888:
                return 1;
                break;
            case Type::RGB161616:
                return 2;
                break;
            case Type::RGB888p:
                return 1;
                break;
            case Type::BGR888p:
                return 1;
                break;
            case Type::RGB888i:
                return 1;
                break;
            case Type::BGR888i:
                return 1;
                break;
            case Type::RGBF16F16F16p:
                return 2;
                break;
            case Type::BGRF16F16F16p:
                return 2;
                break;
            case Type::RGBF16F16F16i:
                return 2;
                break;
            case Type::BGRF16F16F16i:
                return 2;
                break;
            case Type::GRAY8:
                return 1;
                break;
            case Type::GRAYF16:
                return 2;
                break;
            case Type::LUT2:
                return 1;
                break;
            case Type::LUT4:
                return 1;
                break;
            case Type::LUT16:
                return 1;
                break;
            case Type::RAW16:
                return 2;
                break;
            case Type::RAW14:
                return 2;
                break;
            case Type::RAW12:
                return 2;
                break;
            case Type::RAW10:
                return 2;
                break;
            case Type::RAW8:
                return 1;
                break;
            case Type::PACK10:
                return 2;
                break;
            case Type::PACK12:
                return 2;
                break;
            case Type::YUV444i:
                return 1;
                break;
            case Type::NV12:
                return 1;
                break;
            case Type::NV21:
                return 1;
                break;
            case Type::BITSTREAM:
                return 1;
                break;
            case Type::HDR:
                return 1;
                break;
            case Type::NONE:
                return 0;
                break;
        }
        return 0;
    }

    struct Specs {
        Type type = Type::NONE;
        unsigned int width;     // width in pixels
        unsigned int height;    // height in pixels
        unsigned int stride;    // defined as distance in bytes from pix(y,x) to pix(y+1,x)
        unsigned int bytesPP;   // bytes per pixel (for LUT types 1)
        unsigned int p1Offset;  // Offset to first plane
        unsigned int p2Offset;  // Offset to second plane
        unsigned int p3Offset;  // Offset to third plane

        DEPTHAI_SERIALIZE(Specs, type, width, height, stride, bytesPP, p1Offset, p2Offset, p3Offset);
    };
    struct CameraSettings {
        int32_t exposureTimeUs;
        int32_t sensitivityIso;
        int32_t lensPosition;
        int32_t wbColorTemp;

        DEPTHAI_SERIALIZE(CameraSettings, exposureTimeUs, sensitivityIso, lensPosition, wbColorTemp);
    };

    Specs fb = {};
    CameraSettings cam;
    uint32_t category = 0;     //
    uint32_t instanceNum = 0;  // Which source created this frame (color, mono, ...)
    dai::FrameEvent event = dai::FrameEvent::NONE;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::ImgFrame;
    };

    DEPTHAI_SERIALIZE(RawImgFrame, fb, cam, category, instanceNum, RawBuffer::sequenceNum, RawBuffer::ts, RawBuffer::tsDevice);
};

}  // namespace dai
