#include "depthai/pipeline/datatype/ImgFrame.hpp"

#include <cmath>

// #include "spdlog/spdlog.h"

namespace dai {

ImgFrame& ImgFrame::setFrame(cv::Mat frame) {
    img.data.clear();
    img.data.insert(img.data.begin(), frame.datastart, frame.dataend);
    return *this;
}

cv::Mat ImgFrame::getFrame(bool deepCopy) {
    // Convert to cv::Mat. If deepCopy enabled, then copy pixel data, otherwise reference only
    cv::Mat mat;
    cv::Size size = {0, 0};
    int type = 0;

    switch(getType()) {
        case Type::RGB888i:
        case Type::BGR888i:
        case Type::BGR888p:
        case Type::RGB888p:
            size = cv::Size(getWidth(), getHeight());
            type = CV_8UC3;
            break;

        case Type::YUV420p:
        case Type::NV12:
        case Type::NV21:
            size = cv::Size(getWidth(), getHeight() * 3 / 2);
            type = CV_8UC1;
            break;

        case Type::RAW8:
        case Type::GRAY8:
            size = cv::Size(getWidth(), getHeight());
            type = CV_8UC1;
            break;

        case Type::GRAYF16:
            size = cv::Size(getWidth(), getHeight());
            type = CV_16FC1;
            break;

        case Type::RAW16:
        case Type::RAW14:
        case Type::RAW12:
        case Type::RAW10:
            size = cv::Size(getWidth(), getHeight());
            type = CV_16UC1;
            break;

        case Type::RGBF16F16F16i:
        case Type::BGRF16F16F16i:
        case Type::RGBF16F16F16p:
        case Type::BGRF16F16F16p:
            size = cv::Size(getWidth(), getHeight());
            type = CV_16FC3;
            break;

        case dai::RawImgFrame::Type::BITSTREAM:
        default:
            size = cv::Size(static_cast<int>(getData().size()), 1);
            type = CV_8UC1;
            break;
    }

    // Check if enough data
    long requiredSize = CV_ELEM_SIZE(type) * size.area();
    long actualSize = static_cast<long>(img.data.size());
    if(actualSize < requiredSize) {
        throw std::runtime_error("ImgFrame doesn't have enough data to encode specified frame, required " + std::to_string(requiredSize) + ", actual "
                                 + std::to_string(actualSize) + ". Maybe metadataOnly transfer was made?");
    } else if(actualSize > requiredSize) {
        // FIXME doesn't build on Windows (multiple definitions during link)
        // logger::warn("ImgFrame has excess data: actual {}, expected {}", actualSize, requiredSize);
    }
    if(getWidth() <= 0 || getHeight() <= 0) {
        throw std::runtime_error("ImgFrame metadata not valid (width or height = 0)");
    }

    // Copy or reference to existing data
    if(deepCopy) {
        // Create new image data
        mat.create(size, type);
        // Copy number of bytes that are available by Mat space or by img data size
        std::memcpy(mat.data, img.data.data(), std::min((long)(img.data.size()), (long)(mat.dataend - mat.datastart)));
    } else {
        mat = cv::Mat(size, type, img.data.data());
    }

    return mat;
}

cv::Mat ImgFrame::getCvFrame() {
    cv::Mat frame = getFrame();
    cv::Mat output;

    switch(getType()) {
        case Type::RGB888i:
            cv::cvtColor(frame, output, cv::ColorConversionCodes::COLOR_RGB2BGR);
            break;

        case Type::BGR888i:
            output = frame.clone();
            break;

        case Type::RGB888p: {
            cv::Size s(getWidth(), getHeight());
            std::vector<cv::Mat> channels;
            // RGB
            channels.push_back(cv::Mat(s, CV_8UC1, getData().data() + s.area() * 2));
            channels.push_back(cv::Mat(s, CV_8UC1, getData().data() + s.area() * 1));
            channels.push_back(cv::Mat(s, CV_8UC1, getData().data() + s.area() * 0));
            cv::merge(channels, output);
        } break;

        case Type::BGR888p: {
            cv::Size s(getWidth(), getHeight());
            std::vector<cv::Mat> channels;
            // BGR
            channels.push_back(cv::Mat(s, CV_8UC1, getData().data() + s.area() * 0));
            channels.push_back(cv::Mat(s, CV_8UC1, getData().data() + s.area() * 1));
            channels.push_back(cv::Mat(s, CV_8UC1, getData().data() + s.area() * 2));
            cv::merge(channels, output);
        } break;

        case Type::YUV420p:
            cv::cvtColor(frame, output, cv::ColorConversionCodes::COLOR_YUV2BGR_IYUV);
            break;

        case Type::NV12:
            cv::cvtColor(frame, output, cv::ColorConversionCodes::COLOR_YUV2BGR_NV12);
            break;

        case Type::NV21:
            cv::cvtColor(frame, output, cv::ColorConversionCodes::COLOR_YUV2BGR_NV21);
            break;

        case Type::RAW8:
        case Type::RAW16:
        case Type::RAW14:
        case Type::RAW12:
        case Type::RAW10:
        case Type::GRAY8:
        case Type::GRAYF16:
            output = frame.clone();
            break;

        default:
            output = frame.clone();
            break;
    }

    return output;
}

}  // namespace dai
