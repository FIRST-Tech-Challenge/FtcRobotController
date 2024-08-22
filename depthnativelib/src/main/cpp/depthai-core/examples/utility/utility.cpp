#include "utility.hpp"

// libraries
#include "fp16/fp16.h"

#include "errno.h"

#if (defined(_WIN32) || defined(_WIN64))
#include <Windows.h>
#include <direct.h>
#else
#include <sys/stat.h>
#endif

int createDirectory(std::string directory)
{
    int ret = 0;
#if (defined(_WIN32) || defined(_WIN64))
    ret =  _mkdir(directory.c_str());
#else
    ret = mkdir(directory.c_str(), 0777);
#endif
    if (ret == EEXIST) ret = 0;
    return ret;
}

cv::Mat fromPlanarFp16(const std::vector<float>& data, int w, int h, float mean, float scale){
    cv::Mat frame = cv::Mat(h, w, CV_8UC3);

    for(int i = 0; i < w*h; i++) {
        auto b = data.data()[i + w*h * 0] * scale + mean;
        frame.data[i*3+0] = (uint8_t)b;
    }
    for(int i = 0; i < w*h; i++) {
        auto g = data.data()[i + w*h * 1] * scale + mean;
        frame.data[i*3+1] = (uint8_t)g;
    }
    for(int i = 0; i < w*h; i++) {
        auto r = data.data()[i + w*h * 2] * scale + mean;
        frame.data[i*3+2] = (uint8_t)r;
    }
    return frame;
}

cv::Mat toMat(const std::vector<uint8_t>& data, int w, int h , int numPlanes, int bpp){

    cv::Mat frame;

    if(numPlanes == 3){
        frame = cv::Mat(h, w, CV_8UC3);

        // optimization (cache)
        for(int i = 0; i < w*h; i++) {
            uint8_t b = data.data()[i + w*h * 0];
            frame.data[i*3+0] = b;
        }
        for(int i = 0; i < w*h; i++) {
            uint8_t g = data.data()[i + w*h * 1];
            frame.data[i*3+1] = g;
        }
        for(int i = 0; i < w*h; i++) {
            uint8_t r = data.data()[i + w*h * 2];
            frame.data[i*3+2] = r;
        }

    } else {
        if(bpp == 3){
            frame = cv::Mat(h, w, CV_8UC3);
            for(int i = 0; i < w*h*bpp; i+=3) {
                uint8_t b,g,r;
                b = data.data()[i + 2];
                g = data.data()[i + 1];
                r = data.data()[i + 0];
                frame.at<cv::Vec3b>( (i/bpp) / w, (i/bpp) % w) = cv::Vec3b(b,g,r);
            }

        } else if(bpp == 6) {
            //first denormalize
            //dump

            frame = cv::Mat(h, w, CV_8UC3);
            for(int y = 0; y < h; y++){
                for(int x = 0; x < w; x++){

                    const uint16_t* fp16 = (const uint16_t*) (data.data() + (y*w+x)*bpp);
                    uint8_t r = (uint8_t) (fp16_ieee_to_fp32_value(fp16[0]) * 255.0f);
                    uint8_t g = (uint8_t) (fp16_ieee_to_fp32_value(fp16[1]) * 255.0f);
                    uint8_t b = (uint8_t) (fp16_ieee_to_fp32_value(fp16[2]) * 255.0f);
                    frame.at<cv::Vec3b>(y, x) = cv::Vec3b(b,g,r);
                }
            }

        }
    }

    return frame;
}


void toPlanar(cv::Mat& bgr, std::vector<std::uint8_t>& data){

    data.resize(bgr.cols * bgr.rows * 3);
    for(int y = 0; y < bgr.rows; y++){
        for(int x = 0; x < bgr.cols; x++){
            auto p = bgr.at<cv::Vec3b>(y,x);
            data[x + y*bgr.cols + 0 * bgr.rows*bgr.cols] = p[0];
            data[x + y*bgr.cols + 1 * bgr.rows*bgr.cols] = p[1];
            data[x + y*bgr.cols + 2 * bgr.rows*bgr.cols] = p[2];
        }
    }
}

cv::Mat resizeKeepAspectRatio(const cv::Mat &input, const cv::Size &dstSize, const cv::Scalar &bgcolor)
{
    cv::Mat output;

    double h1 = dstSize.width * (input.rows/(double)input.cols);
    double w2 = dstSize.height * (input.cols/(double)input.rows);
    if( h1 <= dstSize.height) {
        cv::resize( input, output, cv::Size(dstSize.width, h1));
    } else {
        cv::resize( input, output, cv::Size(w2, dstSize.height));
    }

    int top = (dstSize.height-output.rows) / 2;
    int down = (dstSize.height-output.rows+1) / 2;
    int left = (dstSize.width - output.cols) / 2;
    int right = (dstSize.width - output.cols+1) / 2;

    cv::copyMakeBorder(output, output, top, down, left, right, cv::BORDER_CONSTANT, bgcolor );

    return output;
}

