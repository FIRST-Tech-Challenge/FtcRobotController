#pragma once

// libraries
#include <algorithm>

#include "depthai-shared/common/Point2f.hpp"
#include "depthai-shared/common/Size2f.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/**
 * Rect structure
 *
 * x,y coordinates together with width and height that define a rectangle.
 * Can be either normalized [0,1] or absolute representation.
 */
struct Rect {
    // default constructor
    Rect() = default;
    Rect(float x, float y, float width, float height) : x(x), y(y), width(width), height(height) {}
    Rect(const Rect& r) : x(r.x), y(r.y), width(r.width), height(r.height) {}
    Rect(const Point2f& org, const Size2f& sz) : x(org.x), y(org.y), width(sz.width), height(sz.height) {}
    Rect(const Point2f& pt1, const Point2f& pt2)
        : x(std::min(pt1.x, pt2.x)), y(std::min(pt1.y, pt2.y)), width(std::max(pt1.x, pt2.x) - x), height(std::max(pt1.y, pt2.y) - y) {}
    Rect& operator=(const Rect& r) = default;
    Rect& operator=(Rect&& r) = default;

    /**
     * The top-left corner.
     */
    Point2f topLeft() const {
        return Point2f(x, y);
    }

    /**
     * The bottom-right corner
     */
    Point2f bottomRight() const {
        return Point2f(x + width, y + height);
    }

    /**
     * Size (width, height) of the rectangle
     */
    Size2f size() const {
        return Size2f(width, height);
    }

    /**
     * Area (width*height) of the rectangle
     */
    float area() const {
        return width * height;
    }

    /**
     * True if rectangle is empty.
     */
    bool empty() const {
        return width <= 0 || height <= 0;
    }

    /**
     * Checks whether the rectangle contains the point.
     */
    bool contains(const Point2f& pt) const {
        return x <= pt.x && pt.x < x + width && y <= pt.y && pt.y < y + height;
    }

    /**
     * Whether rectangle is normalized (coordinates in [0,1] range) or not.
     */
    bool isNormalized() const {
        if(x + width <= 1.f && y + height <= 1.f) return true;
        return !(x == static_cast<int>(x) && y == static_cast<int>(y) && width == static_cast<int>(width) && height == static_cast<int>(height));
    }

    /**
     * Denormalize rectangle.
     * @param destWidth Destination frame width.
     * @param destHeight Destination frame height.
     */
    Rect denormalize(int destWidth, int destHeight) const {
        if(isNormalized()) {
            return Rect(std::round(x * destWidth), std::round(y * destHeight), std::round(width * destWidth), std::round(height * destHeight));
        }
        return *this;
    }

    /**
     * Normalize rectangle.
     * @param srcWidth Source frame width.
     * @param srcHeight Source frame height.
     */
    Rect normalize(int srcWidth, int srcHeight) const {
        if(isNormalized()) {
            return *this;
        }
        return Rect(x / srcWidth, y / srcHeight, width / srcWidth, height / srcHeight);
    }

    // order of declaration must be x, y, width, height for constructor initializer lists
    float x = 0.0f;       // x coordinate of the top-left corner
    float y = 0.0f;       // y coordinate of the top-left corner
    float width = 0.0f;   // width of the rectangle
    float height = 0.0f;  // height of the rectangle
};
DEPTHAI_SERIALIZE_EXT(Rect, x, y, width, height);

}  // namespace dai
