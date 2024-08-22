#include "depthai/pipeline/datatype/IMUData.hpp"

namespace dai {

std::shared_ptr<RawBuffer> IMUData::serialize() const {
    return raw;
}

IMUData::IMUData() : Buffer(std::make_shared<RawIMUData>()), rawIMU(*dynamic_cast<RawIMUData*>(raw.get())), packets(rawIMU.packets) {}
IMUData::IMUData(std::shared_ptr<RawIMUData> ptr) : Buffer(std::move(ptr)), rawIMU(*dynamic_cast<RawIMUData*>(raw.get())), packets(rawIMU.packets) {}

}  // namespace dai
