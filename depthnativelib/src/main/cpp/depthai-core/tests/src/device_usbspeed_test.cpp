#include <catch2/catch_all.hpp>

// std
#include <atomic>
#include <chrono>
#include <iostream>

// Include depthai library
#include <depthai/depthai.hpp>

void makeInfo(dai::Pipeline& p) {
    auto xo = p.create<dai::node::XLinkOut>();
    xo->setStreamName("sysinfo");
    auto info = p.create<dai::node::SystemLogger>();
    info->out.link(xo->input);
}

void verifyInfo(dai::Device& d) {
    auto infoQ = d.getOutputQueue("sysinfo", 1, false);
    bool timeout = false;
    auto infoFrame = infoQ->get<dai::SystemInformation>(std::chrono::seconds(1), timeout);
    if(timeout || (infoFrame == nullptr)) throw std::runtime_error("no valid frame arrived at host");
    if((infoFrame->leonCssCpuUsage.average < 0.0f) || (infoFrame->leonCssCpuUsage.average > 1.0f))
        throw std::runtime_error("invalid leonCssCpuUsage.average " + std::to_string(infoFrame->leonCssCpuUsage.average));
}

TEST_CASE("usb2Mode == true") {
    dai::Pipeline p;
    makeInfo(p);
    dai::Device d(p, true);
    verifyInfo(d);
}

TEST_CASE("UsbSpeed::HIGH") {
    dai::Pipeline p;
    makeInfo(p);
    dai::Device d(p, dai::UsbSpeed::HIGH);
    verifyInfo(d);
}

TEST_CASE("UsbSpeed::SUPER") {
    dai::Pipeline p;
    makeInfo(p);
    dai::Device d(p, dai::UsbSpeed::SUPER);
    verifyInfo(d);
}

TEST_CASE("UsbSpeed::SUPER_PLUS") {
    dai::Pipeline p;
    makeInfo(p);
    dai::Device d(p, dai::UsbSpeed::SUPER_PLUS);

    verifyInfo(d);
}

TEST_CASE("Pipeline config usb speed == SUPER") {
    dai::Pipeline p;
    makeInfo(p);

    dai::DeviceBase::Config cfg;
    cfg.board.usb.maxSpeed = dai::UsbSpeed::SUPER;

    p.setBoardConfig(cfg.board);

    dai::Device d(p);

    CHECK(d.getUsbSpeed() == dai::UsbSpeed::SUPER);

    verifyInfo(d);
}

TEST_CASE("Pipeline config usb speed == HIGH") {
    dai::Pipeline p;
    makeInfo(p);

    dai::DeviceBase::Config cfg;
    cfg.board.usb.maxSpeed = dai::UsbSpeed::HIGH;

    p.setBoardConfig(cfg.board);

    dai::Device d(p);

    CHECK(d.getUsbSpeed() == dai::UsbSpeed::HIGH);

    verifyInfo(d);
}
