#include "utility/Initialization.hpp"

// std
#include <memory>

// project
#include "build/config.hpp"
#include "build/version.hpp"
#include "utility/Environment.hpp"
#include "utility/Logging.hpp"
#include "utility/Resources.hpp"
#include "utility/XLinkGlobalProfilingLogger.hpp"

// libraries
#include "XLink/XLink.h"
#include "spdlog/cfg/env.h"
#include "spdlog/cfg/helpers.h"
#include "spdlog/details/os.h"
#include "spdlog/spdlog.h"
#include "utility/Logging.hpp"
extern "C" {
#include "XLink/XLinkLog.h"
}

#ifdef DEPTHAI_ENABLE_BACKWARD
    #include "backward.hpp"
#endif

// For easier access to dai namespaced symbols
namespace dai {

// Anonymous namespace to hide 'Preloader' symbol and variable as its not needed to be visible to other compilation units
namespace {

// Doing early static initialization hits this stage faster than some libraries initialize their global static members

// Preloader uses static global object constructor (works only for shared libraries)
// to execute some code upon final executable launch  or library import
// Preloader
// struct Preloader {
//     Preloader(){
//         initialize();
//     }
// } preloader;

}  // namespace

// Backward library stacktrace handling
#ifdef DEPTHAI_ENABLE_BACKWARD
static std::unique_ptr<backward::SignalHandling> signalHandler;
#endif

bool initialize() {
    return initialize(nullptr, false, nullptr);
}

bool initialize(void* javavm) {
    return initialize(nullptr, false, javavm);
}

bool initialize(std::string additionalInfo, bool installSignalHandler, void* javavm) {
    return initialize(additionalInfo.c_str(), installSignalHandler, javavm);
}

bool initialize(const char* additionalInfo, bool installSignalHandler, void* javavm) {
    // singleton for checking whether depthai was already initialized
    static const bool initialized = [&]() {
        // Initialize logging
        Logging::getInstance();

#ifdef DEPTHAI_ENABLE_BACKWARD
        // install backward if specified
        auto envSignalHandler = utility::getEnv("DEPTHAI_INSTALL_SIGNAL_HANDLER");
        if(installSignalHandler && envSignalHandler != "0") {
            signalHandler = std::make_unique<backward::SignalHandling>();
        }
#else
        (void)installSignalHandler;
#endif

        // Print core commit and build datetime
        if(additionalInfo != nullptr && additionalInfo[0] != '\0') {
            logger::debug("{}", additionalInfo);
        }
        logger::debug("Library information - version: {}, commit: {} from {}, build: {}, libusb enabled: {}",
                      build::VERSION,
                      build::COMMIT,
                      build::COMMIT_DATETIME,
                      build::BUILD_DATETIME,
                      build::HAVE_LIBUSB_SUPPORT);

        // Executed at library load time

        // Preload Resources (getting instance causes some internal lazy loading to start)
        Resources::getInstance();

        // Static global handler
        static XLinkGlobalHandler_t xlinkGlobalHandler = {};
        xlinkGlobalHandler.protocol = X_LINK_USB_VSC;
        xlinkGlobalHandler.options = javavm;
        auto status = XLinkInitialize(&xlinkGlobalHandler);
        const auto ERROR_MSG_USB_TIP = fmt::format("If running in a container, make sure that the following is set: \"{}\"",
                                                   "-v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw'");
        if(X_LINK_SUCCESS != status) {
            std::string errorMsg = fmt::format("Couldn't initialize XLink: {}. ", XLinkErrorToStr(status));
            if(status == X_LINK_INIT_USB_ERROR) {
                errorMsg += ERROR_MSG_USB_TIP;
            }
            logger::debug("Initialize failed - {}", errorMsg);
            throw std::runtime_error(errorMsg);
        }

        // Check that USB protocol is available, IFF libusb is enabled
#ifdef DEPTHAI_ENABLE_LIBUSB
        if(!XLinkIsProtocolInitialized(X_LINK_USB_VSC)) {
            logger::warn("USB protocol not available - {}", ERROR_MSG_USB_TIP);
        }
#endif

        // Enable Global XLink profiling
        XLinkProfStart();
        auto profilingEnvLevel = utility::getEnv("DEPTHAI_PROFILING");
        if(profilingEnvLevel == "1") {
            XLinkGlobalProfilingLogger::getInstance().enable(true);
        }

        // TODO(themarpe), move into XLink library
        auto xlinkEnvLevel = utility::getEnv("XLINK_LEVEL");
        if(xlinkEnvLevel == "debug") {
            mvLogDefaultLevelSet(MVLOG_DEBUG);
        } else if(xlinkEnvLevel == "info") {
            mvLogDefaultLevelSet(MVLOG_INFO);
        } else if(xlinkEnvLevel == "warn") {
            mvLogDefaultLevelSet(MVLOG_WARN);
        } else if(xlinkEnvLevel == "error") {
            mvLogDefaultLevelSet(MVLOG_ERROR);
        } else if(xlinkEnvLevel == "fatal") {
            mvLogDefaultLevelSet(MVLOG_FATAL);
        } else if(xlinkEnvLevel == "off") {
            mvLogDefaultLevelSet(MVLOG_LAST);
        } else {
            // Suppress XLink related errors by default
            mvLogDefaultLevelSet(MVLOG_FATAL);
        }

        logger::debug("Initialize - finished");

        return true;
    }();
    return initialized;
}

}  // namespace dai
