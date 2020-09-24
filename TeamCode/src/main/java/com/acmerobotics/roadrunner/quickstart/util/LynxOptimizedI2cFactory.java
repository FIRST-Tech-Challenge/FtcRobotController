package com.acmerobotics.roadrunner.quickstart.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxFirmwareVersionManager;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

/**
 * Factory for creating "optimized" (i.e., read-window-less) *REV* I2C devices.
 *
 * Most I2C drivers in the SDK are built upon the {@link I2cDeviceSynch} interface which includes a
 * synchronous API for interacting with the underlying hardware. In the MR years, this was simply a
 * wrapper around an asynchronous API. The implementation (see {@link I2cDeviceSynchImpl}) relies
 * heavily on a complicated caching mechanism based on read windows. Now fast forward to the
 * introduction of REV Expansion Hubs. In contrast to MR, the REV communication protocol is
 * inherently synchronous. However, the REV implementation of {@link I2cDeviceSynch} (see {@link
 * I2cDeviceSynchImplOnSimple} and {@link LynxI2cDeviceSynch}) retains the same read window caching
 * mechanisms from before. This is problematic since each read issued on a REV {@link
 * I2cDeviceSynch} will trigger a controller read (i.e., there is no cache) yet the drivers still
 * attempt to read as many registers as possible on each read to facilitate the MR-era caching.
 * Unfortunately, with SDK v3.x and ExH firmware v1.7.x, the time required for an I2C read
 * transaction is linear -- not constant -- in the number of registers read. To address this, {@link
 * LynxOptimizedI2cFactory} constructs REV {@link I2cDeviceSynch} instances that ignore the read
 * windows specified by drivers. For NO055 IMU Euler angle reads (i.e., {@link
 * BNO055IMU#getAngularOrientation()}), the time reduction is ~40%.
 *
 * Update: This issue is still present with SDK v4.0 and ExH firmware v1.8.2. The use of the new I2C
 * read command speeds up reads compared to v1.7.2, but the read window issue is still present.
 *
 * @link https://github.com/ftctechnh/ftc_app/issues/542
 */
public class LynxOptimizedI2cFactory {
    private static class BetterI2cDeviceSynchImplOnSimple extends I2cDeviceSynchImplOnSimple {
        private BetterI2cDeviceSynchImplOnSimple(I2cDeviceSynchSimple simple, boolean isSimpleOwned) {
            super(simple, isSimpleOwned);
        }

        @Override
        public void setReadWindow(ReadWindow window) {
            // intentionally do nothing
        }
    }

    private LynxOptimizedI2cFactory() {

    }

    /**
     * Constructs an "optimized" REV {@link I2cDeviceSynch} for use with an SDK I2C device
     * driver.
     * @param module corresponding lynx module
     * @param bus bus this device is attached to (these are confusingly labeled ports on the module)
     * @return "optimized" I2cDeviceSynch
     */
    public static I2cDeviceSynch createLynxI2cDeviceSynch(LynxModule module, int bus) {
        return new BetterI2cDeviceSynchImplOnSimple(
                LynxFirmwareVersionManager.createLynxI2cDeviceSynch(AppUtil.getDefContext(), module, bus
        ), true);
    }

    /**
     * Constructs an "optimized" LynxEmbeddedIMU.
     *
     * @see #createLynxI2cDeviceSynch(LynxModule, int)
     */
    public static LynxEmbeddedIMU createLynxEmbeddedImu(LynxModule module, int bus) {
        return new LynxEmbeddedIMU(createLynxI2cDeviceSynch(module, bus));
    }
}
