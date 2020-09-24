/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.hfrobots.tnt.corelib.sensors;

import android.util.Log;

import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cControllerPortDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class I2cDeviceEnableDisable {
    private final I2cControllerPortDevice device;
    private final I2cDeviceSynch syncDevice;
    private final I2cController i2cController;
    private final int port;
    private final I2cController.I2cPortReadyCallback deviceCallback;
    private boolean deviceEnabled = true;

    public I2cDeviceEnableDisable(final I2cControllerPortDevice device) {
        this.device = device;
        i2cController = device.getI2cController();
        port = device.getPort();
        deviceCallback = i2cController.getI2cPortReadyCallback(port);
        deviceEnabled = true;
        syncDevice = null;
    }

    public I2cDeviceEnableDisable(final I2cDeviceSynch syncDevice) {
        this.syncDevice = syncDevice;
        i2cController = null;
        port = 0;
        deviceCallback = null;
        this.device = null;
    }

    public boolean isEnabled() {
        return deviceEnabled;
    }

    public void setEnabled(boolean enabled) {
        if (deviceEnabled != enabled) {
            if (enabled) {
                if (syncDevice != null) {
                    Log.i(LOG_TAG, String.format("Enabling I2cDeviceSync device %s", syncDevice));
                    syncDevice.engage();
                } else {
                    Log.i(LOG_TAG, String.format("Enabling I2cControllerPortDevice device %s", device));
                    i2cController.registerForI2cPortReadyCallback(deviceCallback, port);
                }
            } else {
                if (syncDevice != null) {
                    Log.i(LOG_TAG, String.format("Disabling I2cDeviceSync device %s", syncDevice));
                    syncDevice.disengage();
                } else {
                    Log.i(LOG_TAG, String.format("Disabling I2cControllerPortDevice device %s", device));
                    i2cController.deregisterForPortReadyCallback(port);
                }
            }
            deviceEnabled = enabled;
        }
    }
}
