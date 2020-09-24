/**
 Copyright (c) 2019 HF Robotics (http://www.hfrobots.com)
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 **/

package com.hfrobots.tnt.fakes;

import com.google.common.collect.Maps;
import com.hfrobots.tnt.corelib.util.SimplerHardwareMap;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import javax.annotation.Nullable;

public class FakeHardwareMap implements SimplerHardwareMap {
    Map<String, HardwareDevice> deviceMap = Maps.newHashMap();
    Map<HardwareDevice, String> deviceNames = Maps.newHashMap();

    public void addDevice(String deviceName, HardwareDevice device) {
        deviceMap.put(deviceName, device);
        deviceNames.put(device, deviceName);
    }

    @Override
    public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
        T result = tryGet(classOrInterface, deviceName);

        if (result==null) {
            throw new IllegalArgumentException(String.format("Unable to find a hardware device with name \"%s\" and type %s", deviceName, classOrInterface.getSimpleName()));
        }

        return result;
    }

    public @Nullable <T> T tryGet(Class<? extends T> classOrInterface, String deviceName) {
        deviceName = deviceName.trim();

        HardwareDevice device = deviceMap.get(deviceName);

        if (device != null) {
            if (classOrInterface.isInstance(device)) {
                return classOrInterface.cast(device);
            }
        }

        return null;
    }

    public <T> List<T> getAll(Class<? extends T> classOrInterface) {
        List<T> result = new LinkedList<T>();

        for (HardwareDevice device : deviceMap.values()) {
            if (classOrInterface.isInstance(device)) {
                result.add(classOrInterface.cast(device));
            }
        }

        return result;
    }

    public Set<String> getNamesOf(HardwareDevice device) {
        String result = this.deviceNames.get(device);

        if (result == null) {
            return new HashSet<String>();
        }

        Set<String> toReturn = new HashSet<>();

        toReturn.add(result);

        return toReturn;
    }
}
