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

package com.hfrobots.tnt.corelib.util;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;
import java.util.Set;

import lombok.AllArgsConstructor;
import lombok.NonNull;

@AllArgsConstructor
public class RealSimplerHardwareMap implements SimplerHardwareMap {
    @Override
    public Set<String> getNamesOf(HardwareDevice device) {
        return realMap.getNamesOf(device);
    }

    @Override
    public <T> List<T> getAll(Class<? extends T> classOrInterface) {
        return realMap.getAll(classOrInterface);
    }

    private final HardwareMap realMap;

    @Override
    public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
        return realMap.get(classOrInterface, deviceName);
    }

    @Override
    public int hashCode() {
        return realMap.hashCode();
    }

    @Override
    public boolean equals(Object obj) {
        return realMap.equals(obj);
    }

    @NonNull
    @Override
    public String toString() {
        return realMap.toString();
    }
}
