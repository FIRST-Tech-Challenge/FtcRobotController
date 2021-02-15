/*
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
*/

package com.hfrobots.tnt.corelib.sensors;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

import lombok.NonNull;

public class TotalCurrentSensor {
    private final List<LynxModule> expansionHubs;

    private int sampleCount;

    private double accumulatedCurrent;

    private double maxCurrent = Double.MIN_VALUE;

    public TotalCurrentSensor(final @NonNull HardwareMap hardwareMap) {
        expansionHubs = hardwareMap.getAll(LynxModule.class);
    }

    public void update() {
        double totalCurrent = 0;

        for (LynxModule hub : expansionHubs) {
            totalCurrent += hub.getCurrent(CurrentUnit.AMPS);
            sampleCount++;
        }

        accumulatedCurrent += totalCurrent;

        if (totalCurrent > maxCurrent) {
            maxCurrent = totalCurrent;
        }
    }

    public double getAverageCurrent() {
        return accumulatedCurrent / (double) sampleCount;
    }

    public double getMaxCurrent() {
        return maxCurrent;
    }
}
