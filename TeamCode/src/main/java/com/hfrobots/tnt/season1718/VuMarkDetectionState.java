/**
 Copyright (c) 2018 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.season1718;

import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.Queue;

public class VuMarkDetectionState extends TimeoutSafetyState {
    private final Queue<RelicRecoveryVuMark> vuMarkQueue;
    private final VuMarkThread vuMarkThread;
    private boolean startedThread = false;

    public VuMarkDetectionState(String name, Telemetry telemetry, HardwareMap hardwareMap,
                                Queue<RelicRecoveryVuMark> vuMarkQueue,
                                long safetyTimeoutMillis) {
        super(name, telemetry, safetyTimeoutMillis);
        this.vuMarkQueue = vuMarkQueue;
        vuMarkThread = new VuMarkThread(this.vuMarkQueue, hardwareMap, safetyTimeoutMillis);
    }

    @Override
    public State doStuffAndGetNextState() {
        if (!startedThread) {
            vuMarkThread.start();
            startedThread = true;
        }

        if (!isTimedOut()) {
            if (vuMarkQueue.peek() != null) {
                return nextState;
            }

            return this;
        } else {
            return nextState;
        }
    }

    @Override
    public void liveConfigure(DebouncedGamepadButtons buttons) {

    }
}
