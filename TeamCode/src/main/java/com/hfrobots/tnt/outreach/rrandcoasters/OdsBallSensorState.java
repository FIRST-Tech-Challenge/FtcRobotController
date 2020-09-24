/**
 Copyright (c) 2017 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.outreach.rrandcoasters;

import android.util.Log;

import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Date;

public class OdsBallSensorState extends TimeoutSafetyState {
    private final OpticalDistanceSensor ods;
    private long timerValue;
    private boolean timedOut;

    private State stateWhenTimedOut;

    @Override
    public State doStuffAndGetNextState() {
        telemetry.addData("ODS ", ods.getRawLightDetected());

        if (ods.getRawLightDetected() >= 2.0) {
            timerValue = System.currentTimeMillis();
            return nextState;
        }

        if (safetyTimeoutMillis != 0 && isTimedOut()) {
            if (this.stateWhenTimedOut != null) {
                Log.d("TG", getName() + " timed out at " + new Date(System.currentTimeMillis()));
                timerValue = Long.MIN_VALUE;
                timedOut = true;

                return stateWhenTimedOut;
            }
        }

        return this;
    }

    public long getTimerValue() {
        return timerValue;
    }

    public boolean timedOut() {
        return timedOut;
    }

    @Override
    public void resetToStart() {
        timedOut = false;
        timerValue = Long.MIN_VALUE;
        super.resetToStart();
    }

    @Override
    public void liveConfigure(DebouncedGamepadButtons buttons) {

    }

    public OdsBallSensorState(String stateName, Telemetry telemetry, OpticalDistanceSensor ods, long timeout) {
        super(stateName, telemetry, timeout);
        this.ods = ods;
        resetToStart();
    }

    public void setStateWhenTimedOut(State stateWhenTimedOut) {
        this.stateWhenTimedOut = stateWhenTimedOut;
    }
}
