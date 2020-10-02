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

import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.state.State;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.atomic.AtomicReference;

public class ResetTimersState extends State {
    private final OdsBallSensorState startState;
    private final OdsBallSensorState endState;

    public final AtomicReference<Double> latestVelocity = new AtomicReference<>(null);

    @Override
    public State doStuffAndGetNextState() {
        Log.d("TG", " Reset timers");

        long currentStartTimer = startState.getTimerValue();
        long currentEndTimer = endState.getTimerValue();

        if (currentStartTimer != Long.MIN_VALUE && currentEndTimer != Long.MIN_VALUE) {
            long elapsedTimeMs = currentEndTimer - currentStartTimer;

            if (elapsedTimeMs > 0) {
                // If we have number of milliseconds, and 1m apart, how do we get to meters/sec

                double velocityMetersPerSecond = (double) 1 /* meters */ / ((double) elapsedTimeMs / 1000.0D) /* seconds */;

                latestVelocity.set(velocityMetersPerSecond);
            } else {
                latestVelocity.set(null);
            }
        }

        startState.resetToStart();
        endState.resetToStart();


        return nextState;
    }

    @Override
    public void resetToStart() {

    }

    @Override
    public void liveConfigure(NinjaGamePad gamePad) {

    }

    public ResetTimersState(Telemetry telemetry,
                            OdsBallSensorState startState,
                            OdsBallSensorState endState) {
        super("Reset timers", telemetry);
        this.startState = startState;
        this.endState = endState;
    }
}
