/*
 Copyright (c) 2021 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.season2021;

import android.util.Log;

import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.state.State;
import com.ftc9929.corelib.state.StopwatchDelayState;
import com.google.common.base.Stopwatch;
import com.google.common.base.Ticker;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

import lombok.NonNull;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class RingHoldDown {
    public final static double UP_POSITION = 1.0;

    public final static double THREE_RINGS_POSITION = 0.7;

    public final static double TWO_RINGS_POSITION = 0.66;

    public final static double ONE_RINGS_POSITION = 0.646;

    private final Servo ringHoldDownServo;

    private State currentState;

    private final RingHolderPosition upPosition;

    private final RingHolderPosition threeRingsPosition;
    private final StopwatchDelayState delayAfterLaunch;
    private final PositionRingHolderServoState positionRingHolderServoState;

    @RequiredArgsConstructor
    static class RingHolderPosition {
        final int numberOfRings;

        final double servoPosition;

        @Setter
        RingHolderPosition nextPosition;
    }

    private RingHolderPosition ringHolderPosition;

    private boolean disabled = false;

    public RingHoldDown(HardwareMap hardwareMap, Telemetry telemetry, Ticker ticker) {
        ringHoldDownServo = hardwareMap.get(Servo.class, "ringHoldDownServo");

        positionRingHolderServoState = new PositionRingHolderServoState(telemetry);

        delayAfterLaunch = new StopwatchDelayState("Launch delay",
                telemetry, ticker, 100, TimeUnit.MILLISECONDS);

        delayAfterLaunch.setNextState(positionRingHolderServoState);

        currentState = positionRingHolderServoState;

        // Setup the relationship between how many rings, what position, and what the
        // *next* position for the number of rings would be...

        upPosition = new RingHolderPosition(0, UP_POSITION);

        threeRingsPosition = new RingHolderPosition(3, THREE_RINGS_POSITION);
        upPosition.setNextPosition(threeRingsPosition);

        RingHolderPosition twoRingsPosition = new RingHolderPosition(2, TWO_RINGS_POSITION);
        threeRingsPosition.setNextPosition(twoRingsPosition);

        RingHolderPosition oneRingPosition = new RingHolderPosition(1, ONE_RINGS_POSITION);
        twoRingsPosition.setNextPosition(oneRingPosition);
        oneRingPosition.setNextPosition(upPosition);

        ringHolderPosition = upPosition;
    }

    public void emergencyDisable() {
        disabled = true;
    }

    public void holdThreePosition() {
        ringHolderPosition = threeRingsPosition;

        currentState = positionRingHolderServoState;
        periodicTask(); // force the servo to move
    }

    public void resetToUpPosition() {
        ringHolderPosition = upPosition;

        currentState = positionRingHolderServoState;
        periodicTask(); // force the servo to move
    }

    public void launchedARing() {
        if (disabled) {
            Log.w(LOG_TAG, "Indexing disabled, not moving due to ring launch");

            resetToUpPosition();

            return; // don't allow auto movement
        }

        Log.d(LOG_TAG, "Now moving to hold " + ringHolderPosition.numberOfRings + " rings");

        currentState = delayAfterLaunch;
        periodicTask();

        ringHolderPosition = ringHolderPosition.nextPosition;
    }

    public void periodicTask() {
        if (currentState != null) {
            State nextState = currentState.doStuffAndGetNextState();

            if (nextState != currentState) {
                Log.d(LOG_TAG, String.format("State transition from %s to %s", currentState.getClass()
                        + "(" + currentState.getName() + ")", nextState.getClass() + "(" + nextState.getName() + ")"));
            }

            currentState = nextState;
        } else {
            Log.e(LOG_TAG, "No state machine setup!");
        }
    }

    class PositionRingHolderServoState extends State {

        protected PositionRingHolderServoState(Telemetry telemetry) {
            super("Ring holder servo", telemetry);
        }

        @Override
        public State doStuffAndGetNextState() {
            ringHoldDownServo.setPosition(ringHolderPosition.servoPosition);

            return this; // keep holding, until something external picks a different state
        }

        @Override
        public void resetToStart() {

        }

        @Override
        public void liveConfigure(NinjaGamePad gamePad) {

        }
    }
}