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

import com.hfrobots.tnt.corelib.state.StateMachine;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name="Timing Gate")
@Disabled
@SuppressWarnings("unused")
public class TimingGate extends OpMode {
    private StateMachine stateMachine;
    private OdsBallSensorState detectBallStart;
    private OdsBallSensorState detectBallEnd;
    private double lastVelocityMetersPerSecond = 0;
    private ResetTimersState resetTimersState;

    @Override
    public void init() {
        OpticalDistanceSensor odsStart = hardwareMap.opticalDistanceSensor.get("ODS start");
        OpticalDistanceSensor odsEnd = hardwareMap.opticalDistanceSensor.get("ODS end");

        detectBallStart = new OdsBallSensorState("Detect ball start", telemetry,
                odsStart, 0 /* don't timeout */);

        detectBallEnd =  new OdsBallSensorState("Detect ball end", telemetry,
                odsEnd, 45 * 1000 /* 45 seconds */);

        resetTimersState = new ResetTimersState(telemetry, detectBallStart, detectBallEnd);

        // after reset, start detecting ball at the beginning again
        resetTimersState.setNextState(detectBallStart);

        stateMachine = new StateMachine(telemetry);
        stateMachine.addSequential(detectBallStart);
        stateMachine.addSequential(detectBallEnd);
        detectBallEnd.setNextState(resetTimersState); // restart after measuring
        detectBallEnd.setStateWhenTimedOut(resetTimersState); // back to the beginning if we timed out

    }

    @Override
    public void loop() {
        stateMachine.doOneStateLoop();

        long currentStartTimer = detectBallStart.getTimerValue();
        long currentEndTimer = detectBallEnd.getTimerValue();

        if (currentStartTimer == Long.MIN_VALUE) {
            // have not yet seen the ball
            telemetry.addData("Start", "waiting to detect object");
        } else if (currentStartTimer != Long.MIN_VALUE && currentEndTimer == Long.MIN_VALUE) {
            // have seen the ball at start, haven't seen at end
            telemetry.addData("Start", "detected object");
            telemetry.addData("End", "waiting to detect object");
        }

        Double timerVelocity = resetTimersState.latestVelocity.get();

        if (timerVelocity != null) {
            lastVelocityMetersPerSecond = timerVelocity;
            telemetry.addData("Last velocity ", "%.2f m/s", lastVelocityMetersPerSecond);
        } else {
            telemetry.addData("Last velocity ", "---- m/s");
        }


        updateTelemetry(telemetry);
    }
}
