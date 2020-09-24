/**
 Copyright (c) 2016 HF Robotics (http://www.hfrobots.com)
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

import android.util.Log;

import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.drive.Turn;
import com.hfrobots.tnt.corelib.state.State;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import java.util.concurrent.TimeUnit;

//@TeleOp(name="RR Gyro Turn Test")
@SuppressWarnings("unused")
public class RelicRecoveryGyroTurnTest extends RelicRecoveryHardware {


    private static final String LOG_TAG = "TNT Auto";
    private State currentState = null;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void start() {
        super.start();
        logBatteryState("Auto.start()");
    }

    @Override
    public void stop() {
        super.stop();
        logBatteryState("Auto.stop()");
    }

    @Override
    public void loop() {
        try {
            if (currentState == null) {
                /* We have not configured the state machine yet, do so from the options
                 selected during init_loop() */

                final Turn turn = new Turn(Rotation.CCW, 10);

                MecanumGyroTurnState.Builder turnBuilder = MecanumGyroTurnState.builder();
                turnBuilder.setTurn(turn).setImu(imu).setMecanumDrive(mecanumDrive).setPLargeTurnCoeff(RobotConstants.P_LARGE_TURN_COEFF)
                        .setPSmallTurnCoeff(RobotConstants.P_SMALL_TURN_COEFF).setName("Turn towards cryptobox")
                        .setSafetyTimeoutMillis(TimeUnit.SECONDS.toMillis(15));

                MecanumGyroTurnState turnForward = turnBuilder.build();

                State delayState = newDelayState("wait a bit", 2);

                turnForward.setNextState(delayState);

                turnBuilder = MecanumGyroTurnState.builder();
                turnBuilder.setTurn(turn.invert()).setImu(imu).setMecanumDrive(mecanumDrive).setPLargeTurnCoeff(RobotConstants.P_LARGE_TURN_COEFF)
                        .setPSmallTurnCoeff(RobotConstants.P_SMALL_TURN_COEFF).setName("Turn towards cryptobox")
                        .setSafetyTimeoutMillis(TimeUnit.SECONDS.toMillis(15));

                State turnBackState = turnBuilder.build();
                delayState.setNextState(turnBackState);
                turnBackState.setNextState(newDoneState("Done."));

                currentState = turnForward;
            }

            State nextState = currentState.doStuffAndGetNextState();

            if (nextState != currentState) {
                // We've changed states alert the driving team, log for post-match analysis
                telemetry.addData("00-State", "From %s to %s", currentState, nextState);
                Log.d(LOG_TAG, String.format("State transition from %s to %s", currentState, nextState));
            }

            currentState = nextState;
            telemetry.update(); // send all telemetry to the drivers' station
        } catch (Throwable t) {
            // Better logging than the FTC SDK provides :(
            Log.e("VV", "Exception during state machine", t);

            if (t instanceof RuntimeException) {
                throw (RuntimeException)t;
            }

            RuntimeException rte = new RuntimeException();
            rte.initCause(t);

            throw rte;
        }
    }
}
