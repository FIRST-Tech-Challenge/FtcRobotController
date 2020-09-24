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

import lombok.NonNull;
import android.util.Log;

import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.drive.Turn;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.StateMachine;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import java.util.concurrent.TimeUnit;

// @TeleOp(name="RR Drive Distance Test")
@SuppressWarnings("unused")
public class RelicRecoveryDriveDistanceTest extends RelicRecoveryHardware {
    private StateMachine stateMachine = null;

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
            if (stateMachine == null) {
                stateMachine = commonStateMachineSetup();
                stateMachine.startDebugging();
                stateMachine.addSequential(new MecanumDriveDistanceState("give me a name",
                                telemetry, mecanumDrive, 36.0, TimeUnit.SECONDS.toMillis(15)));
                stateMachine.addSequential(newDoneState("done"));
            }

            stateMachine.doOneStateLoop();

            telemetry.update(); // send all telemetry to the drivers' station
        } catch (Throwable t) {
            // Better logging than the FTC SDK provides :(
            Log.e(Constants.LOG_TAG, "Exception during state machine", t);

            if (t instanceof RuntimeException) {
                throw (RuntimeException)t;
            }

            RuntimeException rte = new RuntimeException();
            rte.initCause(t);

            throw rte;
        }
    }

    @NonNull
    private StateMachine commonStateMachineSetup() {
        StateMachine stateMachine = new StateMachine(telemetry);

        // Setup debugger controls
        stateMachine.setDoOverButton(driverBRedButton);
        stateMachine.setGoBackButton(driverYYellowButton);
        stateMachine.setGoButton(driverAGreenButton);
        stateMachine.setConfigureGamepad(operatorsGamepad);

        return stateMachine;
    }
}
