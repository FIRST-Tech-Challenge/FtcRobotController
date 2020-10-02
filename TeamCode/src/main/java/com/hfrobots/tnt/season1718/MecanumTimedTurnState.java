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

package com.hfrobots.tnt.season1718;


import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.state.State;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/**
 * Proof of concept of an time-based turn for the Mecanum drive
 */
public class MecanumTimedTurnState extends TimeoutSafetyState
{
    private static final double POWER_LEVEL = 0.2;

    private boolean useEncoders = true;

    private boolean useBraking = true;

    private final Rotation direction;

    private final MecanumDrive mecanumDrive;

    private long lastCycleTimestamp = 0;

    private boolean doingTurn = false;

    private boolean initialized = false;

    public MecanumTimedTurnState(String name, Telemetry telemetry, MecanumDrive mecanumDrive, Rotation direction, long timeMillis) {
        super(name, telemetry, timeMillis);
        this.mecanumDrive = mecanumDrive;
        this.direction = direction;
    }

    public State doStuffAndGetNextState() {
        if (!initialized) {
            initialize();
        }

        if (!isTimedOut()) {
            return this;
        }

        mecanumDrive.stopAllDriveMotors();

        return nextState;
    }

    private void initialize() {
        // not-yet initialized

        configureMotorParameters();

        if (direction.equals(Rotation.CCW)) {
            mecanumDrive.driveCartesian(0, 0, POWER_LEVEL, false, 0);
        } else {
            mecanumDrive.driveCartesian(0, 0, -POWER_LEVEL, false, 0);
        }

        initialized = true;
    }

    @Override
    public void resetToStart() {
        super.resetToStart();
        initialize();
    }

    @Override
    public void liveConfigure(NinjaGamePad gamePad) {

    }

    private void configureMotorParameters() {
        for (DcMotor motor : mecanumDrive.motors) {
            if (useEncoders) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (useBraking) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }
    }
}
