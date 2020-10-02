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

// Inspired by, ported from FRC Team 254, "The Cheesy Poofs" Cheesy Drive
package com.hfrobots.tnt.corelib.drive;

import com.ftc9929.corelib.control.OnOffButton;
import com.ftc9929.corelib.control.RangeInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CheesyDrive {
    private final Telemetry telemetry;
    private final TankDrive tankDrive;
    private final RangeInput throttle;
    private final RangeInput steer;
    private final OnOffButton quickTurnMode;
    private final OnOffButton directionFlip;
    private final OnOffButton brakeNoBrake;
    private final OnOffButton lowGear;
    private double quickStopAccumulator;
    public static final double throttleDeadband = 0.02;
    private static final double steerDeadband = 0.02;
    private static final double kTurnSensitivity = 1.5; // < 2 or it turns into "quick turn"

    public CheesyDrive(final Telemetry telemetry, final TankDrive tankDrive, final RangeInput throttle, final RangeInput steer,
                       final OnOffButton quickTurnMode, final OnOffButton directionFlip,
                       final OnOffButton brakeNoBrake,
                       final OnOffButton lowGear) {
        this.telemetry = telemetry;
        this.tankDrive = tankDrive;
        this.throttle = throttle;
        this.steer = steer;
        this.quickTurnMode = quickTurnMode;
        this.directionFlip = directionFlip;
        this.brakeNoBrake = brakeNoBrake;
        this.lowGear = lowGear;
    }

    public void handleDrive() {
        double steerPosition = steer.getPosition();
        double throttlePosition = throttle.getPosition();

        if (directionFlip.isPressed()) {
            throttlePosition = - throttlePosition;
        }

        steerPosition = handleDeadband(steerPosition, steerDeadband);
        throttlePosition = -handleDeadband(throttlePosition, throttleDeadband);

        //steerPosition = scaleValue(steerPosition);
        throttlePosition = scaleValue(throttlePosition);

        boolean isQuickTurn = throttlePosition == 0.0D;

        double overPower;

        double angularPower;

        if (isQuickTurn) {
            if (Math.abs(throttlePosition) < 0.2) {
                double alpha = 0.1;
                quickStopAccumulator = (1 - alpha) * quickStopAccumulator + alpha
                        * limit(steerPosition, 1.0) * 2;
            }
            overPower = 1.0;
            angularPower = steerPosition;
        } else {
            overPower = 0.0;
            angularPower = Math.abs(throttlePosition) * steerPosition * kTurnSensitivity - quickStopAccumulator;
            if (quickStopAccumulator > 1) {
                quickStopAccumulator -= 1;
            } else if (quickStopAccumulator < -1) {
                quickStopAccumulator += 1;
            } else {
                quickStopAccumulator = 0.0;
            }
        }

        double rightMotorPower = throttlePosition - angularPower;
        double leftMotorPower = throttlePosition + angularPower;

        if (leftMotorPower > 1.0) {
            rightMotorPower -= overPower * (leftMotorPower - 1.0);
            leftMotorPower = 1.0;
        } else if (rightMotorPower > 1.0) {
            leftMotorPower -= overPower * (rightMotorPower - 1.0);
            rightMotorPower = 1.0;
        } else if (leftMotorPower < -1.0) {
            rightMotorPower += overPower * (-1.0 - leftMotorPower);
            leftMotorPower = -1.0;
        } else if (rightMotorPower < -1.0) {
            leftMotorPower += overPower * (-1.0 - rightMotorPower);
            rightMotorPower = -1.0;
        }

        final boolean isFloat;

        if (lowGear.isPressed()) {
            leftMotorPower = leftMotorPower / 3;
            rightMotorPower = rightMotorPower / 3;
            isFloat = false;
            tankDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            if (brakeNoBrake.isPressed()) {
                isFloat = false;
                tankDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                isFloat = true;
                tankDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }

        telemetry.addData("08", "Power (%s) L/R: %5.2f / %5.2f", isFloat ? "F" : "B", leftMotorPower, rightMotorPower);

        tankDrive.drivePower(leftMotorPower, rightMotorPower);
    }

    private double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

    private static double limit(double v, double limit) {
        return (Math.abs(v) < limit) ? v : limit * (v < 0 ? -1 : 1);
    }

    protected float gain = 0.3F;
    protected float deadband = 0;

    double scaleValue(double unscaledPower) {
        if (unscaledPower >= 0) {
            return deadband + (1-deadband)*(gain * (float)Math.pow(unscaledPower, 3) + (1 - gain) * unscaledPower);
        } else {
            return  -1 * deadband + (1-deadband)*(gain * (float)Math.pow(unscaledPower, 3) + (1 - gain) * unscaledPower);
        }
    }
}
