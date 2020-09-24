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

package com.hfrobots.tnt.season1617;

import android.util.Log;

import com.hfrobots.tnt.corelib.control.OnOffButton;
import com.hfrobots.tnt.corelib.control.RangeInput;
import com.hfrobots.tnt.corelib.drive.TankDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArcadeDrive {
    private final Telemetry telemetry;
    private final TankDrive tankDrive;
    private final RangeInput throttle;
    private final RangeInput steer;
    private final OnOffButton quickTurnMode;
    private final OnOffButton directionFlip;
    private final OnOffButton brakeNoBrake;
    private final OnOffButton lowGear;
    float previousX;
    float previousY;
    long previousSampleTime;

    float maxXRate = Float.MIN_VALUE;
    float maxYRate = Float.MIN_VALUE;


    public ArcadeDrive(final Telemetry telemetry, final TankDrive tankDrive, final RangeInput throttle, final RangeInput steer,
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
        //----------------------------------------------------------------------
        //
        // DC Motors
        //
        // Obtain the current values of the joystick controllers.
        //
        // Note that x and y equal -1 when the joystick is pushed all of the way
        // forward and all the way right (i.e. away from the human holder's body).
        //
        // The clip method guarantees the value never exceeds the range +-1.
        //
        // The DC motors are scaled to make it easier to control them at slower
        // speeds.
        //
        // The setPower methods write the motor power values to the DcMotor
        // class, but the power levels aren't applied until the loop() method ends.
        //

        final boolean isFloat;

        if (brakeNoBrake.isPressed()) {
            isFloat = false;
            tankDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            isFloat = true;
            tankDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        //updateThrottleParameters();

        // these scale the motor power based on the amount of input on the drive stick
        float xStickPosition = steer.getPosition();
        float yStickPosition = throttle.getPosition();

        if (previousSampleTime == 0) {
            previousX = xStickPosition;
            previousY = yStickPosition;
            previousSampleTime = System.currentTimeMillis();
        } else {
            float deltaX = previousX - xStickPosition;
            float deltaY = previousY - yStickPosition;
            long timeMsNow = System.currentTimeMillis();
            long deltaTimeMs = timeMsNow - previousSampleTime;

            float xRate = deltaX / deltaTimeMs;
            float yRate = deltaY / deltaTimeMs;

            if (Math.abs(xRate) > maxXRate) {
                maxXRate = xRate;
                Log.d("VV", "Max xRate " + maxXRate);
            }

            if (Math.abs(yRate) > maxYRate) {
                maxYRate = yRate;
                Log.d("VV", "Max yRate" + maxYRate);
            }

        }

        float xValue = scaleMotorPower(xStickPosition);
        float yValue = -scaleMotorPower(yStickPosition);

        if (directionFlip.isPressed()) {
            yValue = -yValue;
        }

        //calculate the power needed for each motor
        float leftPower = yValue + xValue;
        float rightPower = yValue - xValue;

        //clip the power values so that it only goes from -1 to 1
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        if (lowGear.isPressed()) {
            leftPower = leftPower / 3;
            rightPower = rightPower / 3;
        }

        telemetry.addData("08", "Power (%s) L/R: %5.2f / %5.2f", isFloat ? "F" : "B", leftPower, rightPower);

        //set the power of the motors with the gamepad values
        tankDrive.drivePower(leftPower, rightPower);
    }

    protected float gain = 0.3F;
    protected float deadband = 0;

    float scaleMotorPower(float unscaledPower) {
        if (unscaledPower >= 0) {
            return deadband + (1-deadband)*(gain * (float)Math.pow(unscaledPower, 3) + (1 - gain) * unscaledPower);
        } else {
            return  -1 * deadband + (1-deadband)*(gain * (float)Math.pow(unscaledPower, 3) + (1 - gain) * unscaledPower);
        }
    }
}
