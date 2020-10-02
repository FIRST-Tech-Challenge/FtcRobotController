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
package com.hfrobots.tnt.corelib.drive;

import android.util.Log;

import com.ftc9929.corelib.control.NinjaGamePad;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class ProportionalDriveInchesStateExternalControl extends DriveInchesStateExternalControl {
    protected long slowDownEncoderCount;
    protected double inchesToSlowDown = 6;
    protected double minimumPowerLevel = 0.2;
    protected boolean loggedProportionalMode = false;


    public ProportionalDriveInchesStateExternalControl(String name, TankDrive drive, Telemetry telemetry, double inchesToDrive, double powerLevel, long safetyTimeoutMillis) {
        super(name, drive, telemetry, inchesToDrive, powerLevel, safetyTimeoutMillis);
        calculateEncoderCountToSlowDown(drive);
        Log.d("VV", "PropDrive encoderToSlowDown=" + slowDownEncoderCount + "for inches="
                + inchesToSlowDown);
    }

    public ProportionalDriveInchesStateExternalControl(String name, TankDrive drive,
                                                       Telemetry telemetry,
                                                       double inchesToDrive,
                                                       double powerLevel,
                                                       DcMotorSimple.Direction direction,
                                                       long safetyTimeoutMillis) {
        super(name, drive, telemetry, inchesToDrive, powerLevel, direction, safetyTimeoutMillis);
        calculateEncoderCountToSlowDown(drive);
        Log.d("VV", "PropDrive encoderToSlowDown=" + slowDownEncoderCount + "for inches="
                + inchesToSlowDown);
    }

    protected void calculateEncoderCountToSlowDown(TankDrive drive) {
        slowDownEncoderCount = drive.getEncoderCountsForInchesTravel(inchesToSlowDown);
    }

    @Override
    protected double[] calculateNewPowerLevels(TankDrive.SidedTargetPositions currentPositions,
                                               TankDrive.SidedTargetPositions targetPositions) {
        long leftDifference = Math.abs(targetPositions.getLeftTargetPosition()
                - currentPositions.getLeftTargetPosition());

        long rightDifference = Math.abs(targetPositions.getRightTargetPosition()
                - currentPositions.getRightTargetPosition());

        //Log.d("VV", "PropDrive, leftDiff=" + leftDifference + ", rightDiff=" + rightDifference  +
          //      " >=? " + slowDownEncoderCount);

        if (leftDifference >= slowDownEncoderCount || rightDifference >= slowDownEncoderCount) {
            // keep driving normally
            return new double[] {powerLevel, powerLevel};
        }

        final long currentError;

        if (leftDifference >= rightDifference) {
            currentError = leftDifference;
        } else {
            currentError = rightDifference;
        }

        if (loggedProportionalMode) {
            Log.d("VV", "Prop Drive - entered proportional mode, left diff " + leftDifference +
                    ", right diff " + rightDifference + ", chosen diff " + currentError + " of "
                    + slowDownEncoderCount);
            loggedProportionalMode = true;
        }

        double proportion = (double)currentError / (double)slowDownEncoderCount;

        double proportionalPowerLevel = powerLevel * proportion;

        if (Math.abs(proportionalPowerLevel) < minimumPowerLevel) {
            double powerLevelSign = Math.signum(powerLevel);

            proportionalPowerLevel = powerLevelSign * minimumPowerLevel;
        }

        return new double[] { proportionalPowerLevel, proportionalPowerLevel};
    }

    @Override
    public void liveConfigure(NinjaGamePad gamePad) {

    }

    @Override
    public void resetToStart() {
        super.resetToStart();
        loggedProportionalMode = false;
    }
}
