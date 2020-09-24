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

import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * A proportional drive inches state - that when the encoders say the target
 * has been reached, it's cross-checked with a range sensor for a distance
 * to something range-sensor-able, and the drive continues (at minimum power)
 * until within range.
 */
public class DriveInchesWithinRangeStateExternalControl extends ProportionalDriveInchesStateExternalControl {

    protected boolean closingRangeDistance = false;

    protected final ModernRoboticsI2cRangeSensor rangeSensor;

    protected double expectedRangeInches;

    public DriveInchesWithinRangeStateExternalControl(String name, TankDrive drive, Telemetry telemetry,
                                                      ModernRoboticsI2cRangeSensor rangeSensor,
                                                      double inchesToDrive, double expectedRangeInches,
                                                      double powerLevel,
                                                      DcMotorSimple.Direction direction,
                                                      long safetyTimeoutMillis) {
        super(name, drive, telemetry, inchesToDrive, powerLevel, direction, safetyTimeoutMillis);
        this.rangeSensor = rangeSensor;
        this.expectedRangeInches = expectedRangeInches;
    }

    @Override
    protected double[] calculateNewPowerLevels(TankDrive.SidedTargetPositions currentPositions,
                                               TankDrive.SidedTargetPositions targetPositions) {
        if (closingRangeDistance) {
            return new double[] {minimumPowerLevel, minimumPowerLevel};
        }

        // We are not using the range sensor (yet), use the proportional drive code
        return super.calculateNewPowerLevels(currentPositions, targetPositions);
    }

    @Override
    protected boolean isTargetReached(TankDrive.SidedTargetPositions currentPositions) {
        // First, are we done driving with just encoders?
        boolean proportionalDriveEnded = super.isTargetReached(currentPositions);

        // if we aren't what do we do?

        if (!proportionalDriveEnded) {
            return false;
        }

        // if we *are*, then we're using the range sensor
        closingRangeDistance = true;

        // how do we know we're in range with the range sensor? Reflected light may not work
        // so we use ultrasonic
        double rangeInches = rangeSensor.cmUltrasonic() / 2.54;

        // are there any values we'd throw out from the range sensor to be safe?

        // if we're not in range, do we have any other "safety" we can use?

        if (rangeInches <= expectedRangeInches) {
            Log.d("VV", "Range sensor reports in-range, target reached, final range is " + rangeInches + " inches");

            return true;
        }

        Log.d("VV", "Range sensor reports not in-range, target not reached, reported range is " + rangeInches + " inches");

        return false;
    }

    @Override
    public void liveConfigure(DebouncedGamepadButtons buttons) {
        super.liveConfigure(buttons);

        if (buttons.getLeftStickButton().getRise()) {
            expectedRangeInches -= .25;
        } else if (buttons.getRightStickButton().getRise()) {
            expectedRangeInches += .25;
        }

        telemetry.addData("05", "expected range inches " + expectedRangeInches);
    }

    @Override
    public void resetToStart() {
        super.resetToStart();
        closingRangeDistance = false;
    }
}
