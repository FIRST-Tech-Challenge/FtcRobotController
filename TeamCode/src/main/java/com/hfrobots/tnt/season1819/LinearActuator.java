/**
 Copyright (c) 2018 HF Robotics (http://www.hfrobots.com)
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

package com.hfrobots.tnt.season1819;

import android.util.Log;

import com.hfrobots.tnt.corelib.Constants;
import com.hfrobots.tnt.corelib.drive.PidController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

// (Friday)
// spacer wasn't long enough, gear ground against side of drivebase
// kaylin was tired
// kaylin taught katelyn about the drill press
// need something to "kick" the robot out so it doesn't tilt (4")

// (Sunday)
// Added hard stop (but drilled hole in wrong spot)
// Experimented with magnetic limit switch, mounted magnet to linear actuator

public class LinearActuator {
    private final DcMotor motor;

    /**
     * Notes about lead screw attributes (from https://www.servocity.com/lead-screws)
     *
     * Each rotation of the lead screw will drive the mating nut precisely 8mm
     *
     * The LinearActuator currently has an NR40 motor, which has 1120 encoder counts/revolution
     *
     * We should eventually be able to use this knowledge to add distance-specific travel methods
     * to this class...
     */
    // 1090 ticks per inch
    private int distanceToTravel = 8400;  // MM count for a NR-20 Orbital, calculated from prior
                                          // value of 11,500 for a NR-20 non-orbital plus gear!!

    private int currentLowPosition = Integer.MIN_VALUE;

    private int currentHighPosition = Integer.MIN_VALUE;

    private final DigitalChannel lowPositionLimitSwitch;

    private boolean isHoming = false;

    private boolean hasHomed = false;

    public LinearActuator(final DcMotor motor, final DigitalChannel lowPositionLimitSwitch) {
        this.motor = motor;

        this.lowPositionLimitSwitch = lowPositionLimitSwitch;

        if (isLowerLimitReached()) {
            hasHomed = true;
            Log.d(Constants.LOG_TAG, "homed at start");
            calculateEncoderLimits();
        }
    }

    public void home() {
        Log.d(Constants.LOG_TAG, "linear actuator homing"); // tell them we're homing

        isHoming = true;
    }

    // SuperFIXME: Is there a way to magically do this?

    public void doPeriodicTask() {
        if (isHoming) {
            // check limit switch
            if (isLowerLimitReached()) {
                // do what?
                hasHomed = true;
                // (1) stop homing
                isHoming = false;
                stopMoving();

                // (2) measure current low position and recompute current high position

                calculateEncoderLimits();
            } else {
                motor.setPower(-0.5);
            }
        }
    }

    private void calculateEncoderLimits() {
        currentLowPosition = motor.getCurrentPosition();
        currentHighPosition = currentLowPosition + distanceToTravel;
        // // tell everybody what values we came up with for current low position, high position

        Log.d(Constants.LOG_TAG, String.format("Linear actuator currentLowPos: %d, currentHighPos: %d",
                currentLowPosition, currentHighPosition));
    }

    public boolean isLowerLimitReached() {
        return !lowPositionLimitSwitch.getState();
    }

    public void extend(boolean ignoreLimits /* use with caution !!! */) {
        double direction = 1;
        double powerLevel = 1;

        if (ignoreLimits) {
            Log.d(Constants.LOG_TAG, "Not enforcing limits when extending");
        }

        if (!ignoreLimits && hasHomed) {
            int currentEncoderPosition = motor.getCurrentPosition();

            if (currentEncoderPosition >= currentHighPosition) {
                Log.d(Constants.LOG_TAG,
                        String.format("Linear actuator reached high limit of encoder position: %d",
                                currentEncoderPosition));
                motor.setPower(0);

                return;
            }
        }

        motor.setPower(powerLevel * direction);

    }

    private PidController pidController;

    private double kP = .002;

    private double powerLevel = 1.0;

    public void extendToMax() {
        // Hint for tolerance - 1120 encoder ticks per full revolution, full revolution moves lead screw
        // 8mm...so what * distance * in mm of error is okay, then convert that to encoder ticks

        pidController = PidController.builder().setInstanceName("AcDc pid-controller")
                .setKp(kP).setAllowOscillation(false)
                .setTolerance(140)
                .build();
        pidController.setOutputRange(-powerLevel, powerLevel);
        pidController.setAbsoluteSetPoint(true); // MM - Discuss with Team, have them re-debug this

        int currentPosition = motor.getCurrentPosition();

        if (hasHomed) {
            Log.d(Constants.LOG_TAG,
                    "Ascender descender starting from home position, going from " +
                            currentPosition + " to position " + currentHighPosition);

            pidController.setTarget(currentHighPosition, currentPosition);
        } else {
            // FIXME: There is a bug, we saw it on the evening of 11/30
            // Hint, what is currentHighPosition if the robot has not homed?
            int targetPosition = currentPosition + distanceToTravel;

            Log.d(Constants.LOG_TAG,
                    "Ascender descender starting from unknown position! Going from " +
                            currentPosition + " to position " + targetPosition);

            pidController.setTarget(targetPosition /* currentHighPosition */, currentPosition);
        }
    }

    public boolean hasExtended() {
        if (pidController.isOnTarget()) {
            Log.d(Constants.LOG_TAG, "ascender descender on target, stopping");

            stopMoving();

            return true;
        }

        double pidPower = pidController.getOutput(motor.getCurrentPosition());

        if (pidPower < .05) {
            pidPower = .05;
        }

        motor.setPower(pidPower);
        Log.d(Constants.LOG_TAG, "Have not reached target, setting ascender descender power to:" + pidPower);

        return false;
    }

    public boolean isHoming() {
        return isHoming;
    }

    public void stopMoving() {
        isHoming = false;
        motor.setPower(0);
    }

    public void retract(boolean ignoreLimits /* use with caution !!! */) {
        double direction = -1;
        double powerLevel = 1;

        if (ignoreLimits) {
            Log.d(Constants.LOG_TAG, "Not enforcing limits when retracting");
        }

        if (!ignoreLimits && isLowerLimitReached()) {
            stopMoving();

            if (!hasHomed) {
                hasHomed = true;
                Log.d(Constants.LOG_TAG, "Found home while calling retract()");
                calculateEncoderLimits();
            }

            return;
        }

        motor.setPower(powerLevel * direction);
    }
}
