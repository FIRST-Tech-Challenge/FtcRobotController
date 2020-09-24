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

package com.hfrobots.tnt.corelib.drive;

import android.util.Log;

import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * State machine state that will drive a TankDrive the given number of inches. Notice that
 * this class assumes we're using "west coast" dual-motor drive and implements it's own
 * encoder value tracking since FTC motor controllers can't handle two motors on same output
 * or run competing PID loops (no way to synchronize).
 */
public class DriveInchesStateExternalControl extends TimeoutSafetyState {
    protected final TankDrive drive;
    protected double powerLevel;
    protected double inchesToDrive;
    protected boolean driveStarted = false;
    protected TankDrive.SidedTargetPositions targetPositions;
    protected final static long THRESHOLD_ENCODER_VALUE = 10;
    protected final DcMotorSimple.Direction driveDirection;

    /**
     * Constructs a state machine state that will drive the TankDrive the number of inches (- is reverse)
     * at the given power level. The state will transition to the state given by setNextState() when
     * the distance is reached (within a threshold), or if the timeout is reached.
     */
    // TODO - would be nice to auto-calculate the timeout here based on distance/power
    public DriveInchesStateExternalControl(String name, TankDrive drive,
                                           Telemetry telemetry,
                                           double inchesToDrive,
                                           double powerLevel,
                                           long safetyTimeoutMillis) {
        this(name, drive, telemetry, inchesToDrive, powerLevel, DcMotorSimple.Direction.FORWARD, safetyTimeoutMillis);
    }

    public DriveInchesStateExternalControl(String name, TankDrive drive,
                                           Telemetry telemetry,
                                           double inchesToDrive,
                                           double powerLevel,
                                           DcMotorSimple.Direction direction,
                                           long safetyTimeoutMillis) {
        super(name, telemetry, safetyTimeoutMillis);
        this.drive = drive;
        this.powerLevel = powerLevel;
        this.inchesToDrive = inchesToDrive;
        this.driveDirection = direction;
    }


    @Override
    public State doStuffAndGetNextState() {
        if (!driveStarted) {
            if (driveDirection != DcMotorSimple.Direction.FORWARD) {
                drive.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            targetPositions = drive.getTargetPositionsForInchesTravel(inchesToDrive);

            TankDrive.SidedTargetPositions currentPositions = drive.getCurrentPositions();

            Log.d("VV", "DriveInches - starting drive of " + inchesToDrive + "inches -> l_cur="
                    + currentPositions.getLeftTargetPosition() + ", r_cur=" +
                    + currentPositions.getRightTargetPosition() + ", l_target="
                    + targetPositions.getLeftTargetPosition() + "r_target="
                    + targetPositions.getRightTargetPosition());

            drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive.drivePower(powerLevel, powerLevel);
            driveStarted = true;

            return this;
        }

        TankDrive.SidedTargetPositions currentPositions = drive.getCurrentPositions();

        telemetry.addData("04", "inches tl/tr/cl/cr: " + targetPositions.getLeftTargetPosition() + "/"
                + targetPositions.getRightTargetPosition() + "/" + currentPositions.getLeftTargetPosition() + "/"
                + currentPositions.getRightTargetPosition());

        if (isTargetReached(currentPositions)) {
            Log.d("VV", "Target reached - stopping drive");
            stopDriving();

            return nextState;
        }

        if (isTimedOut()) {
            Log.e("VV", "Drive inches state timed out - stopping drive. Current encoder positions "+
                "l_cur=" + currentPositions.getLeftTargetPosition() + ", r_cur="
                    + currentPositions.getRightTargetPosition() + ", l_target=" +
                    targetPositions.getLeftTargetPosition() + "r_target=" +
                    targetPositions.getRightTargetPosition());

            stopDriving();

            return nextState;
        }

        double[] newPowerLevels = calculateNewPowerLevels(currentPositions, targetPositions);
        drive.drivePower(newPowerLevels[0], newPowerLevels[1]);

        return this;
    }

    /**
     * Is the current target position reached? In this class, this is evaluated simply
     * on encoder counts being exceeded or w/in thresholds. Sub-classes may provide
     * more complex behavior (range/distance sensing)
     */
    protected boolean isTargetReached(TankDrive.SidedTargetPositions currentPositions) {
        if (currentPositions.getLeftTargetPosition() >= targetPositions.getLeftTargetPosition() ||
                currentPositions.getRightTargetPosition() >= targetPositions.getRightTargetPosition()) {
            Log.d("VV", "Target reached. Encoder positions of l_target=" + targetPositions.getLeftTargetPosition() +
                    ", r_target=" + targetPositions.getRightTargetPosition()
                    + " met or exceeded l_cur=" + currentPositions.getLeftTargetPosition()
                    + ", r_cur=" + currentPositions.getRightTargetPosition());

            return true;
        }

        long leftDifference = Math.abs(targetPositions.getLeftTargetPosition() - currentPositions.getLeftTargetPosition());
        long rightDifference = Math.abs(targetPositions.getRightTargetPosition() - currentPositions.getRightTargetPosition());

        if (leftDifference < THRESHOLD_ENCODER_VALUE && rightDifference < THRESHOLD_ENCODER_VALUE) {
            Log.d("VV", "Target reached. Encoder positions reached w/in threshold of " +
                    THRESHOLD_ENCODER_VALUE + ", left_diff=" + leftDifference + ", right_diff=" +
                    rightDifference);

            return true;
        }

        return false;
    }

    /**
     * Calculate new left/right power levels based on current encoder values (or other sensors
     * in subclasses)
     */
    protected double[] calculateNewPowerLevels(TankDrive.SidedTargetPositions currentPositions,
                                               TankDrive.SidedTargetPositions targetPositions) {
        return new double[] { powerLevel, powerLevel };
    }

    @Override
    public void resetToStart() {
        super.resetToStart();
        driveStarted = false;
        targetPositions = null;
    }

    @Override
    public void liveConfigure(DebouncedGamepadButtons buttons) {

        if (buttons.getLeftBumper().getRise()) {
            inchesToDrive -= .25;
        } else if (buttons.getRightBumper().getRise()) {
            inchesToDrive += .25;
        }

        if (buttons.getaButton().getRise()) {
            powerLevel -= .1;
        } else if (buttons.getyButton().getRise()) {
            powerLevel += .1;
        }

        telemetry.addData("03", "power level " + powerLevel);
        telemetry.addData("04", "inches to drive " + inchesToDrive);
    }

    private void stopDriving() {
        // (1) Stop the motors
        drive.drivePower(0, 0);
        // (2) "reset" the motors/drive train to a "normal" state, which is?
        drive.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (driveDirection != DcMotorSimple.Direction.FORWARD) {
            drive.setDirection(DcMotorSimple.Direction.FORWARD); // back to normal
        }
    }
}
