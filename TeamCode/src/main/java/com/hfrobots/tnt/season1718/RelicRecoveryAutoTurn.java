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


import android.util.Log;

import com.hfrobots.tnt.corelib.control.DebouncedButton;
import com.hfrobots.tnt.corelib.control.NinjaGamePad;
import com.hfrobots.tnt.corelib.drive.Turn;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

/**
 * Proof of concept of an IMU-based turn for the Mecanum drive
 */
@TeleOp(name="MecanumBot Auto Turn")
@Disabled
public class RelicRecoveryAutoTurn extends RelicRecoveryTelemetry

{
    private static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    private double     P_TURN_COEFF            = 0.02;     // Larger is more responsive, but also less stable

    private boolean useEncoders = true;

    private boolean useBraking = true;

    private Turn turn;

    private int degreesToTurn = 90;

    private double powerCutOff = 0.02;

    private boolean initialized = false;

    private boolean reachedTarget = false;

    private float targetHeading;

    private NinjaGamePad driversGamepad;

    private DebouncedButton brakeButton;

    private DebouncedButton encoderButton;

    private DebouncedButton goButton;

    private DebouncedButton stopButton;

    private DebouncedButton increaseCoeButton;

    private DebouncedButton decreaseCoeButton;

    private DebouncedButton increaseTurnButton;

    private DebouncedButton decreaseTurnButton;

    private DebouncedButton increasePowerCutOff;

    private DebouncedButton decreasePowerCutOff;

    private long lastCycleTimestamp = 0;

    private boolean doingTurn = false;

    public RelicRecoveryAutoTurn() {
    }

    /**
     * Places the robot into the initial pre-running state.
     * Robot must fit within sizing guidelines (18x18)
     * before this state completes.
     */
    @Override
    public void init() {
        super.init();
        driversGamepad = new NinjaGamePad(gamepad1);
        brakeButton = new DebouncedButton(driversGamepad.getYButton());
        encoderButton = new DebouncedButton(driversGamepad.getXButton());
        goButton = new DebouncedButton(driversGamepad.getAButton());
        stopButton = new DebouncedButton(driversGamepad.getBButton());
        increaseCoeButton = new DebouncedButton(driversGamepad.getDpadUp());
        decreaseCoeButton = new DebouncedButton(driversGamepad.getDpadDown());
        increaseTurnButton = new DebouncedButton(driversGamepad.getDpadRight());
        decreaseTurnButton = new DebouncedButton(driversGamepad.getDpadLeft());
        increasePowerCutOff = new DebouncedButton(driversGamepad.getRightBumper());
        decreasePowerCutOff = new DebouncedButton(driversGamepad.getLeftBumper());

    }


    /**
     * Implement a state machine that controls the robot during
     * manual-operation.  The state machine uses gamepad input to transition
     * between states.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override public void loop ()

    {
        if (stopButton.getRise()) {
            // Stop whatever we're doing
            doingTurn = false;
            mecanumDrive.stopAllDriveMotors();
        }

        if (doingTurn) {
            // We're doing an autonomous turn, so run that code
            doTurnLoop();
        } else if (goButton.getRise()) {
            // We weren't doing a turn, we've been asked to start one
            initialized = false;
            doingTurn = true;
        } else {
            // Do configuration of the drive base/turn parameters
            if (brakeButton.getRise()) {
                useBraking = !useBraking;
            }

            if (encoderButton.getRise()) {
                useEncoders = !useEncoders;
            }

            if (increaseCoeButton.getRise()) {
                P_TURN_COEFF += 0.01;
            }

            if (decreaseCoeButton.getRise()) {
                P_TURN_COEFF -= 0.01;
            }
            if (increaseTurnButton.getRise()) {
                degreesToTurn += 5;
            }
            if (decreaseTurnButton.getRise()) {
                degreesToTurn -= 5;
            }
            if (increasePowerCutOff.getRise()) {
                powerCutOff += .005;
            }
            if (decreasePowerCutOff.getRise()){
                powerCutOff -= .005;
            }

            telemetry.addData("imu", imu.getCalibrationStatus().toString());
            telemetry.addData("brake / enc", useBraking + " / " + useEncoders);
            telemetry.addData("P_COEFF",  P_TURN_COEFF);
            telemetry.addData("DEGREES_TO_TURN", degreesToTurn);
            telemetry.addData("POWER_CUT_OFF",powerCutOff);
            float currentHeading = imu.getAngularOrientation().firstAngle;
            Log.d(LOG_TAG, "currHeading " + currentHeading);
            telemetry.addData("Heading", currentHeading);
            updateTelemetry();
        }
    }

    protected void doTurnLoop() {
        if (!initialized) {
            turn = new Turn(Rotation.CCW, degreesToTurn);
            // not-yet initialized
            // First angle is heading, second is roll, third is pitch
            reachedTarget = false;
            float currentHeading = imu.getAngularOrientation().firstAngle;
            float turnHeading = turn.getHeading();
            targetHeading = currentHeading + turnHeading;

            configureMotorParameters();

            Log.d(LOG_TAG, "Gyro turn initialized - current heading: " + currentHeading + ", relative turn heading: " + turnHeading + ", target heading: " + targetHeading);
            initialized = true;
        }

        if (!reachedTarget) {
            reachedTarget = onHeading(targetHeading);

            if (lastCycleTimestamp != 0) {
                long elapsedCycleTimeMs = System.currentTimeMillis() - lastCycleTimestamp;
                Log.d(LOG_TAG, "Cycle time ms: " + elapsedCycleTimeMs);
            }

            lastCycleTimestamp = System.currentTimeMillis();

            if (reachedTarget) {
                Log.d(LOG_TAG, "Gyro turn heading reached - stopping drive");

                mecanumDrive.driveCartesian(0, 0, 0, false, 0.0);

                mecanumDrive.stopAllDriveMotors();
                doingTurn = false;
            }
        }
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

    // re-use of Pushbot gyro steer

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param target     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @return
     */
    boolean onHeading(float target) {
        float   error ;
        double   steer ;
        boolean  onTarget = false;

        // determine turn power based on +/- error

        // calculate error in -179 to +180 range  (
        float currentHeading = imu.getAngularOrientation().firstAngle;
        error = target - currentHeading;

        while (error > 180)  {
            Log.d(LOG_TAG, "Error is > 180, making smaller angle in other direction " + error);
            error -= 360;
        }

        while (error <= -180) {
            Log.d(LOG_TAG, "Error is > 180, making smaller angle in other direction " + error);
            error += 360;
        }

        double MAX_POWER = .3;

        Log.d(LOG_TAG, "Error: " + error);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            onTarget = true;
        } else {
            steer = Range.clip(error * P_TURN_COEFF, -MAX_POWER, MAX_POWER);
        }

        if (Math.abs (steer) <= powerCutOff) {
            if (steer >= 0) {
                steer = powerCutOff;
            } else {
                steer = -powerCutOff;
            }
        }

        // Ok, let's tell the robot base what to do...
        // What speed x direction?
        // What speed y direction?
        // What rotation?

        Log.d(LOG_TAG, String.format("Target, Curr, Err, St %5.2f , %5.2f , %5.2f , %f",
                target, currentHeading, error, steer));

        mecanumDrive.driveCartesian(0, 0, steer, false, 0.0);

        return onTarget;
    }
}