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

import com.ftc9929.corelib.control.NinjaGamePad;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

/**
 * Proof of concept of an IMU-based turn for the Mecanum drive
 */
@TeleOp(name="MecanumBot Auto Distance")
@Disabled
public class RelicRecoveryAutoDriveDistance extends RelicRecoveryTelemetry

{
    private boolean initialized = false;

    private long lastCycleTimestamp = 0;

    private NinjaGamePad driversGamepad;

    //NFNT could be cauesed by missicalculated cirumpfranc
    // ENB corection work but we don't know what is the cause of needing it
    private double inchesToDrive = 3D * 12D * .9D;

    public RelicRecoveryAutoDriveDistance() {
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
        rightFrontDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRearDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    /**
     * Implement a state machine that controls the robot during
     * manual-operation.
     * <p>
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override
    public void loop()

    {
        if (!initialized) {
            // (1) - Compute encoder counts for each wheel...remember some rotate "backwards" - later
            // get this working with "drivetrain" class

            double encoderCountPerRev = 1120;

            double wheelDiaInches = 4;

            Log.d(LOG_TAG, "inches to drive (adj) " + inchesToDrive);

            double encoderCountForDistance = (inchesToDrive / (wheelDiaInches * Math.PI)) * encoderCountPerRev;

            // ENB: right side rotates clockwise (backwards)

            int curRightFrontPos = rightFrontDriveMotor.getCurrentPosition();
            int curLeftFrontPos = leftFrontDriveMotor.getCurrentPosition();
            int curRightRearPos = rightRearDriveMotor.getCurrentPosition();
            int curLeftRearPos = leftRearDriveMotor.getCurrentPosition();

            int rightFrontTargetPos = curRightFrontPos + (int) encoderCountForDistance;
            rightFrontDriveMotor.setTargetPosition(rightFrontTargetPos);
            int rightRearTargetPos = curRightRearPos + (int) encoderCountForDistance;
            rightRearDriveMotor.setTargetPosition(rightRearTargetPos);
            int leftFrontTargetPos = curLeftFrontPos + (int) encoderCountForDistance;
            leftFrontDriveMotor.setTargetPosition(leftFrontTargetPos);
            int leftRearTargetPos = curLeftRearPos + (int) encoderCountForDistance;
            leftRearDriveMotor.setTargetPosition(leftRearTargetPos);

            // ENB: Need to set motor mode to run_to_position, otherwise errors happen
            for (DcMotor motor : mecanumDrive.motors) {
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            Log.d(LOG_TAG, "encD " + encoderCountForDistance);
            Log.d(LOG_TAG, "RFP " + rightFrontTargetPos);
            Log.d(LOG_TAG, "RRP" + rightRearTargetPos);
            Log.d(LOG_TAG, "LFP " + leftFrontTargetPos);
            Log.d(LOG_TAG, "LRP" + leftRearTargetPos);
            // (2) - Cartesian drive with the correct direction

            for (DcMotor motor : mecanumDrive.motors) {
                motor.setPower(.5);
            }

            initialized = true;
            updateTelemetry();
        } else {
            // (3) - Continue to check if the hub is driving the motors, stop doing stuff once done

            int curRightFrontPos = rightFrontDriveMotor.getCurrentPosition();
            int curLeftFrontPos = leftFrontDriveMotor.getCurrentPosition();
            int curRightRearPos = rightRearDriveMotor.getCurrentPosition();
            int curLeftRearPos = leftRearDriveMotor.getCurrentPosition();
            //ENB we don't see origoinal telemetry, refreshed so fast that we didn't see ir
            telemetry.addData("RFP", curRightFrontPos);
            telemetry.addData("RRP", curLeftFrontPos);
            telemetry.addData("LFP", curRightRearPos);
            telemetry.addData("LRP", curLeftRearPos);

            Log.d(LOG_TAG, "curPos RF/RR/LF/LR " + curRightFrontPos + "/" + curRightRearPos + "/" + curLeftFrontPos + "/" + curLeftRearPos);
            if (!rightFrontDriveMotor.isBusy()
                && !rightRearDriveMotor.isBusy()
                && !leftFrontDriveMotor.isBusy()
                && !leftRearDriveMotor.isBusy()) {
                mecanumDrive.stopAllDriveMotors();
                telemetry.addData("END", "Reached target position");
                updateTelemetry();
            }
        }

    }
}