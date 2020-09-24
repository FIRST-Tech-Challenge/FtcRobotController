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

import com.hfrobots.tnt.corelib.control.DebouncedGamepadButtons;
import com.hfrobots.tnt.corelib.drive.PidController;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class MecanumDriveDistanceState extends TimeoutSafetyState {
    private boolean initialized = false;

    private final MecanumDrive mecanumDrive;

    private double inchesToDrive;

    private double powerLevel = 0.3D;

    private double pidP;

    private double pidD = 0.0D;

    private double pidI = 0.0D;

    private PidController pid;

    public MecanumDriveDistanceState(String name, Telemetry telemetry, MecanumDrive mecanumDrive, double inchesToDrive, long timeoutMillis) {
        super(name, telemetry, timeoutMillis);
        this.mecanumDrive = mecanumDrive;

        this.inchesToDrive = inchesToDrive;

        if (inchesToDrive > RobotConstants.P_SMALL_Y_MOVE_THRESHOLD_INCHES) {
            pidP = RobotConstants.P_LARGE_Y_MOVE_COEFF;
        } else {
            pidP = RobotConstants.P_SMALL_Y_MOVE_COEFF;
        }

        setupPidController();
        /* we are not Oscillating because we are driving in a striaght line, "hunting" for the correct value
        takes time. Time >= than extreme accuracy.
         */
    }

    private void setupPidController() {
        this.pid = PidController.builder().setInstanceName("y-axis-drive").setKp(pidP).setTolerance(5).setSettlingTimeMs(100).build();
        this.pid.setOutputRange(-powerLevel, powerLevel);
        this.pid.setNoOscillation(true);
    }

    @Override
    public State doStuffAndGetNextState() {
        if (!initialized) {
            initialize();
        }  else if (isTimedOut()) {
            Log.e(LOG_TAG, "drive distance timeout reached - stopping drive");

            mecanumDrive.stopAllDriveMotors();
            return nextState;
        } else {
            // handle PID control
            double power = pid.getOutput(mecanumDrive.getYPosition());

            if (pid.isOnTarget()) {
                mecanumDrive.stopAllDriveMotors();

                return nextState;
            }

            // FIXME: THis is inverted for some reason
            power = - power;
            mecanumDrive.driveCartesian(0, power, 0, false, 0);

            // TODO: now that we have a power value what do we do with it?
        }

        return this;
    }

    private void initialize() {
        initializeMotors();

        // (1) - Compute encoder counts for each wheel...remember some rotate "backwards" - later
        // get this working with "drivetrain" class

        double encoderCountPerRev = mecanumDrive.leftFrontDriveMotor.getEncoderCountsPerRevolution(); // assume all the same

        double wheelDiaInches = 4;

        Log.d(LOG_TAG, "inches to drive " + inchesToDrive);

        double encoderCountForDistance = (inchesToDrive / (wheelDiaInches * Math.PI)) * encoderCountPerRev;

        mecanumDrive.resetOdometry();

        pid.setTarget(encoderCountForDistance, mecanumDrive.getYPosition());
        initialized = true;
    }

    private void initializeMotors() {
        for (DcMotor motor : mecanumDrive.motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    @Override
    public void resetToStart() {
        super.resetToStart();
        pid.reset();
        initialize();
    }

    @Override
    public void liveConfigure(DebouncedGamepadButtons buttons) {
        if (buttons.getLeftBumper().getRise()) {
            inchesToDrive -= .25;
        } else if (buttons.getRightBumper().getRise()) {
            inchesToDrive += .25;
        }

        if (buttons.getaButton().getRise()) {
            powerLevel -= .025;
        } else if (buttons.getyButton().getRise()) {
            powerLevel += .025;
        }

        boolean pidChanged = false;

        if (buttons.getDpadUp().getRise()) {
            pidP += .01;
            pidChanged = true;
        } else if (buttons.getDpadDown().getRise()) {
            pidP -= 01;
            pidChanged = true;
        }

        if (buttons.getDpadLeft().getRise()) {
            pidI -= .01;
            pidChanged = true;
        } else if (buttons.getDpadRight().getRise()) {
            pidI += .01;
            pidChanged = true;
        }

        if (buttons.getLeftStickButton().getRise()) {
            pidD -= .01;
            pidChanged = true;
        } else if (buttons.getRightStickButton().getRise()) {
            pidD += .01;
            pidChanged = true;
        }

        if (pidChanged) {
            setupPidController();
        }

        telemetry.addData("01", "PID %f/%f/%f", pidP, pidI, pidD);
        telemetry.addData("03", "power level " + powerLevel);
        telemetry.addData("04", "inches to drive " + inchesToDrive);
    }
}
