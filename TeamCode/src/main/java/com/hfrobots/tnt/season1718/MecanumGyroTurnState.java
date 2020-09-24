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
import com.hfrobots.tnt.corelib.drive.Turn;
import com.hfrobots.tnt.corelib.state.State;
import com.hfrobots.tnt.corelib.state.TimeoutSafetyState;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

public class MecanumGyroTurnState extends TimeoutSafetyState
{
    private static final double HEADING_THRESHOLD = 1 ; // As tight as we can make it with an integer gyro
    private static final double DEFAULT_MAX_POWER = 0.3D;
    private static final double DEFAULT_MIN_POWER = 0.02D;

    private double pTurnCoeff = 0.02;     // Larger is more responsive, but also less stable
    private double pLargeTurnCoeff = pTurnCoeff;
    private double pSmallTurnCoeff = 0.04;
    private double smallTurnThresholdDegrees;
    private double maxPower = 0.3;

    private boolean useEncoders = true;

    private boolean useBraking = true;

    private Turn turn;

    private double minPower = 0.02;

    private boolean initialized = false;

    private boolean reachedTarget = false;

    private float targetHeading;

    private MecanumDrive mecanumDrive;

    private LynxEmbeddedIMU imu;

    private long lastCycleTimestamp = 0;

    public static Builder builder() {
        return new Builder();
    }

    public static class Builder {
        private String name;
        private Telemetry telemetry;
        private MecanumDrive mecanumDrive;
        private LynxEmbeddedIMU imu;
        private Turn turn;
        private double maxPower = DEFAULT_MAX_POWER; // sane defaults
        private double minPower = DEFAULT_MIN_POWER; // sane defaults
        private double pLargeTurnCoeff;
        private double smallTurnThresholdDegrees;
        private double pSmallTurnCoeff;
        private long safetyTimeoutMillis;

        private Builder() {}

        public Builder setName(String name) {
            this.name = name;
            return this;
        }

        public Builder setTelemetry(Telemetry telemetry) {
            this.telemetry = telemetry;
            return this;
        }

        public Builder setMecanumDrive(MecanumDrive mecanumDrive) {
            this.mecanumDrive = mecanumDrive;
            return this;
        }

        public Builder setImu(LynxEmbeddedIMU imu) {
            this.imu = imu;
            return this;
        }

        public Builder setTurn(Turn turn) {
            this.turn = turn;
            return this;
        }

        public Builder setPLargeTurnCoeff(double pLargeTurnCoeff) {
            this.pLargeTurnCoeff = pLargeTurnCoeff;
            return this;
        }

        public Builder setSmallTurnThresholdDegrees(double smallTurnThresholdDegrees) {
            this.smallTurnThresholdDegrees = smallTurnThresholdDegrees;
            return this;
        }

        public Builder setPSmallTurnCoeff(double pSmallTurnCoeff) {
            this.pSmallTurnCoeff = pSmallTurnCoeff;
            return this;
        }

        public Builder setSafetyTimeoutMillis(long safetyTimeoutMillis) {
            this.safetyTimeoutMillis = safetyTimeoutMillis;
            return this;
        }

        public Builder setMaxPower(double maxPower) {
            this.maxPower = maxPower;
            return this;
        }

        public Builder setMinPower(double minPower) {
            this.minPower = minPower;
            return this;
        }

        public MecanumGyroTurnState build() {
            // TODO: Check that all required arguments are set
            MecanumGyroTurnState mgts = new MecanumGyroTurnState(name, telemetry, safetyTimeoutMillis);
            mgts.mecanumDrive = mecanumDrive;
            mgts.imu = imu;
            mgts.turn = turn;
            mgts.maxPower = maxPower;
            mgts.minPower = minPower;
            mgts.pLargeTurnCoeff = pLargeTurnCoeff;
            mgts.smallTurnThresholdDegrees = smallTurnThresholdDegrees;
            mgts.pSmallTurnCoeff = pSmallTurnCoeff;

            return mgts;
        }
    }

    private MecanumGyroTurnState(String name, Telemetry telemetry, long safetyTimeoutMillis) {
        super(name, telemetry, safetyTimeoutMillis);
    }

    public State doStuffAndGetNextState() {
        if (!initialized) {
            initialize();
        }

        if (isTimedOut()) {
            Log.d(LOG_TAG, "Timed out a.k.a we done goofed ");

            mecanumDrive.driveCartesian(0, 0, 0, false, 0.0);

            mecanumDrive.stopAllDriveMotors();

            return nextState;
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

                return nextState;
            }
        }

        return this;
    }

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

        Log.d(LOG_TAG, "Error: " + error);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            onTarget = true;
        } else {
            steer = Range.clip(error * pTurnCoeff, -maxPower, maxPower);
        }

        if (Math.abs (steer) <= minPower) {
            if (steer >= 0) {
                steer = minPower;
            } else {
                steer = -minPower;
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

    private void initialize() {
        // not-yet initialized
        // First angle is heading, second is roll, third is pitch
        reachedTarget = false;

        float currentHeading = imu.getAngularOrientation().firstAngle;
        float turnHeading = turn.getHeading();
        targetHeading = currentHeading + turnHeading;

        if (turnHeading < smallTurnThresholdDegrees) {
            pTurnCoeff = pSmallTurnCoeff;
        } else {
            pTurnCoeff = pLargeTurnCoeff;
        }

        configureMotorParameters();

        Log.d(LOG_TAG, "Gyro turn initialized - current heading: " + currentHeading + ", relative turn heading: " + turnHeading + ", target heading: " + targetHeading);
        initialized = true;

        configureMotorParameters();

        //Log.d(LOG_TAG, "Gyro turn initialized - current heading: " + currentHeading + ", relative turn heading: " + turnHeading + ", target heading: " + targetHeading);
        initialized = true;
    }

    @Override
    public void resetToStart() {
        super.resetToStart();
        initialize();
    }

    @Override
    public void liveConfigure(DebouncedGamepadButtons buttons) {

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
