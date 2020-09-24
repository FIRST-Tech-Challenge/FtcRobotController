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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

/**
 * Provide a basic manual operational mode that controls the tank drive.
 */
@TeleOp(name="RoverRuckus Diag")
@Disabled
public class RoverRuckusDiagnostics extends RoverRuckusTelemetry

{

    private static final double COLLECTOR_DEPLOY_IN_POWER = -.25;
    private static final double COLLECTOR_DEPLOY_OUT_POWER = .25;

    @SuppressWarnings("unused")
    public RoverRuckusDiagnostics() {
        imuNeeded = false;
    }

    /**
     * Places the robot into the initial pre-running state.
     * Robot must fit within sizing guidelines (18x18)
     * before this state completes.
     */
    @Override
    public void init() {

        super.init();

        for (DcMotor motor : mecanumDrive.motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Reset RUN_TO_POSITION from autonomous
        acDcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        teamMarkerServo.setPosition(TEAM_MARKER_DUMP_POS);
        updateGamepadTelemetry();
    }

    @Override
    public void init_loop() {
        updateGamepadTelemetry();
    }
    /**
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override public void loop ()

    {
        teamMarkerServo.setPosition(TEAM_MARKER_DUMP_POS);

        //handleDrivingInputs();
        //handleAscender();
        //handleCollector();
        //handleParticleScoringMechanism();


        //
        // Send telemetry data to the driver station.
        //
        updateTelemetry(); // Update common telemetry

        if (driverRightBumper.getRise()) {
            stowM3();
        }

        if (driverLeftBumper.getRise()) {
            deployM3();
        }
    }

    private void keepM3Stowed() {
        m3ArmUp();
        m3FlagUp();
    }

    private void stowM3() {
        m3FlagUp();

        try {
            Thread.currentThread().sleep(500);
        } catch (InterruptedException intEx) {
            // do nothing
        }

        m3ArmUp();
    }

    private void deployM3() {
        m3FlagOut();

        try {
            Thread.currentThread().sleep(500);
        } catch (InterruptedException intEx) {
            // do nothing
        }

        m3ArmOut();
    }

    @Override
    public void stop() {
        Log.d(LOG_TAG, "x-throttle-usage: " + xThrottleHistogram.toString());
        Log.d(LOG_TAG, "y-throttle-usage: " + yThrottleHistogram.toString());
        Log.d(LOG_TAG, "rot-throttle-usage: " + rotateThrottleHistogram.toString());
        super.stop();
    }

    private void handleParticleScoringMechanism() {
        particleScoringMechanism.doPeriodicTask();
    }

    double collectorGateOpenPosition = 1;

    double collectorGateClosedPosition = 0;

    private boolean isSafeToLoadAllTheThings(){
        boolean collectorInPos = !collectorStowLimitSwitch.getState(); // returns false for active
        boolean scoreBoxInPos = !scoreBoxReadyToLoadLimitSwitch.getState(); // returns false for active

        if (collectorInPos & scoreBoxInPos) {
            return true;
        }

        return false;
    }

    private void handleCollector() {
        // WARNING WARNING WARNING WARNING
        //
        // Order here is important - operator can't override the loading behavior if it comes
        // after the processing of commands to manually operate the collector...Any refactoring
        // of this code needs to maintain the override functionality - we have needed it during
        // matches! (we accidentally created this order - we found out we want it)

        if (loadToBoxButton.isPressed()) {
            // Note, only allow loading if elevator is at low limit position (unless overridden)
            if (limitOverrideButton.isPressed() || particleScoringMechanism.isAtLowerLimit()){
                collectorGateServo.setPosition(collectorGateOpenPosition);
                collectorSweepMotor.setPower(-.5);

                collectorDeployMotor.setPower(COLLECTOR_DEPLOY_IN_POWER / 2);
            }
        } else {

            collectorGateServo.setPosition(collectorGateClosedPosition);

            // TODO: File issues/tasks to make it less error-prone for 3rd league meet

            // We will be using left stick, right stick, y axis on operator controller
            // Remember, y axis, -1 is "forward"

            double DEADBAND = 0.5; // How far do we need to push the stick to make it go?

            double collectorDeployPosition = collectorDeployInput.getPosition();

            double collectorSweepPosition = collectorSweepInput.getPosition();

            if (collectorDeployPosition > DEADBAND) {
                collectorDeployMotor.setPower(COLLECTOR_DEPLOY_IN_POWER); // in
            } else if (collectorDeployPosition < -DEADBAND) {
                collectorDeployMotor.setPower(COLLECTOR_DEPLOY_OUT_POWER); // out
            } else {
                collectorDeployMotor.setPower(0);
            }

            if (collectorSweepPosition < -DEADBAND) {
                collectorSweepMotor.setPower(1.0); // out
            } else if (collectorSweepPosition > DEADBAND) {
                collectorSweepMotor.setPower(-1.0); // in
                collectorDeployMotor.setPower(COLLECTOR_DEPLOY_OUT_POWER / 2);
            } else {
                collectorSweepMotor.setPower(0);
            }
        }
    }

    private void handleDrivingInputs() {
        double x = - driveStrafe.getPosition(); // positive robot x axis is negative joystick axis
        double y = - driveForwardReverse.getPosition();
        double rot = - driveRotate.getPosition(); // positive robot z rotation (human-normal) is negative joystick x axis

        y = -y; // still need to figure this one out!

        // do this first, it will be cancelled out by bump-strafe
        if (!driveFastButton.isPressed()) {
            y /= 1.5;
            x /= 1.25;
            rot /= 1.5;
        }

        final boolean driveInverted;

        if (driveInvertedButton.isPressed()) {
            driveInverted = true;
        } else {
            driveInverted = false;
        }

        double xScaled = x;
        double yScaled = y;
        double rotateScaled = rot;

        // we check both bumpers - because both being pressed is driver 'panic', and we
        // don't want unexpected behavior!
        if (driveBumpStrafeLeftButton.isPressed() && !driveBumpStrafeRightButton.isPressed()) {
            xScaled = .6;
            yScaled = 0;
            rotateScaled = 0;
        } else if (driveBumpStrafeRightButton.isPressed() && !driveBumpStrafeLeftButton.isPressed()) {
            xScaled = -.6;
            yScaled = 0;
            rotateScaled = 0;
        }

        xThrottleHistogram.accumulate(xScaled);
        yThrottleHistogram.accumulate(yScaled);
        rotateThrottleHistogram.accumulate(rotateScaled);

        telemetry.addData("pow", "y %.3f, x %.3f, r %.3f", yScaled, xScaled, rotateScaled);

        mecanumDrive.driveCartesian(xScaled, yScaled, rotateScaled, driveInverted, 0.0);
    }

    private Histogram xThrottleHistogram = new Histogram();
    private Histogram yThrottleHistogram = new Histogram();
    private Histogram rotateThrottleHistogram = new Histogram();

    class Histogram {
        int[] buckets = new int[20];

        void accumulate(double value) {
            int bucketIndex = Range.clip((int)(Math.abs(value) * buckets.length), 0, buckets.length - 1);
            buckets[bucketIndex]++;
        }

        @Override
        public String toString() {
            StringBuffer buf = new StringBuffer();
            for (int i = 0; i < buckets.length; i++) {
                if (i != 0) {
                    buf.append(", ");
                }
                buf.append(buckets[i]);
            }

            return buf.toString();
        }
    }

}

