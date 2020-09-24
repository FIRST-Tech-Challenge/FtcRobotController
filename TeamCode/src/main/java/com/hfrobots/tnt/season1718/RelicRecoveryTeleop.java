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

import com.hfrobots.tnt.corelib.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

/**
 * Provide a basic manual operational mode that controls the tank drive.
 */
@TeleOp(name="00 RR Teleop")
@Disabled
public class RelicRecoveryTeleop extends RelicRecoveryTelemetry

{

    @SuppressWarnings("unused")
    public RelicRecoveryTeleop() {
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
    }


    /**
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override public void loop ()

    {
        handleDrivingInputs();
        handleGlyphGripper();

        //
        // Send telemetry data to the driver station.
        //
        updateTelemetry(); // Update common telemetry
        updateGamepadTelemetry();

        // Keep jewel mechanism stowed
        redAllianceJewelMech.stowSensor();
        blueAllianceJewelMech.stowSensor();

        if (markLogButton.getRise()){
            // log Glyph servos, limit switchs, jewel servos, battery
            Log.d(LOG_TAG, " -- MARK --");
            if (naturalTopGlyphServo != null) {
                Log.d(LOG_TAG, "Natural Top Glyph Servo value : " + naturalTopGlyphServo.getPosition());
            }

            if (naturalBottomGlyphServo != null) {
                Log.d(LOG_TAG, "Natural Bottom Glyph Servo value :" + naturalBottomGlyphServo.getPosition());
            }

            if (glyphRotateServo != null) {
                Log.d(LOG_TAG, "Glyph Rotate Servo value :" + glyphRotateServo.getPosition());
            }

            if (invertedGlyphLimit != null) {
                Log.d(LOG_TAG, "Inverted glyph limit : " + invertedGlyphLimit.getState());
            }

            if (uprightGlyphLimit != null) {
                Log.d(LOG_TAG, "Upright Glyph limit : " + uprightGlyphLimit.getState());
            }

            if (glyphLiftBottomLimit != null) {
                Log.d(LOG_TAG, "Lift Bottom Limit :" + glyphLiftBottomLimit.getState());
            }

            if (glyphLiftTopLimit != null) {
                Log.d(LOG_TAG, "Lift Top Limit" + glyphLiftTopLimit.getState());
            }

            logBatteryState("-- requested by log mark --");
        }
    }

    @Override
    public void stop() {
        Log.d(LOG_TAG, "x-throttle-usage: " + xThrottleHistogram.toString());
        Log.d(LOG_TAG, "y-throttle-usage: " + yThrottleHistogram.toString());
        Log.d(LOG_TAG, "rot-throttle-usage: " + rotateThrottleHistogram.toString());
        super.stop();
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

