/*
 Copyright (c) 2020 HF Robotics (http://www.hfrobots.com)
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
 */

package com.hfrobots.tnt.season2021;

import com.google.common.base.Stopwatch;
import com.google.common.base.Ticker;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

import lombok.NonNull;

public class Launcher {

    // GoBilda 1:1 positive power rotation CCW, max theoretical encoder/sec is 2800, measured is 2700

    public final static int LAUNCH_SPEED_ENC_SEC = 1600;

    private final static int LAUNCH_SPEED_TOLERANCE_ENC_SEC = 300;

    private final static int IDLE_SPEED_ENC_SEC = 500;
    private static final double LIFT_ADJUSTMENT_VALUE = 0.1;

    private final DcMotorEx frontLauncherMotor;

    private final DcMotorEx rearLauncherMotor;

    // Taylor said in Slack that center is "up", right is "down"

    private final Servo hopperPullDownServo;

    public final static double HOPPER_PULLED_DOWN_POSITION = 1;

    public final static double HOPPER_FLOAT_POSITION = 0.5;

    private final Servo ringFeederServo;

    private final Servo launcherLiftServo;

    public final static double RING_FEEDER_FEEDING_POSITION = 0.5;

    public final static double RING_FEEDER_PARKED_POSITION = 1.0;

    public final static double LAUNCHER_LIFT_STOWED_POSITION = 0.5;

    public final static double LAUNCHER_LIFT_HIGH_POSITION = 1.0;

    private final VelocityTracker frontVelocityTracker;

    private final VelocityTracker rearVelocityTracker;

    private final Telemetry telemetry;

    private final Ticker ticker;

    public Launcher(@NonNull HardwareMap hardwareMap, Telemetry telemetry, Ticker ticker) {
        this.telemetry = telemetry;
        this.ticker = ticker;

        frontLauncherMotor = hardwareMap.get(DcMotorEx.class, "frontLauncherMotor");
        rearLauncherMotor = hardwareMap.get(DcMotorEx.class, "rearLauncherMotor");

        frontLauncherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLauncherMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        ringFeederServo = hardwareMap.get(Servo.class, "ringFeedServo");

        hopperPullDownServo = hardwareMap.get(Servo.class, "hopperPullDownServo");
        launcherLiftServo = hardwareMap.get(Servo.class, "launcherLiftServo");

        frontVelocityTracker = new VelocityTracker();
        rearVelocityTracker = new VelocityTracker();
    }

    public void launcherToStowedPosition() {
        launcherLiftServo.setPosition(LAUNCHER_LIFT_STOWED_POSITION);
    }

    public void launcherToHighPosition() {
        launcherLiftServo.setPosition(LAUNCHER_LIFT_HIGH_POSITION);
    }

    public void raiseLauncher() {
        adjustLauncher(1);
    }

    public void lowerLauncher() {
        adjustLauncher(-1);
    }

    private void adjustLauncher(int magnitude) {
        double currentPosition = launcherLiftServo.getPosition();

        if (magnitude > 1 && currentPosition + LIFT_ADJUSTMENT_VALUE > 1.0) {
            return;
        }

        if (magnitude < 1 && currentPosition - LIFT_ADJUSTMENT_VALUE < LAUNCHER_LIFT_STOWED_POSITION) {
            return;
        }

        launcherLiftServo.setPosition(currentPosition + (double)magnitude * LIFT_ADJUSTMENT_VALUE);

        telemetry.addData("Lch", "lift: %.2f", launcherLiftServo.getPosition());
    }

    public void pulldownHopper() {
        hopperPullDownServo.setPosition(HOPPER_PULLED_DOWN_POSITION);
    }

    public void floatHopperWithLauncher() {
        hopperPullDownServo.setPosition(HOPPER_FLOAT_POSITION);
    }

    public void feedRing() {
        ringFeederServo.setPosition(RING_FEEDER_FEEDING_POSITION);
    }

    public void parkRingFeeder() {
        ringFeederServo.setPosition(RING_FEEDER_PARKED_POSITION);
    }

    public void launcherToIdleSpeed() {
        frontLauncherMotor.setVelocity(IDLE_SPEED_ENC_SEC);
        rearLauncherMotor.setVelocity(IDLE_SPEED_ENC_SEC);
        writeWheelSpeedsToTelemetry();
    }

    public void launcherToFullSpeed() {
        frontLauncherMotor.setVelocity(LAUNCH_SPEED_ENC_SEC);
        rearLauncherMotor.setVelocity(LAUNCH_SPEED_ENC_SEC);
        writeWheelSpeedsToTelemetry();
    }

    public void writeWheelSpeedsToTelemetry() {
        double frontEncoderTicksPerSecond = frontVelocityTracker.getTicksPerSec(frontLauncherMotor.getCurrentPosition());
        double rearEncoderTicksPerSecond = rearVelocityTracker.getTicksPerSec(rearLauncherMotor.getCurrentPosition());

        telemetry.addData("lv",  "vf: %.1f vr: %.1f", frontEncoderTicksPerSecond, rearEncoderTicksPerSecond);
    }

    public boolean isLauncherAtFullSpeed() {
        double frontEncoderTicksPerSecond = frontVelocityTracker.getTicksPerSec(frontLauncherMotor.getCurrentPosition());
        double rearEncoderTicksPerSecond = rearVelocityTracker.getTicksPerSec(rearLauncherMotor.getCurrentPosition());

        telemetry.addData("lv",  "vf: %.1f vr: %.1f", frontEncoderTicksPerSecond, rearEncoderTicksPerSecond);

        double frontDeltaEncSec = Math.abs(frontEncoderTicksPerSecond - LAUNCH_SPEED_ENC_SEC);
        double rearDeltaEncSec = Math.abs(rearEncoderTicksPerSecond - LAUNCH_SPEED_ENC_SEC);

        // FIXME - Later, warn if tolerance is too low, or too high so intelligent decision can be made!

        return frontDeltaEncSec < LAUNCH_SPEED_TOLERANCE_ENC_SEC && rearDeltaEncSec < LAUNCH_SPEED_TOLERANCE_ENC_SEC;
    }

    private class VelocityTracker {
        private int lastEncoderPos = Integer.MIN_VALUE;

        private double encoderTicksPerSecond = 0;

        private Stopwatch stopWatch;

        public double getTicksPerSec(int currentEncoderPosition) {
            if (stopWatch == null) {
                stopWatch = Stopwatch.createStarted(ticker);
            }

            long elapsedTime = stopWatch.elapsed(TimeUnit.MILLISECONDS);

            if (elapsedTime >= 100) {
                stopWatch.reset();
                stopWatch.start();

                if (lastEncoderPos != Integer.MIN_VALUE) {
                    int encoderTicksDelta = currentEncoderPosition - lastEncoderPos;

                    double encoderTicksPerMs = (double) encoderTicksDelta / (double) elapsedTime;
                    encoderTicksPerSecond = encoderTicksPerMs * 1000;
                }

                lastEncoderPos = currentEncoderPosition;
            }

            return encoderTicksPerSecond;
        }
    }
}


