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

import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.control.RangeInput;
import com.google.common.base.Stopwatch;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Launcher Tester", group="Utilities")
public class LauncherTester extends OpMode {

    private int lastFrontEncoderPos = Integer.MIN_VALUE;

    private int lastRearEncoderPos = Integer.MIN_VALUE;

    private double velocity;

    private DcMotorEx frontLauncherMotor;

    private DcMotorEx rearLauncherMotor;

    private Stopwatch stopWatch;

    private RangeInput leftStickY;

    private double frontEncoderTicksPerSecond = 0;
    private double rearEncoderTicksPerSecond = 0;

    @Override
    public void init() {

        frontLauncherMotor = hardwareMap.get(DcMotorEx.class, "frontLauncherMotor");
        rearLauncherMotor = hardwareMap.get(DcMotorEx.class, "rearLauncherMotor");
        leftStickY = new NinjaGamePad(gamepad1).getLeftStickY();
    }


    @Override
    public void loop() {
        if (stopWatch == null) {
            stopWatch = Stopwatch.createStarted();
        }

        long elapsedTime = stopWatch.elapsed(TimeUnit.MILLISECONDS);

        if (elapsedTime >= 100) {
            stopWatch.reset();
            stopWatch.start();

            int frontLauncherMotorCurrentPosition = frontLauncherMotor.getCurrentPosition();
            int rearLauncherMotorCurrentPosition = rearLauncherMotor.getCurrentPosition();

            if (lastFrontEncoderPos != Integer.MIN_VALUE) {
                int frontEncoderTicks = frontLauncherMotorCurrentPosition - lastFrontEncoderPos;

                int rearEncoderTicks = rearLauncherMotorCurrentPosition - lastRearEncoderPos;

                double frontEncoderTicksPerMs = (double) frontEncoderTicks / (double) elapsedTime;
                frontEncoderTicksPerSecond = frontEncoderTicksPerMs * 1000;


                double rearEncoderTicksPerMs = (double) rearEncoderTicks / (double) elapsedTime;
                rearEncoderTicksPerSecond = rearEncoderTicksPerMs * 1000;
            }

            lastFrontEncoderPos = frontLauncherMotorCurrentPosition;
            lastRearEncoderPos = rearLauncherMotorCurrentPosition;
        }

        float stickPos = leftStickY.getPosition();

        velocity += -stickPos * 4.0;

        frontLauncherMotor.setVelocity(velocity);
        rearLauncherMotor.setVelocity(velocity);

        telemetry.addData("v ",  "v %.1f vm_f: %.1f vm_r: %.1f", velocity, frontEncoderTicksPerSecond, rearEncoderTicksPerSecond);
    }
}
