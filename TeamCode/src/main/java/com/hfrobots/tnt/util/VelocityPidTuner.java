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

package com.hfrobots.tnt.util;

import android.util.Log;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.google.common.base.Stopwatch;
import com.hfrobots.tnt.corelib.drive.PidController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

import static com.ftc9929.corelib.Constants.LOG_TAG;

@TeleOp(name="Velocity PID", group="Utilities")
public class VelocityPidTuner extends OpMode {
    private List<NamedDeviceMap.NamedDevice<DcMotor>> namedMotors;
    private Map<DcMotor, String> motorsToNames = new HashMap<>();
    private int currentListPosition;

    private DebouncedButton aButton;

    private DebouncedButton bButton;

    private DebouncedButton rightBumper;

    private DebouncedButton dpadUp;

    private DebouncedButton dpadDown;

    private boolean runningPid = false;

    private int lastEncoderPos = Integer.MIN_VALUE;

    private Stopwatch stopWatch;

    private PidController pidController;

    private static final double TARGET_RPM = 200;

    private static final double TARGET_TICKS_PER_RPM = 537.6 * TARGET_RPM;

    private static final double TARGET_TICKS_PER_SECOND = TARGET_TICKS_PER_RPM / 60;

    @Override
    public void init() {
        NamedDeviceMap namedDeviceMap = new NamedDeviceMap(hardwareMap);
        namedMotors = namedDeviceMap.getAll(DcMotor.class);
        currentListPosition = 0;

        NinjaGamePad ninjaGamePad = new NinjaGamePad(gamepad1);
        aButton = new DebouncedButton(ninjaGamePad.getAButton());
        bButton = new DebouncedButton(ninjaGamePad.getBButton());
        rightBumper = new DebouncedButton(ninjaGamePad.getRightBumper());
        dpadUp = new DebouncedButton(ninjaGamePad.getDpadUp());
        dpadDown = new DebouncedButton(ninjaGamePad.getDpadDown());
        double ku = .005;

        pidController = PidController.builder().setAllowOscillation(true).setKp(.005).setkI(0).setkF(3.24e-4).setTolerance(10).build();
    }

    @Override
    public void loop() {
        if (stopWatch == null) {
            stopWatch = Stopwatch.createStarted();
        }

        if (namedMotors.isEmpty()) {
            telemetry.addData("No DC Motors", "");
            updateTelemetry(telemetry);
            return;
        }

        if (rightBumper.getRise()) {
            currentListPosition++;

            if (currentListPosition == namedMotors.size()) {
                currentListPosition = 0;
            }
        }

        NamedDeviceMap.NamedDevice<DcMotor> namedDcMotor = namedMotors.get(currentListPosition);
        DcMotor currentMotor = namedDcMotor.getDevice();
        String motorName = namedDcMotor.getName();

        if (!runningPid) {
            telemetry.addData("motor ", "%s", motorName);
        }

        updateTelemetry(telemetry);

        if (bButton.getRise()) {
            runningPid = true;

            return;
        }

        if (runningPid) {
            if (lastEncoderPos == Integer.MIN_VALUE) {
                pidController.setTarget(2100, 0);
                pidController.setAbsoluteSetPoint(true);
                pidController.setOutputRange(0.05, 1.0);

                lastEncoderPos = currentMotor.getCurrentPosition();

                return;
            }

            long elapsedTime = stopWatch.elapsed(TimeUnit.MILLISECONDS);
            double encoderTicksPerSecond = 0;

            if (elapsedTime >= 100) {
                stopWatch.reset();
                stopWatch.start();

                // We have enough samples to run the PID

                int encoderTicks = currentMotor.getCurrentPosition() - lastEncoderPos;

                double encoderTicksPerMs = (double) encoderTicks / (double) elapsedTime;
                encoderTicksPerSecond = (double) encoderTicksPerMs * 1000;

                Log.d(LOG_TAG, String.format("ticks: %d, pms: %f, ps: %f", encoderTicks, encoderTicksPerMs, encoderTicksPerSecond));

                lastEncoderPos = currentMotor.getCurrentPosition();

                if (!pidController.isOnTarget()) {
                    double output = pidController.getOutput(encoderTicksPerSecond);

                    //if (output >= 0.7) {
                    //   output = 0.7;
                    //}

                    currentMotor.setPower(output);
                }
            }

            telemetry.addData("motor ",  "%s p: %f v: %f", motorName, currentMotor.getPower(), encoderTicksPerSecond);
        }
    }
}
