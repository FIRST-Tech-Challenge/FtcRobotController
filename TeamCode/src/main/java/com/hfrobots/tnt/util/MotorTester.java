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

package com.hfrobots.tnt.util;

import com.ftc9929.corelib.control.DebouncedButton;
import com.ftc9929.corelib.control.NinjaGamePad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * An OpMode that allows you to test any/all of the motors on a robot
 *
 * left stick (y) - set motor power (when not running to position)
 * right stick (y) - increment/decrement relative target position
 * left bumper - toggle float/brake power mode
 * right bumper - switch motors
 * a button - run to desired relative position using encoders
 */
@TeleOp(name="Motor Tester", group="Utilities")
public class MotorTester extends OpMode {
    private List<NamedDeviceMap.NamedDevice<DcMotor>> namedMotors;
    private Map<DcMotor, String> motorsToNames = new HashMap<>();
    private int currentListPosition;
    private int desiredPosition;
    private boolean runningToPosition;
    private int targetPosition;
    private boolean runningTimedTest;
    private long timedTestStartMs;

    private DebouncedButton aButton;

    private DebouncedButton bButton;

    private DebouncedButton rightBumper;

    private DebouncedButton dpadUp;

    private DebouncedButton dpadDown;

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
    }

    @Override
    public void loop() {
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

        desiredPosition += (int)(-gamepad1.right_stick_y);

        NamedDeviceMap.NamedDevice<DcMotor> namedDcMotor = namedMotors.get(currentListPosition);
        DcMotor currentMotor = namedDcMotor.getDevice();
        String motorName = namedDcMotor.getName();

        if (runningTimedTest) {
            if ((System.currentTimeMillis() - timedTestStartMs) >= 5000) {
                currentMotor.setPower(0D);
                runningTimedTest = false;

                updateTelemetry(currentMotor, motorName, 0, DcMotor.ZeroPowerBehavior.BRAKE);

                return;
            }

            return;
        }

        if (bButton.getRise()) {
            runningTimedTest = true;
            timedTestStartMs = System.currentTimeMillis();
            currentMotor.setPower(0.75D);
            updateTelemetry(currentMotor, motorName, 0, DcMotor.ZeroPowerBehavior.BRAKE);

            return;
        }

        if (runningToPosition) {
            if (Math.abs(
                    currentMotor.getCurrentPosition() - targetPosition) >= 10) {
                updateTelemetry(currentMotor, motorName, 0, DcMotor.ZeroPowerBehavior.BRAKE);

                return;
            }

            runningToPosition = false;
        }

        float leftStickYPosition = -gamepad1.left_stick_y;

        if (aButton.getRise()) {
            targetPosition = currentMotor.getCurrentPosition() + desiredPosition;
            currentMotor.setPower(1.0);
            currentMotor.setTargetPosition(targetPosition);
            runningToPosition = true;
        } else {
            currentMotor.setPower(leftStickYPosition);
        }

        DcMotor.ZeroPowerBehavior powerBehavior;

        if (gamepad1.left_bumper) {
            powerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
        } else {
            powerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
        }

        currentMotor.setZeroPowerBehavior(powerBehavior);

        updateTelemetry(currentMotor, motorName, leftStickYPosition, powerBehavior);
    }

    protected void updateTelemetry(DcMotor currentMotor, String motorName, float leftStickYPosition, DcMotor.ZeroPowerBehavior powerBehavior) {
        int currentPosition = currentMotor.getCurrentPosition();

        telemetry.addData("motor ",  "%s - %s - pow %s - despos %s - curpos %s", motorName, powerBehavior,
                Double.toString(leftStickYPosition),
                Integer.toString(desiredPosition),
                Integer.toString(currentPosition));
        updateTelemetry(telemetry);
    }
}
