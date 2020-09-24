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

import com.hfrobots.tnt.corelib.control.DebouncedButton;
import com.hfrobots.tnt.corelib.control.NinjaGamePad;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/**
 * An OpMode that allows you to test two continuous servos that are ganged together
 *
 * gamepad1.left_stick.y = "power" level sent to the CR servo
 *
 * Requires a hardware map with 2 continuous rotation servos defined, named 'servo1' and
 * 'servo2'.
 */
@TeleOp(name="Dual Servo Tester", group="Utilities")
@Disabled
@SuppressWarnings("unused")
public class DualServoTester extends OpMode {
    private DebouncedButton stopButton;

    private CRServo servo1;

    private CRServo servo2;

    @Override
    public void init() {

        NinjaGamePad ninjaGamePad = new NinjaGamePad(gamepad1);
        stopButton = new DebouncedButton(ninjaGamePad.getBButton());
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");
        servo1.setDirection(DcMotorSimple.Direction.FORWARD);
        servo2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        boolean canRun = true;

        if (servo1 == null) {
            telemetry.addData("s1", "no 'servo1' in hw map");

            canRun = false;
        }

        if (servo2 == null) {
            telemetry.addData("s2", "no 'servo2' in hw map");

            canRun = false;
        }

        if (!canRun) {
            updateTelemetry(telemetry);

            return;
        }

        float leftStickYPosition = -gamepad1.left_stick_y;

        if (stopButton.getRise()) {
            servo1.setPower(0);
            servo2.setPower(0);
        } else {
            servo1.setPower(leftStickYPosition);
            servo2.setPower(leftStickYPosition);
        }
    }
}
