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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/**
 * An OpMode that allows you to test any/all of the servos on a robot
 *
 * gamepad1.left_stick.y = proportional adjustment
 * gamepad1.dpad up/down = micro adjustment
 * gamepad1.right_bumper = cycle through all configured servos
 *
 * current servo name and position are displayed in driver station telemetry
 *
 */
@TeleOp(name="Servo Tester", group="Utilities")
@SuppressWarnings("unused")
public class ServoTester extends OpMode {
    private List<NamedDeviceMap.NamedDevice<Servo>> namedServos;

    private int currentListPosition;

    private DebouncedButton rightBumper;

    private DebouncedButton dpadUp;

    private DebouncedButton dpadDown;

    @Override
    public void init() {
        NamedDeviceMap namedDeviceMap = new NamedDeviceMap(hardwareMap);

        namedServos = namedDeviceMap.getAll(Servo.class);

        currentListPosition = 0;

        for (NamedDeviceMap.NamedDevice<Servo> namedServo : namedServos) {
            namedServo.getDevice().setPosition(0);
        }

        NinjaGamePad ninjaGamePad = new NinjaGamePad(gamepad1);
        rightBumper = new DebouncedButton(ninjaGamePad.getRightBumper());
        dpadUp = new DebouncedButton(ninjaGamePad.getDpadUp());
        dpadDown = new DebouncedButton(ninjaGamePad.getDpadDown());
    }

    @Override
    public void loop() {
        if (namedServos.isEmpty()) {
            telemetry.addData("No servos", "");
            updateTelemetry(telemetry);
            return;
        }

        if (rightBumper.getRise()) {
            currentListPosition++;

            if (currentListPosition == namedServos.size()) {
                currentListPosition = 0;
            }

        }

        NamedDeviceMap.NamedDevice<Servo> currentNamedServo = namedServos.get(currentListPosition);
        Servo currentServo = currentNamedServo.getDevice();
        String servoName = currentNamedServo.getName();

        double currentPosition = currentServo.getPosition();

        float leftStickYPosition = -gamepad1.left_stick_y * .01f;
        double microAdjust;

        if (dpadUp.getRise()) {
            microAdjust = 0.001;
        } else if (dpadDown.getRise()) {
            microAdjust = -0.001;
        } else {
            microAdjust = 0;
        }

        double newPosition = currentPosition + leftStickYPosition + microAdjust;
        newPosition = Range.clip(newPosition, 0, 1.0);
        telemetry.addData("servo ",  "%s : %s", servoName, Double.toString(newPosition));
        updateTelemetry(telemetry);

        currentServo.setPosition(newPosition);
    }
}
