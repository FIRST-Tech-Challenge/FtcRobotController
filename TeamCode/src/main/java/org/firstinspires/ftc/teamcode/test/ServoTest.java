/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */


package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.GreenBotBaseOpMode;

@TeleOp(name = "Servo Test", group = "Test")
public class ServoTest extends GreenBotBaseOpMode {

    private static final double DRIVE_POWER = 0.2;

    @Override
    public void runOpMode() {
        config();

        wedgeServo.setPosition(0.8);

        waitForStart();

        while (opModeIsActive()) {

            double pos = wedgeServo.getPosition();
            if (gamepad1.dpadUpWasPressed()) {
                wedgeServo.setPosition(pos + 0.1);
            }
            if (gamepad1.dpadDownWasPressed()) {
                wedgeServo.setPosition(pos - 0.1);
            }

            if (gamepad1.yWasPressed()) {
                wedgeServo.setPosition(0.8);
            }
            if (gamepad1.bWasPressed()) {
                wedgeServo.setPosition(0.65);
            }
            if (gamepad1.aWasPressed()) {
                wedgeServo.setPosition(0.35);
            }
            telemetry.addData("D Pad Up", "frontRight");
            telemetry.addData("D Pad Down", "backLeft");
            telemetry.addData("Servo pos", wedgeServo.getPosition());
            telemetry.update();
        }
    }
}
