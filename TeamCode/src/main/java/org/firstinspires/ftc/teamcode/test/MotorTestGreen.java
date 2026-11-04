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

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.base.GreenBotBaseOpMode;

@TeleOp(name = "Motor Tester - Green Bot", group = "StarterBot")
public class MotorTestGreen extends GreenBotBaseOpMode {

    private static final double DRIVE_POWER = 0.2;

    @Override
    public void runOpMode() {

        config();
        waitForStart();

        while (opModeIsActive()) {
            frontRight.setPower(gamepad1.dpad_up ? DRIVE_POWER : 0.0);
            backRight.setPower(gamepad1.dpad_right ? DRIVE_POWER : 0.0);
            frontLeft.setPower(gamepad1.dpad_left ? DRIVE_POWER : 0.0);
            backLeft.setPower(gamepad1.dpad_down ? DRIVE_POWER : 0.0);

            turret.setPower(gamepad1.y ? DRIVE_POWER : 0.0);
            launcher.setPower(gamepad1.b ? DRIVE_POWER : 0.0);
            intake.setPower(gamepad1.a ? DRIVE_POWER : 0.0);
            feeder.setPower(gamepad1.x ? DRIVE_POWER * 2 : 0.0);

            rightIntakeServo.setPower(gamepad1.right_bumper ? 1.0 : 0.0);
            leftIntakeServo.setPower(gamepad1.left_bumper ? 1.0 : 0.0);

            telemetry.addData("D Pad Up", "frontRight");
            telemetry.addData("D Pad Right", "backRight");
            telemetry.addData("D Pad Down", "backLeft");
            telemetry.addData("D Pad Left", "frontLeft");

            telemetry.addData("Y", "turret");
            telemetry.addData("B", "launcher");
            telemetry.addData("A", "intake");
            telemetry.addData("X", "feeder");

            telemetry.addData("right bumper", "rightIntakeServo");
            telemetry.addData("left bumper", "leftIntakeServo");

            telemetry.update();
        }
    }
}
