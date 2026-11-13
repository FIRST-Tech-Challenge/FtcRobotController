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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.base.GreenBotBaseOpMode;

@TeleOp(name = "Motor Tester - Green Bot", group = "Test")
public class MotorTestGreen extends GreenBotBaseOpMode {

    private static final double DRIVE_POWER = 0.2;

    // Launcher top-speed test (hold right trigger): full power with no velocity control, to
    // measure the flywheel's real top speed and from it the launcher's KF.
    private static final double LAUNCHER_TEST_POWER = 1.0;
    private static final double SETTLE_SECONDS = 2.0; // ignore readings while it spins up

    @Override
    public void runOpMode() {

        config();
        // Raw power instead of the hub's velocity control, so full power really is full power.
        launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        ElapsedTime launcherTimer = new ElapsedTime();
        boolean wasTesting = false;
        double velocitySum = 0;
        int velocitySamples = 0;

        waitForStart();

        while (opModeIsActive()) {
            frontRight.setPower(gamepad1.dpad_up ? DRIVE_POWER : 0.0);
            backRight.setPower(gamepad1.dpad_right ? DRIVE_POWER : 0.0);
            frontLeft.setPower(gamepad1.dpad_left ? DRIVE_POWER : 0.0);
            backLeft.setPower(gamepad1.dpad_down ? DRIVE_POWER : 0.0);

            turret.setPower(gamepad1.y ? DRIVE_POWER : 0.0);

            // Launcher: B = gentle spin, right trigger (hold) = full-power top-speed test.
            // A new press starts a fresh measurement; the last result stays up after release.
            boolean testing = gamepad1.right_trigger > 0.5;
            if (testing && !wasTesting) {
                launcherTimer.reset();
                velocitySum = 0;
                velocitySamples = 0;
            }
            wasTesting = testing;
            if (testing) launcherMotor.setPower(LAUNCHER_TEST_POWER);
            else launcherMotor.setPower(gamepad1.b ? DRIVE_POWER : 0.0);

            double launcherVelocity = launcherMotor.getVelocity();
            if (testing && launcherTimer.seconds() > SETTLE_SECONDS) {
                velocitySum += launcherVelocity;
                velocitySamples++;
            }

            intake.setPower(gamepad1.a ? DRIVE_POWER : 0.0);
            feeder.setPower(gamepad1.x ? DRIVE_POWER * 2 : 0.0);

            telemetry.addData("D Pad Up", "frontRight");
            telemetry.addData("D Pad Right", "backRight");
            telemetry.addData("D Pad Down", "backLeft");
            telemetry.addData("D Pad Left", "frontLeft");

            telemetry.addData("Y", "turret");
            telemetry.addData("B", "launcher");
            telemetry.addData("A", "intake");
            telemetry.addData("X", "feeder");
            telemetry.addData("RT (hold)", "launcher FULL POWER top-speed test");

            telemetry.addLine();
            telemetry.addData("Battery", "%.2f V", battery.getVoltage());
            telemetry.addData("Launcher velocity", "%.0f ticks/s", launcherVelocity);
            if (velocitySamples > 0) {
                double topSpeed = velocitySum / velocitySamples;
                telemetry.addData("Launcher top speed", "%.0f ticks/s   (avg of %d readings)",
                        topSpeed, velocitySamples);
                telemetry.addData("Suggested KF", "%.2f   (32767 / top speed)", 32767 / topSpeed);
            } else if (testing) {
                telemetry.addLine("Spinning up... keep holding RT");
            } else {
                telemetry.addLine("Hold RT for 3+ seconds to measure launcher top speed");
            }

            telemetry.update();
        }
    }
}
