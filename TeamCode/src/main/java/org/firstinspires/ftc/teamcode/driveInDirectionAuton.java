/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

import java.util.Arrays;
import java.util.Collections;

@Autonomous(name="driveInDirectionAuton", group="Zippo")

public class driveInDirectionAuton extends LinearOpMode {

    /* Declare OpMode members. */
    testPlatformHardware robot  = new testPlatformHardware();
    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        int x = selectInt("X inches");
        sleep(200);
        int y = selectInt("Y inches");
        wheelMecanumDrive(calculateInches(x,y), Math.hypot(x,y)/3);
    }
    public void wheelMecanumDrive(double[] inches, double timeoutS) {
        int FLTarget = 0;
        int FRTarget = 0;
        int BLTarget = 0;
        int BRTarget = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            double inchesMax = 0;
            for (double inch : inches) {
                if (inch > inchesMax) {
                    inchesMax = inch;
                }
            }

            FLTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (inches[0] * testPlatformHardware.COUNTS_PER_INCH);
            FRTarget = robot.motorFrontRight.getCurrentPosition() + (int) (inches[1] * testPlatformHardware.COUNTS_PER_INCH);
            BLTarget = robot.motorBackLeft.getCurrentPosition() + (int) (inches[2] * testPlatformHardware.COUNTS_PER_INCH);
            BRTarget = robot.motorBackRight.getCurrentPosition() + (int) (inches[3] * testPlatformHardware.COUNTS_PER_INCH);

            robot.motorFrontLeft.setTargetPosition(FLTarget);
            robot.motorFrontRight.setTargetPosition(FRTarget);
            robot.motorBackLeft.setTargetPosition(BLTarget);
            robot.motorBackRight.setTargetPosition(BRTarget);


            // Turn On RUN_TO_POSITION
            setAllMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            setAllPower(testPlatformHardware.DRIVE_SPEED);


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.motorFrontLeft.isBusy() || robot.motorFrontRight.isBusy())) {
                //Display data for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", FLTarget, FRTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.motorFrontLeft.getCurrentPosition(),
                        robot.motorFrontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            setAllPower(0);

            // Turn off RUN_TO_POSITION
            setAllMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void setAllPower(double power) {
        robot.motorFrontLeft.setPower(power);
        robot.motorFrontRight.setPower(power);
        robot.motorBackLeft.setPower(power);
        robot.motorBackRight.setPower(power);
    }
    public void setAllMode(DcMotor.RunMode mode) {
        robot.motorFrontLeft.setMode(mode);
        robot.motorFrontRight.setMode(mode);
        robot.motorBackLeft.setMode(mode);
        robot.motorBackRight.setMode(mode);
    }
    public double[] calculateInches(double xInches, double yInches) {
        double r = Math.hypot(xInches, yInches);
        double robotAngle = Math.atan2(yInches, xInches) - Math.PI / 4;
        return new double[]{r * Math.cos(robotAngle), r * Math.sin(robotAngle), r * Math.sin(robotAngle), r * Math.cos(robotAngle)}; //fl,fr,bl,br
    }
    public int selectInt(String input) {
        int selected = 0;
        do {
            if (gamepad1.dpad_left) {
                selected -= 1;
            } else if (gamepad1.dpad_right) {
                selected += 1;
            }
            telemetry.addData(input, selected);
            telemetry.addLine("Press the A button to confirm");
            sleep(100);
            telemetry.update();
        } while (!gamepad1.a || isStopRequested());
        return selected;
    }

}
