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

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name="TestDrive01", group="Linear Opmode")
public class TestDrive01 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor01;
    private DcMotor motor02;
    private DcMotor motor03;
    private DcMotor motor04;
    private Servo servo01;

    private double oldMotor01Power;
    private double oldMotor02Power;
    private double oldMotor03Power;
    private double oldMotor04Power;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // Yay! Git. :)
        motor01  = hardwareMap.get(DcMotor.class, "motor1");
        motor02 = hardwareMap.get(DcMotor.class, "motor2");
        motor03 = hardwareMap.get(DcMotor.class, "motor3");
        motor04 = hardwareMap.get(DcMotor.class, "motor4");
        servo01 = hardwareMap.get(Servo.class,"servo1");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        motor01.setDirection(DcMotor.Direction.REVERSE);
        //motor02.setDirection(DcMotor.Direction.FORWARD);
        motor03.setDirection(DcMotor.Direction.REVERSE);
        //motor04.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double lefty = -gamepad1.left_stick_y;
            double leftx = gamepad1.left_stick_x;
            double righty = -gamepad1.right_stick_y;
            double rightx = gamepad1.right_stick_x;
            //servo01.setDirection();

            //if (lefty > rightx) {
            if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
                motor01.setPower(lefty - leftx);
                motor02.setPower(lefty + leftx);
                motor03.setPower(lefty + leftx);
                motor04.setPower(lefty - leftx);
                //} if (lefty < rightx) {
                motor01.setPower(rightx);
                motor02.setPower(-rightx);
                motor03.setPower(rightx);
                motor04.setPower(-rightx);
                oldMotor01Power = motor01.getPower();
                oldMotor02Power = motor02.getPower();
                oldMotor03Power = motor03.getPower();
                oldMotor04Power = motor04.getPower();
            }
            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                motor01.setPower(-oldMotor01Power);
                motor02.setPower(-oldMotor02Power);
                motor03.setPower(-oldMotor03Power);
                motor04.setPower(-oldMotor04Power);
                sleep(100);
                motor01.setPower(0);
                motor02.setPower(0);
                motor03.setPower(0);
                motor04.setPower(0);
            }
            //}
        }
    }
}
