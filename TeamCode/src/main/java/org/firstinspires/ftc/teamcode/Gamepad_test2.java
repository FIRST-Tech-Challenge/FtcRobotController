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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp", group="Linear OpMode")
// @Disabled
public class Gamepad_test2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftwheel = null;
    public DcMotor rightwheel = null;
    public Servo leftservo ;
    public Servo rightservo ;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftwheel  = hardwareMap.get(DcMotor.class, "leftwheel");
        rightwheel = hardwareMap.get(DcMotor.class, "rightwheel");
        leftservo  = hardwareMap.get(Servo.class, "leftservo");
        rightservo = hardwareMap.get(Servo.class, "rightservo");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftwheel.setDirection(DcMotor.Direction.REVERSE);
        rightwheel.setDirection(DcMotor.Direction.FORWARD);
        leftservo.setDirection(Servo.Direction.FORWARD);
        rightservo.setDirection(Servo.Direction.FORWARD);


        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        leftservo.setPosition(0.5);
        rightservo.setPosition(0.5);
        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double leftAngle;
            double rightAngle;
            double leftdirection;
            double rightdirection;
            double radius;
            double amount_pushed;
            double leftServoPosition;
            double rightServoPosition;
            double apressed;
            double bpressed;

            leftServoPosition = leftservo.getPosition();
            rightServoPosition = rightservo.getPosition();

            //moving

            leftPower  = -gamepad1.left_stick_y ;
            rightPower = -gamepad1.right_stick_y ;

            apressed = 0;
            bpressed = 0;
            //servo movement

            if (gamepad1.a) {
                if (leftServoPosition == 0.42d) {
                    leftservo.setPosition(0d);
                    apressed = 1;
                } else {
                    leftservo.setPosition(0.42d);
                }
            }

            if (gamepad1.b) {
                if (rightServoPosition == 0.42d) {
                    rightservo.setPosition(0d);
                    bpressed = 1;
                } else {
                    rightservo.setPosition(0.42d);
                }
            }

            //leftAngle  = Math.toDegrees(Math.atan(gamepad1.left_stick_y/gamepad1.left_stick_x));
//            rightAngle = Math.toDegrees(Math.atan(gamepad1.right_stick_y/gamepad1.right_stick_x));
//            leftdirection = (leftAngle/360);
//            rightdirection = (rightAngle/360);
//
//            if (gamepad1.left_stick_x == 0) {
//                leftservo.setPosition(0.5);
//            }
//            else {
//                leftservo.setPosition(leftdirection);
//            }
//
//            if (gamepad1.right_stick_x == 0) {
//                rightservo.setPosition(0.5);
//            }
//            else {
//                rightservo.setPosition(rightdirection);
//            }


            // Send calculated power to wheels
            leftwheel.setPower(leftPower);
            rightwheel.setPower(rightPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("is a pressed", apressed);
            telemetry.addData("is b pressed", bpressed);
            telemetry.addData("right servo direction", rightservo.getDirection());
            telemetry.addData("left servo direction", leftservo.getDirection());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
