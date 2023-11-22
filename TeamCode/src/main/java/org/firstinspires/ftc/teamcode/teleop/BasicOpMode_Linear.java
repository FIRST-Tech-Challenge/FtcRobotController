/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name = "Main TeleOp", group = "Linear OpMode")
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private CRServo intakeSystem = null;
    private DcMotor armExtendMotor = null;

    private DcMotor armRotator = null;
    private CRServo wrist = null;
    private CRServo hand = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        armRotator = hardwareMap.get(DcMotor.class, "armRotator");

        armExtendMotor = hardwareMap.get(DcMotor.class, "armExtendMotor");

        intakeSystem = hardwareMap.get(CRServo.class, "outputServo");

        wrist = hardwareMap.get(CRServo.class, "wrist");
        hand = hardwareMap.get(CRServo.class, "hand");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        armExtendMotor.setDirection(DcMotor.Direction.REVERSE);

        armRotator.setDirection(DcMotor.Direction.REVERSE);

        intakeSystem.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeSystem.resetDeviceConfigurationForOpMode();

        hand.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        boolean open = true;
        //hand.setPower(1);

        boolean wristDirection = true;

        int waitTime = 0;


        int timeToMove = 0;

        wrist.setPower(0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            double arm = -gamepad2.right_stick_y * 0.65;

            double outputServo = gamepad2.right_stick_y;

            double speedMultiplier = 0.5;

            if (gamepad1.right_bumper) { // Right bumper slows down the speed on the wheels
                if (speedMultiplier == 1) {
                    speedMultiplier = 0.5;
                } else if (speedMultiplier == 0.5) {
                    speedMultiplier = 1;
                }
            }

            if (gamepad1.left_bumper) { // Left bumper slows down the speed on the wheels
                if (speedMultiplier == 0.5) {
                    speedMultiplier = 0.25;
                } else if (speedMultiplier == 0.25) {
                    speedMultiplier = 0.5;
                }
            }

            double armPower = arm;
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = (axial - lateral + yaw) * speedMultiplier;
            double rightFrontPower = (axial - lateral - yaw)* speedMultiplier;
            double leftBackPower   = (axial + lateral + yaw) * speedMultiplier;
            double rightBackPower  = (axial + lateral - yaw) * speedMultiplier;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
                armPower /= max;
            }

            /*if (gamepad2.a) {
                if (armRotator.getCurrentPosition() < 5) {
                    armRotator.setTargetPosition(50);
                    armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armRotator.setPower(0.03);
                } else {
                    //537.7
                    armRotator.setTargetPosition(0);
                    armRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armRotator.setPower(0.03);
                }
            }
             */

            //TODO: automate the arm rotator using setPosition and pray that it works
            if (gamepad2.dpad_up) {
                armRotator.setPower(-0.5);
            }
            if (gamepad2.dpad_down) {
                armRotator.setPower(0.25);
            }

            if (!gamepad2.dpad_up && !gamepad2.dpad_down){
                armRotator.setPower(0);
            }

            //TODO: have to automate the wrist
            if (gamepad2.a) { //change the wrist because the automatic thing wasn't working
                wrist.setPower(-0.5);
            }
            else if (gamepad2.b) {
                wrist.setPower(0.5);
            }

            if (!gamepad2.a && !gamepad2.b) {
                wrist.setPower(0);
            }

            //Closes the hand
            if (gamepad2.x && waitTime == 0) {
                if (open) {
                    hand.setPower(-0.27); //-0.27
                    open = false;
                } else {
                    hand.setPower(0.3);
                    open = true;
                }
                waitTime++;
            }

            if (waitTime != 0)
                waitTime++;
            if (waitTime > 250)
                waitTime = 0;

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            armExtendMotor.setPower(armPower);

            intakeSystem.setPower(-gamepad2.left_stick_y * 0.3);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("waitTime: ", waitTime);
            telemetry.addData("arm rotator: ", armRotator.getCurrentPosition());
            telemetry.addData("timeToMove: ", timeToMove);
            telemetry.addData("target position ", armRotator.getTargetPosition());
            telemetry.addData("ArmRotatorPower", armRotator.getPower());
            telemetry.addData("dPad Up", gamepad2.dpad_up);
            telemetry.update();
        }
}}
