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

import static org.firstinspires.ftc.teamcode.Variables.motorBackLeft;
import static org.firstinspires.ftc.teamcode.Variables.motorBackRight;
import static org.firstinspires.ftc.teamcode.Variables.motorFrontLeft;
import static org.firstinspires.ftc.teamcode.Variables.motorFrontRight;
import static org.firstinspires.ftc.teamcode.Variables.motorMajorArm;
import static org.firstinspires.ftc.teamcode.Variables.motorTankTread;
import static org.firstinspires.ftc.teamcode.Variables.servoCarousel;
import static org.firstinspires.ftc.teamcode.Variables.servoClamp;
import static org.firstinspires.ftc.teamcode.Variables.servoStable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.tree.DCTree;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "C1 Chassis Drive", group = "A")

public class C1ChassisDrive extends DriveMethods {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    double leftY1;
    double leftX1;
    double rightX1;
    double rightTrigger1;
    double leftTrigger1;
    double carouselPosition;
    boolean xButton;
    double leftY2;
    double rightTrigger2;
    double level0 = 0;
    double level1 = 315;
    double level2 = 550;
    double level3 = 775;
    double currentPosition;
    double difference;
    boolean movingToLevel0 = false;
    boolean movingToLevel1Up = false;
    boolean movingToLevel1Down = false;
    boolean movingToLevel2Up = false;
    boolean movingToLevel2Down = false;
    boolean movingToLevel3Up = false;
    boolean movingToLevel3Down = false;
    boolean fast = true;



    private final String replacement = "";
    private final String target = " seconds";


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initializeDevices();


        setMotorDirections();

        motorMajorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorMajorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorMajorArm.setPower(-0.15);
        servoStable.setPosition(0.52);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            currentPosition = -motorMajorArm.getCurrentPosition();

            leftY1 = -gamepad1.left_stick_y;
            leftX1 = gamepad1.left_stick_x;
            rightX1 = gamepad1.right_stick_x;
            rightTrigger1 = gamepad1.right_trigger;
            leftTrigger1 = gamepad1.left_trigger;
            xButton = gamepad1.x;
            leftY2 = gamepad2.left_stick_y;
            rightTrigger2 = gamepad2.right_trigger;


            //This allows us to slow down the chassis for more precise driving
            if(gamepad1.right_bumper){
                fast = true;
            }
            if(gamepad1.left_bumper){
                fast = false;
            }

            //Both of the statements are the same, the online difference is the 0.5 applied to one to slow down the chassis
            if(fast = true) {
                motorFrontLeft.setPower(leftY1 + leftX1 + rightX1);
                motorBackLeft.setPower(leftY1 - leftX1 + rightX1);
                motorFrontRight.setPower(leftY1 - leftX1 - rightX1);
                motorBackRight.setPower(leftY1 + leftX1 - rightX1);
            }else if(fast = false){
                motorFrontLeft.setPower(0.5*(leftY1 + leftX1 + rightX1));
                motorBackLeft.setPower(0.5*(leftY1 - leftX1 + rightX1));
                motorFrontRight.setPower(0.5*(leftY1 - leftX1 - rightX1));
                motorBackRight.setPower(0.5*(leftY1 + leftX1 - rightX1));
            }


            /**
             * MajorArm is moving up
             */

            if (movingToLevel1Up && currentPosition > level1) {
                motorMajorArm.setPower(-0.15);
                movingToLevel1Up = false;
            }

            if (movingToLevel2Up && currentPosition > level2) {
                motorMajorArm.setPower(-0.15);
                movingToLevel2Up = false;

            }
            if (movingToLevel3Up && currentPosition > level3) {
                motorMajorArm.setPower(-0.075);
                movingToLevel3Up = false;

            }

            /**
             * MajorArm is moving down
             */

            if (movingToLevel0 && currentPosition <= level0) {
                motorMajorArm.setPower(0);
                movingToLevel0 = false;

            }
            if (movingToLevel1Down && currentPosition < level1) {
                motorMajorArm.setPower(-0.15);
                movingToLevel1Down = false;
            }

            if (movingToLevel2Down && currentPosition < level2) {
                motorMajorArm.setPower(-0.15);
                movingToLevel2Down = false;

            }

            if (movingToLevel3Down && currentPosition < level3) {
                motorMajorArm.setPower(-0.15);
                movingToLevel3Down = false;

            }

            //servos
            carouselPosition = 0.5 + (rightTrigger1 / 2);
            //constrains servo motion to correct direction
            if (carouselPosition > .6) {
                carouselPosition = .6;
            }
            servoCarousel.setPosition(carouselPosition);
            if (xButton) {
                servoCarousel.setPosition(1);
            }

            //gently pushes robot backward whilst rightTrigger1 engaged
            if (rightTrigger1 > 0) {
                servoCarousel.setPosition(0.6);
                while (rightTrigger1 > 0) {
                    rightTrigger1 = gamepad1.right_trigger;
                    driveDirection(.2, Direction.BACKWARD);
                }
                servoCarousel.setPosition(1);
                sleep(500);
                driveForTime(0.2, 250, Direction.FORWARD);
                servoCarousel.setPosition(0.5);
            } else {
                driveDirection(0, Direction.BACKWARD);
            }


            if (leftTrigger1 > 0) {
                servoCarousel.setPosition(0.4);
                while (leftTrigger1 > 0) {
                    leftTrigger1 = gamepad1.left_trigger;
                    driveDirection(.2, Direction.BACKWARD);
                }
                servoCarousel.setPosition(0);
                sleep(500);
                driveForTime(0.2, 250, Direction.FORWARD);
                servoCarousel.setPosition(0.5);
            } else {
                driveDirection(0, Direction.BACKWARD);
            }

            if(leftY2 > 0){
                motorMajorArm.setPower(leftY2 * 0.65);
            }


            //Major Excavator Arm Control is in 3 levels, denoted by the dpad (in order): up, right, bottom
            if (gamepad2.dpad_up) {
                difference = level0 - currentPosition;
                motorMajorArm.setPower(0);
                movingToLevel0 = true;
                currentPosition = -motorMajorArm.getCurrentPosition();
                if (difference < 120 && difference > 10) {
                    motorMajorArm.setPower(-.0015);
                    while (difference == 20) {
                        motorMajorArm.setPower(-.075);
                    }
                }
                motorMajorArm.setPower(0);
                servoStable.setPosition(0.52);
            }

            if (gamepad2.dpad_right) {
                currentPosition = -motorMajorArm.getCurrentPosition();
                difference = level1 - currentPosition;
                if (difference > 0) {
                    currentPosition = -motorMajorArm.getCurrentPosition();
                    motorMajorArm.setPower(-0.275);
                    movingToLevel1Up = true;
                } else if (difference < 0) {
                    currentPosition = -motorMajorArm.getCurrentPosition();
                    motorMajorArm.setPower(-0.0015);
                    movingToLevel1Down = true;

                } else {
                    motorMajorArm.setPower(-0.06);
                }
                currentPosition = -motorMajorArm.getCurrentPosition();
                servoStable.setPosition(0.58);
            }

            if (gamepad2.dpad_down) {
                currentPosition = -motorMajorArm.getCurrentPosition();
                difference = level2 - currentPosition;
                if (difference > 0) {
                    motorMajorArm.setPower(-0.3);
                    movingToLevel2Up = true;
                } else if (difference < 0) {
                    motorMajorArm.setPower(-0.002);
                    movingToLevel2Down = true;
                } else {
                    motorMajorArm.setPower(-0.075);
                }
                currentPosition = -motorMajorArm.getCurrentPosition();
                servoStable.setPosition(0.68);
            }

            if (gamepad2.dpad_left) {
                currentPosition = -motorMajorArm.getCurrentPosition();
                difference = level3 - currentPosition;
                if (difference > 0) {
                    motorMajorArm.setPower(-0.325);
                    movingToLevel3Up = true;
                } else if (difference < 0) {
                    motorMajorArm.setPower(-0.002);
                    movingToLevel3Down = true;
                } else {
                    motorMajorArm.setPower(-0.06);
                }
                currentPosition = -motorMajorArm.getCurrentPosition();
                servoStable.setPosition(0.77);
            }

            if (movingToLevel0) {
                servoStable.setPosition(0.52);
            }

            if (movingToLevel1Up || movingToLevel1Down) {
                servoStable.setPosition(0.58);
            }

            if (movingToLevel2Up || movingToLevel2Down) {
                servoStable.setPosition(0.68);
            }

            if (movingToLevel3Up || movingToLevel3Down) {
                servoStable.setPosition(0.77);
            }

            if(gamepad2.right_bumper){
                servoClamp.setPosition(0.45);
            }
            if(gamepad2.left_bumper){
                servoClamp.setPosition(0);
            }
            /**
             * Safety precaution potentially
             */
            if (currentPosition > 1200) {
                motorMajorArm.setPower(0.1);
                sleep(300);
                motorMajorArm.setPower(-0.0025);
                sleep(500);


            }

            telemetry.addLine("Arm Power Value: "+  motorMajorArm.getPower());
            telemetry.addLine("Arm Encoder Value: " + currentPosition);
            telemetry.addLine("Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
