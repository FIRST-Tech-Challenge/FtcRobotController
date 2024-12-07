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

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
/*
 * This OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name = "all da servos and Drive", group = "Linear OpMode")
//@Disabled
public class ServosNDrive extends LinearOpMode {

    public static final double MID_SERVO = 0.5;
    /* Declare OpMode members. */
    public Servo intakeClaw = null;
    public Servo clawPivot = null;
    public Servo wrist = null;
    public Servo intakeElbow = null;
    public Servo outtakeClaw = null;
    public Servo outtakeElbow = null;
    public Servo intakeSlide1 = null;
    public Servo intakeSlide2 = null;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    private DcMotor outtakeSlide1;
    private DcMotor outtakeSlide2;
    private final int[] slidePosition = {0};
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    @Override
    public void runOpMode() {


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive = hardwareMap.get(DcMotor.class, "Motor0");
        leftBackDrive = hardwareMap.get(DcMotor.class, "Motor1");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Motor2");
        rightBackDrive = hardwareMap.get(DcMotor.class, "Motor3");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Define and initialize ALL installed servos.
        intakeClaw = hardwareMap.get(Servo.class, "0");
        clawPivot = hardwareMap.get(Servo.class, "1");
        wrist = hardwareMap.get(Servo.class, "2");
        intakeElbow = hardwareMap.get(Servo.class, "3");
        outtakeClaw = hardwareMap.get(Servo.class, "4");

        //todo servo slides servo config name
        //intakeSlide1 = hardwareMap.get(Servo.class, "");
        //intakeSlide2 = hardwareMap.get(Servo.class, "");

        // slide motors

        //todo define slides motor config name
        //outtakeSlide1 = hardwareMap.get(DcMotor.class, "");
        //outtakeSlide2 = hardwareMap.get(DcMotor.class, "");
        outtakeSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //outtakeElbow = hardwareMap.get(Servo.class, "5");

        clawPivot.setPosition(MID_SERVO);
        wrist.setPosition(MID_SERVO);
        intakeElbow.setPosition(MID_SERVO);

        //todo set resting positions for intake slides
        //intakeSlide1.setPosition();
        //intakeSlide2.setPosition();

        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -currentGamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = currentGamepad1.left_stick_x;
            double yaw = currentGamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            //all servo stuff

            if (currentGamepad1.circle && previousGamepad1.circle) {
                intakeClaw.setPosition(Values.intakeclawClose);
            } else if (currentGamepad1.circle && !previousGamepad1.circle) {
                intakeClaw.setPosition(Values.intakeClawOpen);
            }

            if (currentGamepad1.cross && previousGamepad1.cross) {
                intakeElbow.setPosition(Values.intakeElbowDown);
            } else if (currentGamepad1.cross && !previousGamepad1.cross) {
                intakeElbow.setPosition(Values.intakeElbowUp);
            }

            if (currentGamepad1.square && previousGamepad1.square) {
                outtakeElbow.setPosition(Values.outtakeElbowDown);
            } else if (currentGamepad1.square && !previousGamepad1.square) {
                outtakeElbow.setPosition(Values.outtakeElbowUp);
            }

            if (currentGamepad1.triangle && previousGamepad1.triangle) {
                outtakeClaw.setPosition(.5);
            } else if (currentGamepad1.triangle && !previousGamepad1.triangle) {
                outtakeClaw.setPosition(1);
            }
            if (currentGamepad1.left_bumper && previousGamepad1.left_bumper) {
                //todo intake slide code in
            }
            if (currentGamepad1.left_bumper && previousGamepad1.left_bumper) {
                //todo intake slide code out
            }

            //slide outtake stuff

            if ((currentGamepad1.left_trigger != 0 && previousGamepad1.left_trigger != 0) || (currentGamepad1.right_trigger != 0 && previousGamepad1.right_trigger != 0)) {

                if (currentGamepad1.right_trigger > 0) {
                    outtakeClaw.setPosition(Values.outtakeClawClose);
                    slidePosition[0] += (int) (20 * currentGamepad1.right_trigger);
                }

                // Move Slide Down
                if (currentGamepad1.left_trigger > 0 && slidePosition[0] > -2300) {
                    slidePosition[0] -= (int) (20 * currentGamepad1.left_trigger);
                }

                // Additional controls...

                // Ensure slides stay within bounds
                if (slidePosition[0] < 0) {
                    slidePosition[0] = 0;
                }

                if (slidePosition[0] > Values.slideMax) {
                    slidePosition[0] = Values.slideMax;
                }
                moveSlides(slidePosition[0], 1);
            }

            //todo telemetry
            //Send telemetry message to signify robot running

            //telemetry.addData("Intake Claw",  "%.2f", intakeClaw.getPosition());
            //telemetry.addData("Intake yaw",  "%.2f", wrist);
            //telemetry.addData("Intake big rotate",  "%.2f", intakeElbow.getPosition());
            //telemetry.addData("outtake Claw",  "%.2f", outtakeClaw.getPosition());
            //telemetry.addData("outtake rotate",  "%.2f", outtakeElbow.getPosition());
            //telemetry.update();
        }
    }

    //methods
    private void moveSlides(int distance, double velocity) {
        outtakeSlide1.setTargetPosition(-distance);
        outtakeSlide2.setTargetPosition(distance);

        outtakeSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        outtakeSlide1.setPower(velocity);
        outtakeSlide2.setPower(velocity);
    }
}