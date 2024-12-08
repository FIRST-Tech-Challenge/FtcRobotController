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
import org.firstinspires.ftc.teamcode.teleop.Values;
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

@TeleOp(name="all da servos and Drive", group="Linear OpMode")
//@Disabled
public class ServosNDrive extends LinearOpMode {

    /* Declare OpMode members. */
    public Servo    intakeClaw    = null;
    public Servo    clawPivot   = null;
    public Servo    wrist   = null;
    public Servo    intakeElbow = null;

    public Servo    outtakeClaw = null;
    public Servo    outtakeElbow = null;

    public Servo    slide1 = null;
    public Servo    slide2 = null;

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    private void slidesOut () {
        slide1.setPosition(Values.slide1out);
        slide2.setPosition(Values.slide2out);
    }
    private void slidesIn () {
        slide1.setPosition(Values.slide1in);
        slide2.setPosition(Values.slide2in);
    }

    @Override
    public void runOpMode() {


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "Motor0");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "Motor1");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Motor2");
        rightBackDrive = hardwareMap.get(DcMotor.class, "Motor3");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Define and initialize ALL installed servos.
        intakeClaw  = hardwareMap.get(Servo.class, "0");
        clawPivot = hardwareMap.get(Servo.class, "1");
        wrist  = hardwareMap.get(Servo.class, "2");
        intakeElbow = hardwareMap.get(Servo.class, "3");
        slide2  = hardwareMap.get(Servo.class, "4");
        slide1 = hardwareMap.get(Servo.class, "5");

        clawPivot.setPosition(Values.MID_SERVO);
        wrist.setPosition(Values.MID_SERVO);
        intakeElbow.setPosition(Values.MID_SERVO);

        slide1.setPosition(Values.slide1in);
        slide2.setPosition(Values.slide2in);

        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -currentGamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  currentGamepad1.left_stick_x;
            double yaw     =  currentGamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

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
            }
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            //all servo stuff

            //horizontal slides in N out
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
                if(slide1.getPosition() != Values.slide1in){
                    slidesIn();
                }
                else if(slide1.getPosition() != Values.slide1out){
                    slidesOut();
                }
            }

            //intake open N close
            if (currentGamepad1.circle && !previousGamepad1.circle){
                if(intakeClaw.getPosition() == Values.intakeClawOpen){
                    intakeClaw.setPosition(Values.intakeclawClose);
                }
                else if (intakeClaw.getPosition() == Values.intakeclawClose) {
                    intakeClaw.setPosition(Values.intakeClawOpen);
                }
            }
            //intake wrist
            if (currentGamepad1.square && !previousGamepad1.square){
                if(wrist.getPosition() == Values.wristUp) {
                    wrist.setPosition(Values.wristDown);
                }
                else if (wrist.getPosition() == Values.wristDown) {
                    wrist.setPosition(Values.wristUp);
                }
            }
            //intake elbow
            if (currentGamepad1.cross && !previousGamepad1.cross){
                if(intakeElbow.getPosition() == Values.intakeElbowUp) {
                    intakeElbow.setPosition(Values.intakeElbowDown);
                }
                else if (intakeElbow.getPosition() == Values.intakeElbowDown) {
                    intakeElbow.setPosition(Values.intakeElbowUp);
                }
            }
            //pivot!
            if (currentGamepad1.dpad_left) {
                clawPivot.setPosition(clawPivot .getPosition() - 0.1);
            } else if (currentGamepad1.dpad_right) {
                clawPivot.setPosition(clawPivot .getPosition() + 0.1);
            } else if (currentGamepad1.left_bumper) {
                clawPivot.setPosition(Values.MID_SERVO);
            }

            //Send telemetry message to signify robot running;
            telemetry.addData("Intake Claw (circle)",  "%.02f", intakeClaw.getPosition());
            telemetry.addData("Intake pitch angle (square)",  "%.02f", wrist);
            telemetry.addData("Intake big rotate (cross)",  "%.02f", intakeElbow.getPosition());
            telemetry.addData("slides (dpad up)",  "%.02f, %.02f", slide1.getPosition(), slide2.getPosition());
            telemetry.update();

        }
    }
}
