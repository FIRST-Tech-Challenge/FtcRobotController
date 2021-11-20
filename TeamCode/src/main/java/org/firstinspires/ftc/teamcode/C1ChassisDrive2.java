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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Variables.motorBackLeft;
import static org.firstinspires.ftc.teamcode.Variables.motorBackRight;
import static org.firstinspires.ftc.teamcode.Variables.motorFrontLeft;
import static org.firstinspires.ftc.teamcode.Variables.motorFrontRight;
import static org.firstinspires.ftc.teamcode.Variables.motorMajorArm;
import static org.firstinspires.ftc.teamcode.Variables.servoCarousel;
import static org.firstinspires.ftc.teamcode.Variables.servoClamp;
import static org.firstinspires.ftc.teamcode.Variables.servoStable;


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

@TeleOp(name = "C1 Chassis Drive2", group = "A")

public class C1ChassisDrive2 extends DriveMethods {

    // Declare OpMode members.

    //Controller/Elapsed time variables
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();
    double leftY1;
    double leftX1;
    double rightX1;
    double rightTrigger1;
    double leftTrigger1;
    boolean xButton;
    boolean yButton;
    double rightTrigger2;

    // arm
    double level0 = 100;
    double level1 = 315;
    double level2 = 575;
    double level3 = 850;
    double currentPosition;
    double difference;
    double level;
    double multiplier;
    double hold;
    double stabilizer = 0.52;

    // Other variables
    double carouselPosition;
    boolean fast = true;
    boolean clamped = false;



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
        servoStable.setPosition(stabilizer);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        timer.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            String stringStateRuntime = timer.toString();
            String processed = stringStateRuntime.replace(target,replacement);
            Double doubleStateRuntime = Double.parseDouble(processed);



            leftY1 = -gamepad1.left_stick_y;
            leftX1 = gamepad1.left_stick_x;
            rightX1 = gamepad1.right_stick_x;
            rightTrigger1 = gamepad1.right_trigger;
            leftTrigger1 = gamepad1.left_trigger;
            xButton = gamepad1.x;
            yButton = gamepad1.y;
            rightTrigger2 = gamepad2.right_trigger;



            motorFrontLeft.setPower(leftY1 + leftX1 + rightX1);
            motorBackLeft.setPower(leftY1 - leftX1 + rightX1);
            motorFrontRight.setPower(leftY1 - leftX1 - rightX1);
            motorBackRight.setPower(leftY1 + leftX1 - rightX1);

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
            }

            if (xButton) {
                servoCarousel.setPosition(1);
            }
            if(yButton) {
                servoCarousel.setPosition(0);
            }



            if(gamepad2.dpad_down) {
                level = level0;
                multiplier = -0.002;
                hold = 0;
                stabilizer = 0.52;
            }
            if(gamepad2.dpad_left) {
                level = level1;
                multiplier = -0.002;
                hold = -0.06;
                stabilizer = 0.58;
            }
            if(gamepad2.dpad_right) {
                level = level2;
                multiplier = -0.00175;
                hold = -0.075;
                stabilizer = 0.68;
            }
            if(gamepad2.dpad_up) {
                level = level3;
                multiplier = -0.0015;
                hold = -0.06;
                stabilizer = 0.77;
            }

            currentPosition = -motorMajorArm.getCurrentPosition();
            difference = level - currentPosition;
            motorMajorArm.setPower(difference * multiplier + hold);
            servoStable.setPosition(stabilizer);







            /**
             * Safety precaution potentially
             */
            if (currentPosition > 1000) {
                motorMajorArm.setPower(0.2);
                sleep(300);
                motorMajorArm.setPower(-0.003);
                sleep(500);
            }







            if(gamepad2.right_trigger > 0  && doubleStateRuntime > 0.5){
                if(clamped == false) {
                    timer.reset();
                    servoClamp.setPosition(0.5);
                    clamped = true;
                }else if(clamped == true){
                    clamped = false;
                    timer.reset();
                    servoClamp.setPosition(0);
                }
            }



            telemetry.addLine("Arm Power Value: "+  motorMajorArm.getPower());
            telemetry.addLine("Arm Encoder Value: " + currentPosition);
            telemetry.addLine("Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
