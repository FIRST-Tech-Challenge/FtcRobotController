
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

package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;



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

@TeleOp(name="Opmode", group="Linear OpMode")

public class BasicOpMode_Linear extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor DcMotorA = hardwareMap.get(DcMotor.class, "m0");
        DcMotor DcMotorB = hardwareMap.get(DcMotor.class, "m1");
        DcMotor DcMotorC = hardwareMap.get(DcMotor.class, "m2");
        DcMotor DcMotorD = hardwareMap.get(DcMotor.class, "m3");
        DcMotor DcMotorE = hardwareMap.get(DcMotor.class,"m4");
        //^for arm
        DcMotorA.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotorB.setDirection(DcMotorSimple.Direction.REVERSE);
        Servo gripServo = hardwareMap.get(Servo.class, "S0");
        Servo droneServo = hardwareMap.get(Servo.class, "S1");
        Servo wristServo = hardwareMap.get(Servo.class, "S2");
//        DcMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        DcMotorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        DcMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        DcMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        DcMotorC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        DcMotorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        DcMotorD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        DcMotorD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DcMotorE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DcMotorE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int  scorePos = -513;
        int  downPos = -10;
        int midPos = -52;


//        boolean lastMovement = false, currMovement = false;
//        boolean downPosition = true;
        waitForStart();
        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {
            telemetry.addData("Motor A Pos:", DcMotorA.getCurrentPosition());
            telemetry.addData("Motor B Pos:", DcMotorB.getCurrentPosition());
            telemetry.addData("Motor C Pos:", DcMotorC.getCurrentPosition());
            telemetry.addData("Motor D Pos:", DcMotorD.getCurrentPosition());
            telemetry.addData("Motor E Pos:", DcMotorE.getCurrentPosition());

            boolean lastMovement = false, currMovement = false;
            boolean downPosition = true;

            if (gamepad1.dpad_up){
                wristServo.setPosition(1);
            }
            if (gamepad1.dpad_down){
                wristServo.setPosition(0.2);
            }

            if (gamepad1.dpad_right){
            }
            if (gamepad1.dpad_left){
            }
            if( (gamepad1.right_stick_x != 0) || (gamepad1.left_stick_y != 0) ) {
                DcMotorE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                DcMotorE.setTargetPosition(midPos);
                DcMotorE.setPower(0.3);
            }



            if (gamepad1.left_stick_y != 0){
                DcMotorA.setPower(-gamepad1.left_stick_y);
                DcMotorB.setPower(-gamepad1.left_stick_y);
                DcMotorC.setPower(-gamepad1.left_stick_y);
                DcMotorD.setPower(-gamepad1.left_stick_y);
            }
            if (gamepad1.left_stick_y == 0) {
                DcMotorA.setPower(0);
                DcMotorB.setPower(0);
                DcMotorC.setPower(0);
                DcMotorD.setPower(0);
            }
            if (gamepad1.right_stick_x != 0){
                DcMotorA.setPower(gamepad1.right_stick_x);
                DcMotorB.setPower(gamepad1.right_stick_x);
                DcMotorC.setPower(-gamepad1.right_stick_x);
                DcMotorD.setPower(-gamepad1.right_stick_x);
            }
            else {
                DcMotorA.setPower(0);
                DcMotorB.setPower(0);
                DcMotorC.setPower(0);
                DcMotorD.setPower(0);
            }

            if (gamepad1.a){
                DcMotorA.setPower(-0.3);
                DcMotorB.setPower(0.3);
                DcMotorC.setPower(0.3);
                DcMotorD.setPower(-0.3);
            }

            if (gamepad1.b){
                gripServo.setPosition(0);
            }
            if (gamepad1.x){
                gripServo.setPosition(0.2);
            }

            if (gamepad1.y){
                droneServo.setPosition(-0.3);
            }


            if(gamepad1.right_bumper){
                telemetry.addLine("L trigger");
                DcMotorE.setTargetPosition(scorePos);
                DcMotorE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                DcMotorE.setPower(0.5);

            }
            if(gamepad1.right_stick_button){
                telemetry.addLine("L trigger");
                DcMotorE.setTargetPosition(downPos);
                DcMotorE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                DcMotorE.setPower(0.3);

            }

            if(gamepad1.left_stick_button){
//                wristServo.setPosition(1);
            }

//            else{
//                telemetry.addLine("L trigger");
//                DcMotorE.setTargetPosition(downPos);
//                DcMotorE.setPower(0.3);
//            }

            // Up is Negative, and Down is Positive
            telemetry.addData("Y Sitck Value",gamepad1.left_stick_y);
            telemetry.addData("X Sitck Value",gamepad1.right_stick_x);
            telemetry.update();
            telemetry.addData("Position A",DcMotorA.getCurrentPosition());
            if(gamepad1.y) {

            }
        }
    }
}
