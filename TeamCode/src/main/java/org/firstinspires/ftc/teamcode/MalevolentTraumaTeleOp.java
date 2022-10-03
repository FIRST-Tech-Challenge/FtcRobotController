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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Malevolent Trauma TeleOp", group="Malevolent Trauma")


public class MalevolentTraumaTeleOp extends LinearOpMode {
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;
    double powerLevel = 0.8;

    @Override
    public void runOpMode() {

        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            //slow down power if bumper is pressed
            if(gamepad1.left_bumper){
                powerLevel = 0.5;
            }else{
                powerLevel = 0.8;
            }

            //Checks if the left joystick is moved significantly, otherwise makes sure the motors are stopped
            //Aka "If X or Y are moved more than .1"
            if (Math.abs(gamepad1.left_stick_y) > .1 || Math.abs(gamepad1.left_stick_x) > .1) {
                //Checks if joystick moved more up than side to side, if so, move foward or backward
                //"If joystick moved more vertically than horizontally, then move forward/backward"
                if (Math.abs(gamepad1.left_stick_x) < Math.abs(gamepad1.left_stick_y)) {
                    frontRightMotor.setPower(-gamepad1.left_stick_y * powerLevel);
                    frontLeftMotor.setPower(-gamepad1.left_stick_y * powerLevel);
                    backLeftMotor.setPower(-gamepad1.left_stick_y * powerLevel);
                    backRightMotor.setPower(-gamepad1.left_stick_y * powerLevel);
                    //Checks if moved more horizontally than up and down, if so, strafes
                    //"If joystick moved more horizontally than vertically, strafe"
                } else if (Math.abs(gamepad1.left_stick_y) < Math.abs(gamepad1.left_stick_x)) {
                    frontRightMotor.setPower(-gamepad1.left_stick_x * powerLevel);
                    frontLeftMotor.setPower(gamepad1.left_stick_x * powerLevel);
                    backLeftMotor.setPower(-gamepad1.left_stick_x * powerLevel);
                    backRightMotor.setPower(gamepad1.left_stick_x * powerLevel);
                }
                //Check if the right joystick is moved significantly, otherwise motors are stopped
            }else if(Math.abs(gamepad1.right_stick_x) > 0.1){
                frontRightMotor.setPower(gamepad1.right_stick_x*powerLevel);
                frontLeftMotor.setPower(-gamepad1.right_stick_x*powerLevel);
                backRightMotor.setPower(gamepad1.right_stick_x*powerLevel);
                backLeftMotor.setPower(-gamepad1.right_stick_x*powerLevel);
            } else {
                frontRightMotor.setPower(0);
                frontLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                backLeftMotor.setPower(0);
            }

        }
    }
}
