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

import static org.firstinspires.ftc.teamcode.Variables.motorBL;
import static org.firstinspires.ftc.teamcode.Variables.motorBR;
import static org.firstinspires.ftc.teamcode.Variables.motorFL;
import static org.firstinspires.ftc.teamcode.Variables.motorFR;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="SafeTeleOp1", group="Linear Opmode")

public class SafeTeleOp1 extends DriveMethods {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    public void initMotorsBlue() {
        motorFL  = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorFR  = hardwareMap.get(DcMotor.class, "motorFR");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initMotorsBlue();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double leftY;
        double leftX;
        double rightX;
        double speedDiv = 1;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //update doubles
            leftY = -gamepad1.left_stick_y;
            leftX = gamepad1.left_stick_x;
            rightX = gamepad1.right_stick_x;

            if (!gamepad1.right_bumper) {
                motorFL.setPower((leftY + leftX + rightX) / speedDiv);
                motorBL.setPower((leftY - leftX + rightX) / speedDiv);
                motorFR.setPower((leftY - leftX - rightX) / speedDiv);
                motorBR.setPower((leftY + leftX - rightX) / speedDiv);
            } else {
                motorFL.setPower(0);
                motorBL.setPower(0);
                motorFR.setPower(0);
                motorBR.setPower(0);
            }
            if (gamepad1.a) {
                if (speedDiv != 4) {
                    speedDiv++;
                } else {
                    speedDiv = 1;
                }
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            telemetry.update();
        }
    }
}
