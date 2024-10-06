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

package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.dfrobot.HuskyLens;



@TeleOp(name="TeleOpHusky", group="Test")
public class TeleOpHusky extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // motors
    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    // sensors
    private HuskyLens huskyLens;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        double direction = 1;

        // Husky setup
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        telemetry.update();

        String sampleColor = "NONE";



        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // gamepad 1 controls
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1; // Counteract imperfect strafing

            double rx = gamepad1.right_stick_x;
            if(direction == 1) {
                rx = gamepad1.right_stick_x * -1;
            }
            else {
                rx = gamepad1.right_stick_x;
            }

            if(gamepad1.left_stick_x < 40 && gamepad1.left_stick_x > 60) {
                x = 50;
            }
            if(gamepad1.left_stick_y < 45 && gamepad1.left_stick_y > 55) {
                y = 50;
            }

            int s = 1;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (gamepad1.left_bumper)
                s = 4;
            else if (gamepad1.right_bumper)
                s = 2;
            else
                s = 1;

            backLeft.setPower(backLeftPower / s);
            frontLeft.setPower(frontLeftPower / s);

            frontRight.setPower(frontRightPower / s);
            backRight.setPower(backRightPower / s);



            // HuskyLens

            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                // BLUE
                if (blocks[i].id == 1 && (blocks[i].x > 120 && blocks[i].x < 200) && blocks[i].width > 120) {
                    sampleColor = "BLUE";
                    telemetry.addData("center: ", sampleColor);
                }
                // RED
                if (blocks[i].id == 2 && (blocks[i].x > 120 && blocks[i].x < 200) && blocks[i].width > 120) {
                    sampleColor = "RED";
                    telemetry.addData("center: ", sampleColor);
                }
                // YELLOW
                if (blocks[i].id == 3 && (blocks[i].x > 120 && blocks[i].x < 200) && blocks[i].width > 120) {
                    sampleColor = "YELLOW";
                    telemetry.addData("center: ", sampleColor);
                }
            } //end getting block id
            telemetry.update();


        }
    }
}
