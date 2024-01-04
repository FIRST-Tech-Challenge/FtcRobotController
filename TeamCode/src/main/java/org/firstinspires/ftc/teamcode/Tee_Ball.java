/*
Copyright 2021 FIRST Tech Challenge Team FTC

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class Tee_Ball extends LinearOpMode {
    public DcMotor back_Left;
    public DcMotor back_Right;
    public DcMotor front_Left;
    public DcMotor front_Right;
    public Blinker expansion_Hub_2;
    public Blinker expansion_Hub_9;
    public DcMotor bart;

    @Override
    public void runOpMode() {
        //naming motors/sensors
        back_Left = hardwareMap.get(DcMotor.class, "back_Left");
        back_Right = hardwareMap.get(DcMotor.class, "back_Right");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        expansion_Hub_9 = hardwareMap.get(Blinker.class, "Expansion Hub 9");
        front_Left = hardwareMap.get(DcMotor.class, "front_Left");
        front_Right = hardwareMap.get(DcMotor.class, "front_Right");
        bart = hardwareMap.get(DcMotor.class, "bart");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            // run until the end of the match (driver presses STOP)

            //reversing motors
            //back_Left.setDirection(DcMotor.Direction.REVERSE);
            back_Right.setDirection(DcMotor.Direction.REVERSE);
            //front_Left.setDirection(DcMotor.Direction.REVERSE);
            front_Right.setDirection(DcMotor.Direction.REVERSE);

            //wheels normal drive
            front_Right.setPower(gamepad1.right_stick_y);
            back_Right.setPower(gamepad1.right_stick_y);
            front_Left.setPower(gamepad1.left_stick_y);
            back_Left.setPower(gamepad1.left_stick_y);

            //strafing
            if (gamepad1.right_trigger > .25) {
                front_Left.setPower(-gamepad1.right_trigger);
                back_Left.setPower(gamepad1.right_trigger);
                front_Right.setPower(gamepad1.right_trigger);
                back_Right.setPower(-gamepad1.right_trigger);

            } else if (gamepad1.left_trigger > .25) {
                front_Left.setPower(gamepad1.left_trigger);
                back_Left.setPower(-gamepad1.left_trigger);
                front_Right.setPower(-gamepad1.left_trigger);
                back_Right.setPower(gamepad1.left_trigger);
            }
            ;

            //running motor to a specific position
            double rotation = 288;
            if (gamepad2.a) {
                bart.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                bart.setTargetPosition(288);
                bart.setPower(0.5);
                bart.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                bart.setPower(0);

                //These telemetry commands will send information back to the Driver Controlled Phone
                telemetry.addData("Status", "Running");
                telemetry.addData("Left Back", back_Left.getPower());
                telemetry.addData("Right Back", back_Right.getPower());
                telemetry.addData("Left Front", front_Left.getPower());
                telemetry.addData("Right Front", front_Right.getPower());
                telemetry.update();
            }
        }
    }}
