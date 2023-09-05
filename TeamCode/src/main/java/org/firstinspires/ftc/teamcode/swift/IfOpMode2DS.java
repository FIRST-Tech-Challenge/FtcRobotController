package org.firstinspires.ftc.teamcode.swift;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class IfOpMode2DS extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Made By", "Dawston");
        //digital signature so I know its mine
    }
    @Override
    public void loop() {

        if (gamepad1.b)
            telemetry.addData("Turbo Mode", "Activated");
        else
            telemetry.addData("Turbo Mode", "Deactivated");

        double ySpeed = -gamepad1.left_stick_y;
        double xSpeed = gamepad1.left_stick_x;

        if (!gamepad1.b)
            ySpeed *= .5;
            //if its not pressed then the speed is half of the whole value
        telemetry.addData("Speed", ySpeed);
// calls the variable ySpeed to be displayed on the drive hub
        if (gamepad1.a) {
            telemetry.addData("Crazy Mode", "Activated");
            xSpeed = -gamepad1.left_stick_y;
            ySpeed = gamepad1.left_stick_x;
            //swaps crazy mode x and y when pressed
        }
        else {
            telemetry.addData("Crazy Mode", "Deactivated");
            // crazy mode deactivated when not pressed
        }
        telemetry.addData("Left stick y", ySpeed);
        telemetry.addData("Left stick x", xSpeed);
        //default x and y value
    }
}
