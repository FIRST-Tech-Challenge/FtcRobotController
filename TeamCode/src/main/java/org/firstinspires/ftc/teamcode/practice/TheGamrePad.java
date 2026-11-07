package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp
public class TheGamrePad extends OpMode {

    @Override
    public void init() {


    }

    @Override
    public void loop() {
        // runs 50x a second
        telemetry.addData("x", gamepad1.left_stick_x);
        telemetry.addData("y", gamepad1.left_stick_y);
        telemetry.addData("a button", gamepad1.a);
        telemetry.addData("b button", gamepad1.b);
        telemetry.addData("y button", gamepad1.y);
        telemetry.addData("x button", gamepad1.x);
        telemetry.addData("x", gamepad1.dpad_down);
        telemetry.addData("y", gamepad1.dpad_down);
        telemetry.addData("x", gamepad1.left_bumper);
        telemetry.addData("y", gamepad1.right_bumper);
    }
}
