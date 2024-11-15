package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class stickDriftTest extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        telemetry.addData("Gamepad 1 L X", gamepad1.left_stick_x);
        telemetry.addData("Gamepad 1 L Y", gamepad1.left_stick_y);
        telemetry.addData("Gamepad 1 R X", gamepad1.right_stick_x);
        telemetry.addData("Gamepad 1 R Y", gamepad1.right_stick_y);
        telemetry.addData("Gamepad 2 L X", gamepad2.left_stick_x);
        telemetry.addData("Gamepad 2 L Y", gamepad2.left_stick_y);
        telemetry.addData("Gamepad 2 R X", gamepad2.right_stick_x);
        telemetry.addData("Gamepad 2 R Y", gamepad2.right_stick_y);
        telemetry.addData("Gamepad 1 A", gamepad1.a);
        telemetry.update();
    }
}
