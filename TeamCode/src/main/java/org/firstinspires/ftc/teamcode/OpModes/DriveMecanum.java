package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control;

@TeleOp
public class DriveMecanum extends Control {
    @Override
    public void loop() {
        super.loop();
        float leftY = gamepad1.left_stick_y < .05 ? 0 : gamepad1.left_stick_y;
        float leftX = gamepad1.left_stick_x < .05 ? 0 : gamepad1.left_stick_x;
        double turn = gamepad1.left_trigger - gamepad1.right_trigger;

        mecanumDrive(leftX, leftY, turn);
    }
}