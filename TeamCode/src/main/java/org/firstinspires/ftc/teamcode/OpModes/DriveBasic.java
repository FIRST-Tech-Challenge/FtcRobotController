package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control;

@TeleOp
public class DriveBasic extends Control {

    @Override
    public void loop() {
        super.loop();
        double leftY = gamepad1.left_stick_y < 5 ? 0 : gamepad1.left_stick_y;
        double rightX = gamepad1.right_stick_x < 5 ? 0 : gamepad1.right_stick_x;

        driveBasic(leftY, rightX);
    }
}
