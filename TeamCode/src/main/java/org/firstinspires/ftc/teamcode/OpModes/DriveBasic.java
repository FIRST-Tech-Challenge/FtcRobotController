package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Control;

@TeleOp
public class DriveBasic extends Control {

    @Override
    public void loop() {
        super.loop();
        double leftY = gamepad1.left_stick_y;
        double rightX = gamepad1.right_stick_x;
        double rightY = gamepad1.right_stick_y;

        //sixDrive(leftY, rightY);
        driveBasic(leftY, rightX);
    }
}
