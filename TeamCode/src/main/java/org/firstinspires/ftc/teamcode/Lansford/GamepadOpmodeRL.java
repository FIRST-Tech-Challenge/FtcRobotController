package org.firstinspires.ftc.teamcode.Lansford;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class GamepadOpmodeRL extends OpMode {
    @Override
    public void init() {
    }

    @Override
    public void loop() {
        telemetry.addData("Left stick x", gamepad1.left_stick_x);/* this means that if the left stick
        is moved on the x axis, then it will move left or right.*/
        telemetry.addData("left stick y", gamepad1. left_stick_y);/* this means that if you move the
        left stick along the y axis, then it will move forward or backward.*/
        telemetry.addData("A button", gamepad1.a);/* this means that if you press the A button,
         then the function that is coded for the a button will be enacted*/
    }
}
