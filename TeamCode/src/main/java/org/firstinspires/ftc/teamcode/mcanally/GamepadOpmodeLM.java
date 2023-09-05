package org.firstinspires.ftc.teamcode.mcanally;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class GamepadOpmodeLM extends OpMode {
    @Override
    public void init(){
    }

    @Override
    public void loop() {
        telemetry.addData("Left Stick x", gamepad1.left_stick_x);
        telemetry.addData("Left Stick y", gamepad1.left_stick_y);
        telemetry.addData("A button", gamepad1.a);
    }
}
