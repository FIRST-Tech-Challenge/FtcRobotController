package org.firstinspires.ftc.teamcode.huffman;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp() //assigns OpMode to TeleOp dropdown on driver station
@Disabled
public class GamepadOpModeAH extends OpMode {
    @Override
    public void init() { //required even if empty
    }

    @Override
    public void loop() {
        //adds captions of Left stick x, Left stick y, and A button to the driver station and shows their values
        telemetry.addData("Left stick x", gamepad1.left_stick_x);
        telemetry.addData("Left stick y", gamepad1.left_stick_y);
        telemetry.addData("A button", gamepad1.a);
        telemetry.addData("B button", gamepad2.b);
    }
}
