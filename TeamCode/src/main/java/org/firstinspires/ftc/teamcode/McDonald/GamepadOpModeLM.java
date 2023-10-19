package org.firstinspires.ftc.teamcode.McDonald;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp()
public class GamepadOpModeLM extends OpMode {
    @Override
    public void init() {
    }
//Nothing added to public void init
    @Override
    public void loop() {

//Three new variables created for gamepad1 including yStick, xStick, and trigger
        telemetry.addData("Right stick x", gamepad1.right_stick_x);
        telemetry.addData("Right stick y", gamepad1.right_stick_y);
        telemetry.addData("B button", gamepad1.b);
//Telemetry added to show the difference between left joystick y and right joystick y as well as right joystick x and left joystick x
        double yStick = gamepad1.left_stick_y - gamepad1.right_stick_y;
        double xStick = gamepad1.left_stick_x - gamepad1.right_stick_x;
        double trigger = gamepad1.left_trigger + gamepad1.right_trigger;
//Telemetry add to show the sum of the left and right triggers
        telemetry.addData("yStickdifference", yStick);
        telemetry.addData("trigger sum", trigger);

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;






    }
}

