package org.firstinspires.ftc.teamcode.huffman;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp() //assigns OpMode to TeleOp dropdown on driver station
@Disabled
public class MathOpModeAH extends OpMode {
    @Override
    public void init() { //required even if empty
    }

    @Override
    public void loop() {

        //variable of type double named stickDifference to show the difference of left stick y and right stick y
        double stickDifference = gamepad1.left_stick_y - gamepad1.right_stick_y;

        //variable of type double named rightStickY to show the value of right stick y
        double rightStickY = gamepad1.right_stick_y;

        //variable of type double named triggerSum to show the sum of the left trigger and the right trigger
        double triggerSum = gamepad1.left_trigger + gamepad1.right_trigger;

        //variable of type boolean named bButton to show the B button
        boolean bButton = gamepad1.b;

        //each telemetry adds data to the driver station for each of their respective variables
        telemetry.addData("Right stick y",rightStickY);
        telemetry.addData("Stick difference", stickDifference);
        telemetry.addData("B button", bButton);
        telemetry.addData("trigger sum", triggerSum);
    }
}
