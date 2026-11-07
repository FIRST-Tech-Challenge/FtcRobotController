package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp
public class TeamPracticeCode extends OpMode {

    Boolean initDone;

    @Override
    public void init() {
        telemetry.addData("init", initDone);
        initDone = true;

    }


   double squarInputWithSign(double input) {
        double output = input * input;

        if (input < 0) {
            output *= -1;
        }
        return output;
   }


    @Override
    public void loop() {
        telemetry.addData("init", initDone);

        double yAxis  = gamepad1.left_stick_y;

        telemetry.addData("Left stick Normal", yAxis);

        yAxis = squarInputWithSign(yAxis);

        telemetry.addData("Left Stick Modified", yAxis);
    }
}
