package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeamMember extends OpMode {
    boolean initDone;

    @Override
    public void init() {
        telemetry.addData("Init", initDone);
        initDone = true
    }

    double squareInputWithSign(double input) {
        double output = input * input

        if (input < 0) {
            output *= -1
        }
        return output;
    }

    @Override
    public void loop() {
        telemetry.addData("Init", initDone);

        double yAxis = gamepad1.left_stick_y;

        telemetry.addData("Left Stick Normal", yAxis);

        yAxis = squareInputWithSign(yAxis);
        telemetry.addData("Left Stick Modified", yAxis);

    }
}
