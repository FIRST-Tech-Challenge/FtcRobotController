package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp
public class VariablesPractice extends OpMode {
    @Override
    public void init() {
        int teamNumber = 32095;
        double motorSpeed = 0.75;
        boolean clawClosed = true;

        telemetry.addData("Team Number", teamNumber);
        telemetry.addData("motor Speed", motorSpeed);
        telemetry.addData("claw Closed", clawClosed);
    }

    @Override
    public void loop() {

    }
}
