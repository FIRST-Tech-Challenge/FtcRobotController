package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class VariablePractice extends OpMode {
    @Override
    public void init() {
        int teamNumber = 30304;
        double motorSpeed = 0.75;
        boolean clawClosed = true;
        String teamName = "Organized Chaos";
        int motorAngle = 145;

        telemetry.addData("Team Number", teamNumber);
        telemetry.addData("Motor Speed", motorSpeed);
        telemetry.addData("Claw State", clawClosed);
        telemetry.addData("Name", teamName);
        telemetry.addData("Motor Angle", motorAngle);
    }

    @Override
    public void loop() {

    }
}
