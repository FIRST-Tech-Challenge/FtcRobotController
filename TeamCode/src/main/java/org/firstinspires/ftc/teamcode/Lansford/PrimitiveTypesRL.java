package org.firstinspires.ftc.teamcode.Lansford;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
@Disabled
public class PrimitiveTypesRL extends OpMode {
    @Override
    public void init() {
        int teamNumber = 17348; // int is short for integer.
        double motorSpeed = 0.5; // double is for floating point numbers. It can hold numbers with decimals.
        boolean touchSensorPressed = true;

        telemetry.addData("Team Number", teamNumber);
        telemetry.addData("Motor Speed", motorSpeed);
        telemetry.addData("Touch Sensor", touchSensorPressed);
    }

    @Override
    public void loop() {

    }
}