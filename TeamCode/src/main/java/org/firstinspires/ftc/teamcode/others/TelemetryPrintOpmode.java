package org.firstinspires.ftc.teamcode.others;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public class TelemetryPrintOpmode extends OpMode {


    @Override
    public void init() {
        telemetry.addData("hello", "world!");

    }
    @Override
    public void loop() {

    }
}
