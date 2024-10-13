package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

/** @noinspection ALL */
@TeleOp(name = "TeleOp")
public class DriverMode extends OpMode {
    private Hardware hardware;

    @Override
    public void init() {
        hardware = new Hardware(this);
    }

    @Override
    public void loop() {
        // Update the information from the robot
        telemetry.update();
    }
}