package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp")
public class TeleOp extends OpMode {
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