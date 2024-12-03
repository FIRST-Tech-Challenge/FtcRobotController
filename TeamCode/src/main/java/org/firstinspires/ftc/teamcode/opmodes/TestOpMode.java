package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}
