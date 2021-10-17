package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Control extends Subsystem {
    private LinearOpMode opMode;
    private Telemetry telemetry;

    public Control(Robot robot) {
        this.opMode = robot.getOpMode();
        this.telemetry = robot.getTelemetry();
    }
}
