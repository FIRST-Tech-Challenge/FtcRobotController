package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Control extends MinorSubsystem {

    public Control(Robot robot) {
        this.opMode = robot.getOpMode();
        this.telemetry = robot.getTelemetry();
        this.timer = robot.getTimer();
    }
}
