package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.util.DelayStorage;

public class DelayRemoveFiveSeconds extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Warning: doing this while queued BREAKS TOURNAMENT RULES!");
        telemetry.addLine("DO NOT RUN THIS WHILE QUEUED.");
        telemetry.update();
        waitForStart();
        DelayStorage.subtractSeconds(5);
        telemetry.addLine("Current seconds: " + DelayStorage.seconds);
        telemetry.update();
        while (!isStopRequested()) {}
    }
}