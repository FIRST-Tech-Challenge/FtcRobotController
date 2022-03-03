package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.util.DelayStorage;

public class DelayRemoveFiveSeconds extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.update();
        waitForStart();
        DelayStorage.subtractSeconds(5);
        telemetry.addLine("Current seconds: " + DelayStorage.seconds);
        telemetry.update();
        while (!isStopRequested()) {}
    }
}