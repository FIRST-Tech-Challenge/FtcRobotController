package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.sensors.AprilTagLogi;

@TeleOp(name = "AprilTag Test", group = "Test")
public class AprilTagTest extends LinearOpMode {

    private final AprilTagLogi aprilTag = new AprilTagLogi();

    @Override
    public void runOpMode() {
        aprilTag.init(hardwareMap, telemetry);

        telemetry.addLine("AprilTag initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            aprilTag.update();
            aprilTag.displayAllDetections();
            telemetry.update();
        }

        aprilTag.halt();
    }
}