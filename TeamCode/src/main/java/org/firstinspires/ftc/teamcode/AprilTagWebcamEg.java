package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.navigate.AprilTagWebcam;

@Autonomous(name = "AprilTagWebcamEg", group = "Test")
public class AprilTagWebcamEg extends OpMode {

    private AprilTagWebcam aprilTagWebcam;

    @Override
    public void init() {
        aprilTagWebcam = new AprilTagWebcam();
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        boolean tag_outcome = aprilTagWebcam.updateDetections();
    }
}
