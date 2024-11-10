package org.firstinspires.ftc.teamcode.test;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
@Autonomous
@Disabled
public class WebcamTest extends LinearOpMode {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam1");
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(webcam, aprilTagProcessor);
        StringBuilder idsFound = new StringBuilder();
        while (opModeInInit()) {
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

            for (AprilTagDetection detection : currentDetections) {
                idsFound.append(detection.id);
                idsFound.append(' ');
                if (detection.id != 0) {
                    Log.d("alan", "new id found: " + detection.id);
                }
            }


        }
        waitForStart();
        telemetry.addData("AprilTags", idsFound);
        while (opModeIsActive()) {
            visionPortal.stopStreaming();
        }
    }
}
