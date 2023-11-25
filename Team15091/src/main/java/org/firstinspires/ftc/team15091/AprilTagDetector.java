package org.firstinspires.ftc.team15091;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;


public class AprilTagDetector {
    public AprilTagProcessor aprilTag;
    public void init () {
        aprilTag = new AprilTagProcessor.Builder().build();
    }

    public List<AprilTagDetection> scanForAprilTags () {
        return aprilTag.getDetections();
    }
    public AprilTagDetection scanForAprilTagById (int id) {
        List<AprilTagDetection> aprilTagDetections = scanForAprilTags();
        for (AprilTagDetection currentAprilTag : aprilTagDetections) {
            if (currentAprilTag.id == id) {
                return currentAprilTag;
            }
        }
        return null;
    }
}
