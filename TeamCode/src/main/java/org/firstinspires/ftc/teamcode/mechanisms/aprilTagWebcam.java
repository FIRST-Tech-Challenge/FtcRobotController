package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

/*@Disabled
public class aprilTagWebcam {
    private static final Logger log = LoggerFactory.getLogger(aprilTagWebcam.class);
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(aprilTagProcessor, true);


    }

    public void update() {
        if (aprilTagProcessor == null) return;

        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        detectedTags.clear();

        if (detections != null) {
            detectedTags.addAll(detections);
        }

    }

 /*   public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedId) {
        if (telemetry == null || detectedId == null) return;

        String tagName = "Unknown";
        if (detectedId.metadata != null && detectedId.metadata.name != null) {
            tagName = detectedId.metadata.name;
        }

        telemetry.addLine(String.format(
                "\n==== (ID %d) %s",
                detectedId.id,
                tagName
        ));

        if (detectedId.ftcPose != null) {
            telemetry.addLine(String.format(
                    "XYZ %6.1f %6.1f %6.1f (inch)",
                    detectedId.ftcPose.x,
                    detectedId.ftcPose.y,
                    detectedId.ftcPose.z
            ));
            telemetry.addLine(String.format(
                    "PRY %6.1f %6.1f %6.1f (deg)",
                    detectedId.ftcPose.pitch,
                    detectedId.ftcPose.roll,
                    detectedId.ftcPose.yaw
            ));
            telemetry.addLine(String.format(
                    "RBE %6.1f %6.1f %6.1f",
                    detectedId.ftcPose.range,
                    detectedId.ftcPose.bearing,
                    detectedId.ftcPose.elevation
            ));
        } else {
            telemetry.addLine("Pose data unavailable");
        }
    }

    public AprilTagDetection getTagByspecificId(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id){
                return detection;
            }
        }
        return null;
    }

    public void stop() {
        if (visionPortal !=null) {
            visionPortal.close();
        }
    }
}



*/