package org.firstinspires.ftc.teamcode.sensors;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;

public class AprilTagLogi {
    private AprilTagProcessor atProcessor;
    private VisionPortal vp;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private Telemetry telemetry;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        atProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Logi"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(atProcessor);

        vp = builder.build();
    }

    public void update() {
        detectedTags = atProcessor.getDetections();
    }
    public List<AprilTagDetection> getDetectedTags() {return detectedTags;}

    public void displayDetectTelem(AprilTagDetection detectedID) {
        if (detectedID == null) {return;}

        if (detectedID instanceof AprilTagSingleDetection) {
            AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detectedID;

            if (singleDet.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", singleDet.id, singleDet.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedID.ftcPose.x, detectedID.ftcPose.y, detectedID.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedID.ftcPose.pitch, detectedID.ftcPose.roll, detectedID.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedID.ftcPose.range, detectedID.ftcPose.bearing, detectedID.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", singleDet.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", singleDet.center.x, singleDet.center.y));
            }
        } else {
            AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detectedID;
            telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
            telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedID.ftcPose.x, detectedID.ftcPose.y, detectedID.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedID.ftcPose.pitch, detectedID.ftcPose.roll, detectedID.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedID.ftcPose.range, detectedID.ftcPose.bearing, detectedID.ftcPose.elevation));
        }
    }

    public void displayAllDetections() {
        if (detectedTags.isEmpty()) {
            telemetry.addLine("No tags detected");
            return;
        }
        for (AprilTagDetection tag : detectedTags) {
            displayDetectTelem(tag);

        }
    }



    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection detectionID : detectedTags) {
            if (detectionID.id == id) {
                return detectionID;
            }
        }
        return null;
    }

    public void halt() {
        if (vp != null) {
            vp.close();
        }
    }
}