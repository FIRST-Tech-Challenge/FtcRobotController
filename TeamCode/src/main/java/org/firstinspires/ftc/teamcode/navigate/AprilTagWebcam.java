package org.firstinspires.ftc.teamcode.navigate;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class AprilTagWebcam {

    private  AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private final List<AprilTagDetection> detectedTags = new ArrayList<>();

    public double x_value;
    public double y_value;
    public double percent_value;

    public boolean tagDetected = false;

    private String target_tag;

    private Telemetry telemetry;

    public boolean updateDetections(){
        x_value = 0;
        List<AprilTagDetection> newDetections = aprilTagProcessor.getDetections();
        boolean hasDetections = !newDetections.isEmpty();
        boolean preferred_detected = false;

        detectedTags.clear();
        detectedTags.addAll(newDetections);

        telemetry.clearAll();
        telemetry.addData("AprilTag found", hasDetections ? "Y" : "N");
        tagDetected = hasDetections;
        if (hasDetections) {
            preferred_detected = renderDetections();
        }
        telemetry.update();
        if (target_tag == null || target_tag.isEmpty()) {
            return hasDetections;
        }
        else{
            return preferred_detected;
        }

    }


    public void init(HardwareMap hwMap, Telemetry telemetry, String target_tag ){
        this.telemetry = telemetry;
        tagDetected = false;
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder vpBuilder= new VisionPortal.Builder();
        vpBuilder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        vpBuilder.setCameraResolution(new Size(640, 480));
        vpBuilder.addProcessor(aprilTagProcessor);
        visionPortal = vpBuilder.build();
        this.target_tag = target_tag;

    }

    public void init(HardwareMap hwMap, Telemetry telemetry ){
        this.telemetry = telemetry;
        tagDetected = false;
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder vpBuilder= new VisionPortal.Builder();
        vpBuilder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        vpBuilder.setCameraResolution(new Size(640, 480));
        vpBuilder.addProcessor(aprilTagProcessor);
        visionPortal = vpBuilder.build();
        this.target_tag = "";

    }

    public List<AprilTagDetection> getDetectedTags(){
        return detectedTags;
    }

    /**
     * Check if tags are currently being tracked (within timeout window)
     */
    public boolean hasActiveTags() {
        return !detectedTags.isEmpty();
    }

    public AprilTagDetection getTagById(){
        return detectedTags.isEmpty() ? null : detectedTags.get(0);
    }

    private boolean renderDetections() {
        boolean preferred_detected = false;
        for (AprilTagDetection detection : detectedTags) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                if (singleDet.metadata != null) {
                    telemetry.addData("ID", singleDet.id);
                    if (singleDet.metadata.name.equals(target_tag)) {
                        telemetry.addData("Target Tag Found", "Y");
                        preferred_detected = true;
                    } else {
                        telemetry.addData("Target Tag Found", "N");
                    }
                    telemetry.addLine(String.format(Locale.US, "\n==== (ID %d) %s", singleDet.id, singleDet.metadata.name));
                    telemetry.addLine(String.format(Locale.US, "XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format(Locale.US, "PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format(Locale.US, "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    x_value = detection.ftcPose.x;
                } else {
                    telemetry.addData("ID", singleDet.id);
                    telemetry.addLine(String.format(Locale.US, "\n==== (ID %d) Unknown", singleDet.id));
                    telemetry.addLine(String.format(Locale.US, "Center %6.0f %6.0f   (pixels)", singleDet.center.x, singleDet.center.y));
                }
            } else {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                x_value = detection.ftcPose.x;
                telemetry.addData("ID", "N/A (cluster)");
                telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                if (clusterDet.metadata.name.equals(target_tag)) {
                    telemetry.addData("Target Tag Found", "Y");
                    preferred_detected = true;
                } else {
                    telemetry.addData("Target Tag Found", "N");
                }
                x_value = detection.ftcPose.x;
                y_value = detection.ftcPose.y;
                percent_value = clusterDet.percentClusterFound;
                telemetry.addLine(String.format(Locale.US, "Percent tags found: %d", clusterDet.percentClusterFound));
                telemetry.addLine(String.format(Locale.US, "XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format(Locale.US, "PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format(Locale.US, "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            }
        }
        return preferred_detected;
    }
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

}


