package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private Telemetry telemetry;

    private void init(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder() //The following code is there so we can visualize the results better
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "camera"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }
    public void update() {
        detectedTags = aprilTagProcessor.getDetections();

    }
    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;

    }

    public AprilTagDetection getSpecificID (int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection instanceof AprilTagSingleDetection) {
                int tagId = ((AprilTagSingleDetection) detection).id;
                if (tagId == id) {
                    return detection;
                }

            }
        }
        return null;
    }
    public void myIphone(AprilTagDetection detection){
        if (detection == null){return;}
        if (detection instanceof AprilTagSingleDetection) {
            int tagId = ((AprilTagSingleDetection) detection).id;

            telemetry.addData("X: ", detection.ftcPose.x);
            telemetry.addData("Y: ", detection.ftcPose.y);
            telemetry.addData("Z: ", detection.ftcPose.z);
            telemetry.addData("ID: ", tagId);
            telemetry.update();
        }
    }
    public void stopStream() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
