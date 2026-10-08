package org.firstinspires.ftc.teamcode.sensors;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam #1"));
        builder.setCameraResolution(new Size(640, 480));
        
        /*MJEPG uses less USB bandwidth,so I added this line cuz that means higher quality
        and steadier framerate: tags detct faster and less lag*/
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG); 
        
        builder.addProcessor(atProcessor);

        vp = builder.build();
    }

    public void update() {
        detectedTags = atProcessor.getDetections();
    }

    public void displayDetectTelem(AprilTagDetection detectedID) {
        if (detectedID == null) { return; }

        telemetry.addLine(String.format("ID %d (%s)", detectedID.id,
                detectedID.metadata != null ? detectedID.metadata.name : "Unknown"));

        if (detectedID.metadata != null) {
            telemetry.addData("Range (cm)", "%.1f", detectedID.ftcPose.range);
            telemetry.addData("Bearing (deg)", "%.1f", detectedID.ftcPose.bearing);
            telemetry.addData("Yaw (deg)", "%.1f", detectedID.ftcPose.yaw);
            telemetry.addData("X (cm)", "%.1f", detectedID.ftcPose.x);
            telemetry.addData("Y (cm)", "%.1f", detectedID.ftcPose.y);
            telemetry.addData("Z (cm)", "%.1f", detectedID.ftcPose.z);
        } else {
            telemetry.addLine("(No metadata — unknown tag, pose unavailable)");
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

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public AprilTagDetection getTagById(int id) { //small edit ig
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
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
