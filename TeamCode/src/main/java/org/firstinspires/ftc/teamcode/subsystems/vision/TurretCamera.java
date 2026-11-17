package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Collections;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.common.HardwareNames;
import org.firstinspires.ftc.teamcode.common.Subsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.*;

public class TurretCamera extends Subsystem {
    private AprilTagProcessor aprilTags;
    private VisionPortal portal;
    private List<AprilTagDetection> detections = Collections.emptyList();
    public TurretCamera(HardwareMap hardwareMap) {
        WebcamName camera = hardwareMap.tryGet(WebcamName.class, HardwareNames.hive_camera);
        if (camera == null) return;
        aprilTags = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES).build();
        portal = new VisionPortal.Builder().setCamera(camera)
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTags).build();
    }
    public void periodic() {
        detections = Collections.emptyList();
        if (portal == null) return;
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) return;
        detections = aprilTags.getDetections();
    }

    public void writeTelemetry(Telemetry telemetry) {
        telemetry.addData("Camera", portal == null ? "MISSING" : portal.getCameraState());
        telemetry.addData("Detections (unfiltered)", detections.size());
        for (AprilTagDetection detection : detections) {
            double ageMs = (System.nanoTime() - detection.frameAcquisitionNanoTime) / 1e6;
            telemetry.addData("Observation age ms", ageMs);
            if (ageMs > 200 || !(detection instanceof AprilTagClusterDetection)) continue;
            AprilTagClusterDetection cell = (AprilTagClusterDetection) detection;
            telemetry.addData("Fresh cell", cell.metadata.shortName);
            telemetry.addData("Visible tags %", cell.percentClusterFound);
            if (cell.ftcPose == null) continue;
            telemetry.addData("Range in / bearing deg / roll deg", "%.1f / %.1f / %.1f",
                    cell.ftcPose.range, cell.ftcPose.bearing, cell.ftcPose.roll);
            }
    }
    public void shutdown(){
        if (portal != null) {
            portal.close();
            portal = null;
        }
    }
    }
