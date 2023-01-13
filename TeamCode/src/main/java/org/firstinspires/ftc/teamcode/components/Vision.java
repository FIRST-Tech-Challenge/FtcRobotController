package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.PoleDetector;
import org.firstinspires.ftc.teamcode.vision.SleeveDetector;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class Vision {

    //"Webcam 1"
    private OpenCvCamera camera;
    private Telemetry telemetry;

    public Vision(OpenCvCamera openCvCamera, Telemetry t){
        this.camera = openCvCamera;
        this.telemetry = t;
    }

    SleeveDetector aprilTagDetectionPipeline;
    PoleDetector poleDetector;

    AprilTagDetection tagOfInterest = null;

    //c&p :(
    //Convert from the counts per revolution of the encoder to counts per inch
    final double HD_COUNTS_PER_REV = 28;
    final double DRIVE_GEAR_REDUCTION = 2-0.15293;
    final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;

    // Lens intrinsics
    // NOTE: this calibration is for the C920 webcam at 800x448.
    final double fx = 578.272;
    final double fy = 578.272;
    final double cx = 402.145;
    final double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    public void init() {
        aprilTagDetectionPipeline = new SleeveDetector(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    public void setPoleDetector() {
        poleDetector = new PoleDetector(telemetry);
        camera.setPipeline(poleDetector);
    }

    public int differenceX() { return poleDetector.differenceX(); }

    public void searchTags() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        for(AprilTagDetection tag : currentDetections) {
            if(tag.id == 1 || tag.id == 2 || tag.id == 3) {
                tagOfInterest = tag;
                break;
            }
        }
    }

    public int tagId() {
        if(tagOfInterest != null) return tagOfInterest.id;
        else return -1;
    }
}
