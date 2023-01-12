package org.firstinspires.ftc.teamcode.components;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.SleeveDetector;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class Vision {

    //"Webcam 1"
    private static OpenCvCamera camera;

    public Vision(OpenCvCamera openCvCamera){
        this.camera = openCvCamera;
    }

    static SleeveDetector aprilTagDetectionPipeline;

    static AprilTagDetection tagOfInterest = null;

    //c&p :(
    //Convert from the counts per revolution of the encoder to counts per inch
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 2-0.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;

    // Lens intrinsics
    // NOTE: this calibration is for the C920 webcam at 800x448.
    static final double fx = 578.272;
    static final double fy = 578.272;
    static final double cx = 402.145;
    static final double cy = 221.506;

    // UNITS ARE METERS
    static double tagsize = 0.166;

    public static void init() {
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

    public static void searchTags() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        for(AprilTagDetection tag : currentDetections) {
            if(tag.id == 1 || tag.id == 2 || tag.id == 3) {
                tagOfInterest = tag;
                break;
            }
        }
    }

    public static int tagId() {
        if(tagOfInterest != null) return tagOfInterest.id;
        else return -1;
    }
}
