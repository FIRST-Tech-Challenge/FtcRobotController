package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.vision.PoleDetector;
import org.firstinspires.ftc.teamcode.vision.SleeveDetector;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class Vision {

    //Distance sensor
    private DistanceSensor distanceSensor;

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

    public void init() {
        // Lens intrinsics
        // NOTE: this calibration is for the C920 webcam at 800x448.
        final double tagsize = 0.166;
        final double fx = 578.272;
        final double fy = 578.272;
        final double cx = 402.145;
        final double cy = 221.506;
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

    public void setDetector(String d) {
        //if(d == "pole") camera.setPipeline(new AutonDetector(telemetry));
        poleDetector = new PoleDetector(telemetry);
        if(d == "pole") camera.setPipeline(poleDetector);
    }

    public double differenceX(){
        return poleDetector.differenceX();
    }

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

//    public double distance() {
//        return distanceSensor.getDistance(DistanceUnit.MM);
//    }
}
