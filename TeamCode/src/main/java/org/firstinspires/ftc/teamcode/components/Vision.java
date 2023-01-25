package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.Autonomous_root;
import org.firstinspires.ftc.teamcode.vision.AutonDetector;
import org.firstinspires.ftc.teamcode.vision.SleeveDetector;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class Vision {

    //Distance sensor
    private DistanceSensor distanceSensor;

    //"Webcam 1"
    private final OpenCvCamera camera;
    private final Telemetry telemetry;
    public Vision(OpenCvCamera openCvCamera, Telemetry t){
        this.camera = openCvCamera;
        this.telemetry = t;
    }

    SleeveDetector sleeveDetector;
    AutonDetector autonDetector;
    AprilTagDetection tagOfInterest = null;

    public void init() {
        sleeveDetector = new SleeveDetector();
        autonDetector = new AutonDetector(telemetry);
        setDetector("sleeve");
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    private String currentDetector = "";
    public void setDetector(String d) {
        if(d.equals("sleeve") && !currentDetector.equals("sleeve")) {
            camera.setPipeline(sleeveDetector);
        } else {
            if(d.equals("pole")) autonDetector.detectMode = AutonDetector.DetectMode.POLE;
            else if(d.equals("cone")) autonDetector.detectMode = AutonDetector.DetectMode.CONE;
            if(!currentDetector.equals("pole") && !currentDetector.equals("cone")) camera.setPipeline(autonDetector);
        }
        currentDetector = d;
    }

    public AutonDetector getAutonPipeline(){
        return autonDetector;
    }

    public void searchTags() {
        ArrayList<AprilTagDetection> currentDetections = sleeveDetector.getLatestDetections();

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
