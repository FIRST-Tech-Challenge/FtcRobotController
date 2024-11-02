package org.firstinspires.ftc.teamcode.myUtil;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class apriltagLib extends Thread{
    @Override
    public void run(){
        initAprilTag();
        while (opMode.isStarted()){
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            if (currentDetections.size() > 0){
                id = currentDetections.get(0).id;
            }else{
                id = -1;
            }
        }
        visionPortal.close();
    }
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private AprilTagProcessor aprilTag;
    static public int id = -1;

    private VisionPortal visionPortal;
    LinearOpMode opMode;

    public apriltagLib(LinearOpMode opMode){
        this.opMode = opMode;
    }

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }
}
