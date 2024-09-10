package org.firstinspires.ftc.teamcode.JackBurr.Camera.AprilTags;

import static org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getCurrentGameTagLibrary;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@Autonomous
public class AprilTagIDDetector extends OpMode {
    private AprilTagProcessor aprilTagProcessor;
    public int aprilTagsCount = 0;
    private VisionPortal visionPortal;
    public WebcamName webcamName;
    public StringBuilder idsFound;
    public ElapsedTime main_timer;
    List<AprilTagDetection> currentDetections;
    public double poseX = 0;
    public boolean xDetected = false;
    public double poseY = 0;
    public boolean yDetected = false;
    public double poseZ = 0;
    public boolean zDetected = false;

    @Override
    public void init() {
        webcamName = getWebcamName("Webcam 1");
        aprilTagProcessor = createAprilTagProcessor();
        visionPortal = createVisionPortal(webcamName, aprilTagProcessor);
        main_timer = getTimer();
        idsFound = createStringBuilder();
        telemetry.addLine(String.valueOf(getCurrentGameTagLibrary()));
    }

    @Override
    public void init_loop(){
        currentDetections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : currentDetections){
            telemetry.addLine(String.valueOf(detection.id));
            try {
                poseX = detection.rawPose.x;
                poseY = detection.rawPose.y;
                poseZ = detection.rawPose.z;
            }
            catch (Exception e){
                telemetry.addLine(e.toString());
            }
        }
        telemetry.addData("AprilTag Count: ", aprilTagsCount);
        telemetry.addData("Time: " , main_timer.time());
        telemetry.addData("Detection X: ",poseX);
        telemetry.addData("Detection Y: ",poseY);
        telemetry.addData("Detection Z: ",poseZ);
        telemetry.update();
    }
    @Override
    public void loop() {
        visionPortal.stopStreaming();
    }

    public WebcamName getWebcamName(String webcam_name){
        return hardwareMap.get(WebcamName.class, webcam_name);
    }
    public AprilTagProcessor createAprilTagProcessor(){
        return AprilTagProcessor.easyCreateWithDefaults();
    }
    public VisionPortal createVisionPortal(WebcamName webcamName, AprilTagProcessor processor){
        return VisionPortal.easyCreateWithDefaults(webcamName, processor);
    }
    public StringBuilder createStringBuilder(){
        return new StringBuilder();
    }
    public ElapsedTime getTimer(){
        return new ElapsedTime();
    }
}