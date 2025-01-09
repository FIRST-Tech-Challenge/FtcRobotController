package org.firstinspires.ftc.teamcode.JackBurr.Camera.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.ArrayList;
import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Autonomous
public class ExampleAprilTagDetection extends OpMode {
    private AprilTagProcessor aprilTagProcessor;
    public int aprilTagsCount = 0;
    private VisionPortal visionPortal;
    public WebcamName webcamName;
    public StringBuilder idsFound;
    public ElapsedTime main_timer;
    List <String> idsList = new ArrayList<>();
    List<AprilTagDetection> currentDetections;
    public String currentPosition = "UNDEFINED";
    public String aprilTagID = "NOT_YET_DETECTED";
    public double xOffset;
    Mat frame = new Mat();
    double poseX = -99756478857656756457.0;
    double poseY = -99756478857656756457.0;
    double poseZ = -99756478857656756457.0;
    double poseAX = -99756478857656756457.0;
    double poseAY = -99756478857656756457.0;
    double poseAZ = -99756478857656756457.0;

    @Override
    public void init() {
        System.out.println("Hello World");
        webcamName = getWebcamName("Webcam 1");
        aprilTagProcessor = createAprilTagProcessor();
        visionPortal = createVisionPortal(webcamName, aprilTagProcessor);
        main_timer = getTimer();
        idsFound = createStringBuilder();
    }

    @Override
    public void init_loop(){
        currentDetections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : currentDetections){
            telemetry.addLine(String.valueOf(detection.id));
            poseX = detection.rawPose.x;
            poseY = detection.rawPose.y;
            poseZ = detection.rawPose.z;
            telemetry.addLine("FTCPose X: " + String.valueOf(detection.ftcPose.x));
            if (idsList.contains(String.valueOf(detection.id)) == false) {
                aprilTagID = String.valueOf(detection.id);
                aprilTagsCount = aprilTagsCount + 1;
                idsList.add(String.valueOf(detection.id));
                telemetry.addLine(String.valueOf(detection.center));
            }
        }
        telemetry.addData("AprilTags found: ", idsList);
        telemetry.addData("AprilTag Count: ", aprilTagsCount);
        telemetry.addData("Time: " , main_timer.time());
        if (poseX != -99756478857656756457.0){
            telemetry.addData("Detection X: ",poseX);
        }
        if (poseY != -99756478857656756457.0){
            telemetry.addData("Detection Y: ",poseY);
        }
        if (poseZ != -99756478857656756457.0){
            telemetry.addData("Detection Z: ",poseZ);
        }
        telemetry.addLine(String.valueOf(xOffset));
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
