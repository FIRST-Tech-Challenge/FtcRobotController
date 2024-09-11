package org.firstinspires.ftc.teamcode.JackBurr.Camera.AprilTags;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class AprilTagIDDetector extends OpMode {
    public OpenCvWebcam camera;
    public VisionPortal vp;
    public AprilTagProcessor processor;
    public double x = -999;
    public double y = -999;
    public double z = -999;
    public double roundX;
    public double roundY;
    public double roundZ;
    @Override
    public void init() {
        processor = AprilTagProcessor.easyCreateWithDefaults();
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        VisionPortal vp = VisionPortal.easyCreateWithDefaults(webcamName, processor);
    }

    @Override
    public void init_loop(){
        ArrayList<AprilTagDetection> detectionsList = processor.getDetections();
        for (AprilTagDetection detection : detectionsList){
            try {
                x = detection.rawPose.x;
                y = detection.rawPose.y;
                z = detection.rawPose.z;
                roundX = Math.round(x*100)/100;
                roundY = Math.round(y*100)/100;
                roundZ = Math.round(z*100)/100;
                telemetry.addLine("Tag ID: " + String.valueOf(detection.id));
                telemetry.addLine(String.valueOf(roundX));
                telemetry.addLine(String.valueOf(roundY));
                telemetry.addLine(String.valueOf(roundZ));
            }
            catch (Exception e) {
                telemetry.addLine(e.getMessage());
            }
        }
    }
    @Override
    public void loop() {
        vp.stopStreaming();
    }


}
