package org.firstinspires.ftc.teamcode.JackBurr.Camera.AprilTags;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvWebcam;

import java.math.RoundingMode;
import java.util.ArrayList;
import java.text.DecimalFormat;
import java.util.List;

@Autonomous
public class AprilTagIDDetector extends OpMode {
    public OpenCvWebcam camera;
    public VisionPortal vp;
    public AprilTagProcessor processor;
    public double x = -999;
    public double y = -999;
    public double z = -999;
    @Override
    public void init() {
        processor = AprilTagProcessor.easyCreateWithDefaults();
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vp = VisionPortal.easyCreateWithDefaults(webcamName, processor);
    }

    @Override
    public void init_loop(){
        ArrayList<AprilTagDetection> detectionsList = processor.getDetections();
        for (AprilTagDetection detection : detectionsList){
            try {
                x = detection.rawPose.x;
                y = detection.rawPose.y;
                z = detection.rawPose.z;
                telemetry.addLine("Tag ID: " + String.valueOf(detection.id));
                if (x != -999) {
                    telemetry.addLine(String.format("%.3f %n", String.valueOf(x)));
                }
                if (y != -999) {
                    telemetry.addLine(String.format("%.3f %n", String.valueOf(y)));
                }
                if (z != -999) {
                    telemetry.addLine(String.format("%.3f %n", String.valueOf(z)));
                }
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
