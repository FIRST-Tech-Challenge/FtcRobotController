package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.*;

// LiveView refers only to the Robot Controller preview (example shown above). It’s completely separate from Driver Station Camera Stream, which still operates normally even if LiveView is stopped (manually or automatically).
@Autonomous(group="Concept")
public class Visionportal1 extends LinearOpMode {

    private TfodProcessor myTfodProcessor;
    private AprilTagProcessor myAprilTagProcessor;
    private VisionPortal myVisionPortal;

    private void initProcessors() {
        // Tfod = tensorflow object detection
        myTfodProcessor = new TfodProcessor.Builder()
                .setMaxNumRecognitions(10) // max # recognitions
                .setUseObjectTracker(true) // use object tracker
                .setTrackerMaxOverlap((float) 0.2) // max % of box overlapped by another box for recognition
                .setTrackerMinSize(16) // minimum size of a tracked/recognized object (units?)
                .build();

        myAprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()) // currentgametaglibrary = centerstage + sample apriltags
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true) // default = false
                .setDrawCubeProjection(true) // default = false
                .build();

        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // change name depending on mech team wants to name it
                .addProcessor(myAprilTagProcessor) // add apriltag processor
                .addProcessor(myTfodProcessor) // add tfod processor
                .setCameraResolution(new Size(640, 480)) // import android.util.Size;
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG) // changed to MJPEG instead of YUY2 since it uses less bandwidth
                // .enableCameraMonitoring(true) // enableCameraMonitoring not identified? I also didn't find any related methods in VisionPortal.java
                .enableLiveView(true) // manually added this method because it was the closest thing to enableCameraMonitoring
                .setAutoStopLiveView(true)
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initProcessors();
        waitForStart();

        if (!isStopRequested()) {
            while(opModeIsActive()) {

                telemetry.update();
            }
        }
        myVisionPortal.setProcessorEnabled(myTfodProcessor,false);
        myVisionPortal.setProcessorEnabled(myAprilTagProcessor,false);
        myVisionPortal.close();

    }

}
