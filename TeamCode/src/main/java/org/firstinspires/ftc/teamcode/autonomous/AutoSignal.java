package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous
public class AutoSignal extends OpMode {
    static final double FEET_PER_METER = 3.28084;

    // Hardware
    OpenCvWebcam camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    
    public ElapsedTime elapsedTime = new ElapsedTime();
    
    AutoPhase phase = AutoPhase.Waiting;

    // Keep a rolling average of our signal results
    ArrayList<SignalState> signalResults = new ArrayList<>(16);
    // The final result for our signal state
    SignalState signalState = SignalState.Undetermined;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // TODO: If it's practical, these should be adjusted for use with the C270 @ 640x480
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    // TODO: Make sure this is reasonably accurate to the actual signal sleeve
    double tagsize = 0.05;
    
    @Override
    public void init() {
        // Create the OpenCV Webcam and pipeline
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                // Once our camera is running, begin detecting the signal cone
                phase = AutoPhase.DetectSignal;
                telemetry.addLine("Detecting Signal");
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    @Override
    public void init_loop() {
        if (phase == AutoPhase.DetectSignal) {
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            // getDetectionsUpdate will return null if no new frames have been processed since its last call
            if (detections != null) {
                if (detections.size() == 0) {
                    // We don't have any detections
                    signalResults.add(SignalState.Undetermined);
                } else {
                    // TODO: We might see more than one apriltag, so it might be prudent to filter the results
                    
                    if (detections.get(0).id > 2) {
                        telemetry.addLine("An apriltag with an id higher than 2 was found.");
                        return;
                    }
                    
                    // Right now, the id of the apriltag is 1 lower than its int representation in the enum
                    // TODO: This operation can be expensive, but it's not an issue right now
                    signalResults.add(SignalState.values()[detections.get(0).id + 1]);
                }
                
                // If the history gets too high, pop the least recent value
                if (signalResults.size() > 15) {
                    signalResults.remove(0);
                }
            }
        }
    }

    @Override
    public void start() {
        // To be safe, make sure we consider the start to be time 0
        elapsedTime.reset();
        
        // Determine what signal state we are in by using the mode of our history
        int notFound = 0, state1 = 0, state2 = 0, state3 = 0;
        int maxState = 0;
        int maxCount = 0;

        int[] counts = new int[signalResults.size()];

        for (int i = 0; i < signalResults.size(); i++) {
            int num = signalResults.get(i).ordinal();
            counts[num]++;

            if (maxCount < counts[num]) {
                maxCount = counts[num];
                maxState = num;
            }
        }
        
        // TODO: What if the signal is not found, or two states are found in equal amounts?
        signalState = SignalState.values()[maxState];
        telemetry.addLine("Detected signal of " + signalState.name());

        phase = AutoPhase.MoveToSignalZone;
        telemetry.addLine("Moving to Signal Zone");
    }

    @Override
    public void loop() {

    }
}
