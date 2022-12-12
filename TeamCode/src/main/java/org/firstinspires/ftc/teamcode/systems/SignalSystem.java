package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.SignalState;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Random;

/**
 * Manages the connection to the webcam used for signal detection,
 * and evaluates the images from it to determine the signal state.
 */
public class SignalSystem extends SubsystemBase {
    /** How long the signal state history should be. */
    static final int SIGNAL_HISTORY_SIZE = 15;

    // Lens intrinsics
    // Pixels
    // For the C920 @ 800x448
    // TODO: If it's practical, these should be adjusted for use with the C270 @ 640x480
    static final double fx = 578.272;
    static final double fy = 578.272;
    static final double cx = 402.145;
    static final double cy = 221.506;

    // Meters
    // TODO: Make sure this is reasonably accurate to the actual signal sleeve
    static final double TAG_SIZE = 0.05;
    
    OpenCvWebcam camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    
    /** A rolling history of the detected signal state. The int values correspond to the SignalState enum. */
    volatile ArrayList<Integer> signalHistory;
    
    /** Whether the system is active and getting results. */
    public volatile boolean active = false;
    
    volatile boolean resultReady = false;

    /**
     * Initializes the subsystem. It does not open a connection to the camera.
     */
    public SignalSystem(final HardwareMap hardwareMap, final String name) {
        // Create the OpenCV Webcam and pipeline
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, name));
        
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(TAG_SIZE, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);

        signalHistory = new ArrayList<>(SIGNAL_HISTORY_SIZE + 1);
    }

    @Override
    public void periodic() {
        // We don't do anything if we aren't active
        if (!active) return;
        
        // Get the detections from the camera
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
        
        // If no new frames have been processed, then detections will be null
        if (detections == null) return;
        
        if (detections.size() > 0) {
            // TODO: What if there is more than one apriltag detected? Some filtering might be needed
            // The id of the apriltag is different from its representation as an enum
            int detected = detections.get(0).id + 1;

            signalHistory.add(detected);
        } else {
            // We did not detect any apriltags
            signalHistory.add(0);
        }

        // If we are at the max size of the history, remove the oldest element to make room
        if (signalHistory.size() >= SIGNAL_HISTORY_SIZE) {
            signalHistory.remove(0);
            
            // We also have a full history, so we can make an accurate judgement
            // TODO: Under certain conditions, like multiple times without detection, we should reset the history
            resultReady = true;
        }
    }

    /**
     * Creates a connection to the camera and begins processing images from it.
     * @return Whether the camera was successfully connected to.
     */
    public boolean startCamera() {
        if (active) {
            // We can't open a camera twice
            return false;
        }

        resultReady = false;
        
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);

                // Reset the signal state history
                // We will add one more element before removing the first, so the capacity needs to be 1 higher
                signalHistory = new ArrayList<>(SIGNAL_HISTORY_SIZE + 1);

                active = true;
            }

            @Override
            public void onError(int errorCode) {
                
            }
        });
        
        return true;
    }

    /**
     * Stops processing images from the camera and closes the connection to it.
     * @return Whether the camera was successfully closed.
     */
    public boolean stopCamera() {
        if (!active) {
            // We can't close a camera twice
            return false;
        }
        
        camera.closeCameraDeviceAsync(() -> {});

        active = false;

        return true;
    }
    
    public boolean isResultReady() {
        return resultReady;
    }

    /**
     * Returns the signal state as determined by the system.
     * @throws IllegalStateException If the function is called while resultReady is false.
     */
    public SignalState GetResult() throws IllegalStateException {
        // Make sure we have a result to give
        if (!resultReady) {
            // FIXME: this exception is being thrown when isResultReady returns true. Might be a thread issue
            throw new IllegalStateException("GetResult was called while resultReady was false.");
        }
        
        // Find the most common (mode) signal state from the history
        int[] counts = new int[4];
        int maxCount = 0;
        int maxState = 0;

        // TODO: What if multiple states occur the same number of times?
        for (int i = 0; i < signalHistory.size(); i++) {
            int value = signalHistory.get(i);
            
            counts[value]++;
            
            if (counts[value] >= maxCount) {
                maxCount = counts[value];
                maxState = value;
            }
        }

        // For debugging, we want to know when somethings gone wrong
        return SignalState.values()[maxState];
        
        /*// TODO: Come back to this
        // If our result was Unidentified, then pick a random state at random because we cannot lose points for a wrong guess
        int result;
        
        if (maxState == 0) {
            // Random integer from [0,3]
            result = new Random().nextInt(4);
        } else {
            result = maxState;
        }
        
        return SignalState.values()[result];*/
    }
}
