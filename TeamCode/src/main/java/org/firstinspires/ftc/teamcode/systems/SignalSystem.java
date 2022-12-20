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
    
    private boolean active = false;
    
    private SignalState state = SignalState.Undetermined;

    /**
     * Initializes the subsystem. It does not open a connection to the camera.
     */
    public SignalSystem(final HardwareMap hardwareMap, final String name) {
        // Create the OpenCV Webcam and pipeline
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, name));
        
        FtcDashboard.getInstance().startCameraStream(camera, 15);
        
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(TAG_SIZE, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
    }

    @Override
    public void periodic() {
        // We don't do anything if we aren't active
        if (!active) return;
        
        // Get the detections from the camera
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();
        
        if (detections == null || detections.size() == 0) {
            return;
        }
        
        // The enum's indices are shifted one higher than the id's of the apriltags
        state = SignalState.values()[detections.get(0).id + 1];
    }

    /**
     * Creates a connection to the camera and begins processing images from it.
     * @return Whether the camera was successfully connected to.
     */
    public boolean startCamera() {
        if (active) {
            // Camera is already open
            return false;
        }
        
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

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
            // Camera is already closed
            return false;
        }
        
        camera.closeCameraDeviceAsync(() -> {});

        active = false;

        return true;
    }

    /**
     * Returns the signal state as determined by the system.
     */
    public SignalState GetResult() {
        return state;
    }
}
