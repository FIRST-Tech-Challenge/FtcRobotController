package org.firstinspires.ftc.teamcode.Testing.Helper_test;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public class doubleVision extends LinearOpMode {

    public static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    public VisionPortal visionPortal;               // Used to manage the video source for AprilTags.
    public AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    public TfodProcessor tfod;                      // Used for managing TensorFlow detection
    public final String TFOD_MODEL_FILE = "model_20231110_182709.tflite";
    private static final String[] LABELS = {
            "red"
    };


    // ---------------------------------------------
    // Initialize AprilTag and TensorFlow processors
    // ---------------------------------------------
    public void initDoubleVision() {

        // ----------------------
        // AprilTag Configuration
        // ----------------------
        aprilTag = new AprilTagProcessor.Builder()

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // ------------------
        // TFOD Configuration
        // ------------------
        tfod = new TfodProcessor.Builder()

                // Sets the file name for TensorFlow detection
                .setModelFileName(TFOD_MODEL_FILE)

                // Sets the labels for TensorFlow detection
                .setModelLabels(LABELS)

                .build();

        // --------------------------
        // VisionPortal Configuration
        // --------------------------
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessors(tfod, aprilTag)
                .build();


    } // end initDoubleVision()
    @Override
    public void runOpMode() {
    }
}
