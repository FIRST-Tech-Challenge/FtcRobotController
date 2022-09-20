package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.components.Vuforia.CameraChoice;
import org.firstinspires.ftc.teamcode.params.VuforiaParams;

/**
 * Creates a Tensorflow Object Detector which can detect certain game objects
 */
public class Tensorflow {
    // Labels of the game objects given
    private static final String TFOD_MODEL_ASSET = "INSERT_NAME_HERE";
    private static final String LABEL_FIRST_ELEMENT = "INSERT_NAME_HERE";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    /**
     * Creates and initializes a new Tensorflow Object Detector
     * @param name name in hardware map of desired webcam
     * @param tfodMonitorId
     */
    public Tensorflow(WebcamName name, int tfodMonitorId) {
        // Initialize Vuforia parameters
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey           = BuildConfig.VUFORIA_KEY;
        parameters.cameraName                  = name;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorId);
        // Probability of > minimumConfidence = object identified
        // Increase if too many false item detections
        tfodParameters.minimumConfidence = 0.3;
        // Creates TFObjectDetector
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        // Load models to identify (NOTE: CAN ADD MORE)
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT);
        // Start recognizing those objects
        tfod.activate();
    }

    /**
     * Gets the recognitions that have been detected if updated since last check
     * @return list of recognitions
     */
    public List<Recognition> getInference() {
        if (tfod != null) {
            return tfod.getUpdatedRecognitions();
        }
        return null;
    }

    /** Activates TFObjectDetector to start recognizing objects
     */
    public void activate() {
        tfod.activate();
    }

    /** Cleanup after shutdown so that unnecessary resources aren't used
     */
    public void shutdown() {
        tfod.shutdown();
    }
}