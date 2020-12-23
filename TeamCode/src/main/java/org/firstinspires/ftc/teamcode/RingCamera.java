package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class RingCamera {
    //This class contains the camera
    //This class will include methods for recognizing the number of rings on the field

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    int i = 0;

    private static final String VUFORIA_KEY =
            "ATL0ncP/////AAABmT2pP6X1AUWloxNJ52z7pHEOjunJDTCB7rFvK9R38fBtenmamHFRo1IJFI/H+TzU90tguYTW4L/I0g2wBlqld9RzXISaItld+tNXARVeJUVn1GEESaZa8dG9bn9POCtVrfAysAHVgXrpzSvVI+AS+3tCxcDaVJtRbStvRaL/62OeNleqMRHJpRU8026f5odx6L0/toQbfvpWV9HUU1/naUwOhBRxeh+8yuVHVuGFOxii3qmHewAB3955neBrABR8eb8Llh9LbYg1ZCGNBDNKbqa762bGQzEPPijQTJMNcJGRbNAQZP/mPMOJGFLz9gXRByPVLzUWpjEtbsjQBohKWIKdNKzo0+vgpgcTTXWB+a5A";

    HardwareMap hwMap = null;

    /* Constructor */
    public RingCamera() {

    }
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    /**
     * Initialize the Vuforia localization engine.
     */

    //Init VuForia and TFOD
    public void init(HardwareMap ahwMap) {
        initVuforia(ahwMap);
        initTfod(ahwMap);

        if (tfod != null) {
            tfod.activate();
        }
    }

    private void initVuforia(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    /**
     * This method returns the integer number of rings the TFOD detects in its vision at the time it is called.
     * @return Number of rings
     */
    public int ringCount() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                String count = "";
                for (Recognition recognition : updatedRecognitions) {
                    count = recognition.getLabel();
                    if (count.equals("Quad")) {
                        i = 4;
                    } else if (count.equals("Single")) {
                        i = 1;
                    }
                }
                if (count.equals("")) {
                    i = 0;
                }
            }
        }
        return i;
    }
}
