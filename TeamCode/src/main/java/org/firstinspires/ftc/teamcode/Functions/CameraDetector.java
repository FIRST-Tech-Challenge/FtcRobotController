package org.firstinspires.ftc.teamcode.Functions;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class CameraDetector {

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    public VuforiaLocalizer vuforia;
    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;
    public WebcamName name;
    public CameraDetector(VuforiaLocalizer _vuforia, TFObjectDetector _tfod, WebcamName _name)
    {
        vuforia = _vuforia;
        tfod = _tfod;
        name = _name;
    }

    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */

    public static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    public static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    public static final String VUFORIA_KEY =
            "AeK2PvT/////AAABmTTCnhmbVUgFg93ZCRG8vUF1RRs5KbLEL7ILBtDXxAX3VJNJBBvkAEB2xBQ6yqr9yOlQWKwHR9mAKBkwOX" +
                    "7SUJJwHIwNimUwhfIpyQ6bsANOY67gAgPxBNXO2TGp7WXAAi+h/JVHjTPBJVMMmah9JURERd1w50biR4+6Mltk44izNVk" +
                    "JuH5RVuxyX2BpK7BlCDsus8/o7280n6CaQeJwqkZwW7WVuzdzyi0JdZL5nmgCHOI65lNQrKKu9ldVA4NBabfk6Lj5kSvd40u" +
                    "e4fUJRzPxPuiSoxgpJ5PFpsmuCPyJN5EOO1EITRSqXvtHZfYChrxIQKjtut+ihbbW8f6y3KeQqpRq5WbQxuQ6cPbuBhJf ";

    /**
     * Initialize the Vuforia localization engine.
     */
    /**
     * Activate TensorFlow Object Detection before we wait for the start command.
     * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
     **/


    public boolean duckIsDetected(TFObjectDetector tfod) {
        boolean isDuckDetected = false;
        if (tfod != null) {
            while (isDuckDetected==false) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    isDuckDetected = false;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                        //Checks to see if the camera detected the duck
                        if (recognition.getLabel().equals("Duck")) {
                            addData(recognition.getLeft(), recognition.getTop(), recognition.getRight(), recognition.getBottom());
                            isDuckDetected = true;
                            telemetry.addData("Object Detected", "Duck");
                        } else {
                            isDuckDetected = false;
                        }
                    }
                }
                telemetry.update();
            }
        }
        return isDuckDetected;
    }
    public class data
    {
        public float left;
        public float top;
        public float right;
        public float bottom;
    }
    public data duckValues = new data();
    public void addData(float _left, float _top, float _right, float _bottom)
    {
        duckValues.left = _left;
        duckValues.top = _top;
        duckValues.right = _right;
        duckValues.bottom = _bottom;
    }
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = name;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    public void initTfod(int tfodMonitorViewId) {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

        if(tfod!=null)
        {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.8, 16.0/9.0);
        }
    }
}