package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class OldTensorFlowHelper {
    //private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String TFOD_MODEL_ASSET = "TSE2.tflite";
    private static final String[] LABELS = {
            //"Ball",
            //"Cube",
            //"Duck",
            //"Marker"
            "TSE"
    };
    private static final String VUFORIA_KEY =
            "ATiC+z7/////AAABmXtA5vl7V0d0gQ4DVekIFZshpexpsey81tveHMKnl6UM/8RhLS9Y46sTtV8cNxkXG73m6igcTqMDrY4lOw0d9h0IvUNC2J5hf2/HfFx7ky9u8KTxh5aBkcTLUgon942jVe1udZoCjWh8k5U3c2Bg0mNiLBS93Y/KuYq9BOteOWfgY3L+igxnD0KnUwY5fhBLKlXR3wNllTnE5Wwhw+NxEzyMi/lzKKRdogcLzNoEGtZ5+pRHJ0JYWxY+n2/KLr6lx3qxz3saTxJNG45SBCUo/li/nLjNYD0oHz7dW5lzTZlLGHRt6AP8bq6lRK7xXZUTeZj/+eqAPqI8l/cM/fa4CmUQ+rqvMYfQcleIfVS8KvsP";
    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    public final int DUCK_POSITION_RIGHT = 3;
    public final int DUCK_POSITION_CENTER = 2;
    public final int DUCK_POSITION_UNKNOWN = 0;
    public final int DUCK_POSITION_LEFT = 1;

    public int duckPosition;
    public float duckLeft;

    HardwareMap _hardwareMap;

    Telemetry _telemetry;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        _hardwareMap = hardwareMap;
        _telemetry = telemetry;

        initVuforia();
        initTfod();
        activate();
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = _hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = _hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", _hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.40f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.isModelQuantized = false;

        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    public void activate() {
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            //tfod.setClippingMargins(0, 50, 50, 0);
            tfod.setZoom(1, 16.0 / 9.0);
        }
    }

    int tensorFlowDetection() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                _telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    _telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    _telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());

                    //_telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                    //        recognition.getRight(), recognition.getBottom());
                    if (recognition.getLeft() < 150) {
                        duckPosition = DUCK_POSITION_LEFT;
                        duckLeft = recognition.getLeft();
                    } else if (recognition.getLeft() > 350) {
                        duckPosition = DUCK_POSITION_RIGHT;
                        duckLeft = recognition.getLeft();
                    } else {
                        duckPosition = DUCK_POSITION_CENTER;
                        duckLeft = recognition.getLeft();
                    }
                    // middle is left x: 148
                    // right is left x: 385
                    _telemetry.addData("Duck Barcode Position", duckPosition);
                    _telemetry.addData("Duck Left Edge", duckLeft);
                    i++;
                }
                _telemetry.update();
            }
        }
        return duckPosition;
    }

    int tensorFlowMarkerDetection() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                _telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    _telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    _telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());

                    if (recognition.getLeft() > 300) {
                        duckPosition = DUCK_POSITION_RIGHT;
                        duckLeft = recognition.getLeft();
                    } else if (recognition.getLeft() <= 300) {
                        duckPosition = DUCK_POSITION_CENTER;
                        duckLeft = recognition.getLeft();
                    }
                    // middle is left x: 148
                    // right is left x: 385
                    _telemetry.addData("Duck Barcode Position", duckPosition);
                    i++;
                }
                _telemetry.update();
            }
        }
        return duckPosition;
    }


}