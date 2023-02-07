package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;



public class TensorFlowHelper {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String VUFORIA_KEY =
            "ATiC+z7/////AAABmXtA5vl7V0d0gQ4DVekIFZshpexpsey81tveHMKnl6UM/8RhLS9Y46sTtV8cNxkXG73m6igcTqMDrY4lOw0d9h0IvUNC2J5hf2/HfFx7ky9u8KTxh5aBkcTLUgon942jVe1udZoCjWh8k5U3c2Bg0mNiLBS93Y/KuYq9BOteOWfgY3L+igxnD0KnUwY5fhBLKlXR3wNllTnE5Wwhw+NxEzyMi/lzKKRdogcLzNoEGtZ5+pRHJ0JYWxY+n2/KLr6lx3qxz3saTxJNG45SBCUo/li/nLjNYD0oHz7dW5lzTZlLGHRt6AP8bq6lRK7xXZUTeZj/+eqAPqI8l/cM/fa4CmUQ+rqvMYfQcleIfVS8KvsP";

    private VuforiaLocalizer vuforia;

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    private TFObjectDetector tfod;

    public void init(HardwareMap hwMap, Telemetry t){
        hardwareMap = hwMap;
        telemetry = t;

        initVuforia();
        initTfod();
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }


    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0/9.0);
        }
    }



    public String detect(){
        String position = "error";
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());

                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                    double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                    double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                    double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                    telemetry.addData(""," ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                    telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                    telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);

                    if(recognition.getLabel() == "1 Bolt"){
                        position = "left";
                    }else if(recognition.getLabel() == "2 Bulb"){
                        position = "center";
                    }else if(recognition.getLabel() == "3 Panel"){
                        position = "right";
                    }

                    telemetry.addData("Position:",position);



                }
                telemetry.update();
            }

        }

        return position;
    }
}
