/*package org.firstinspires.ftc.teamcode;


//import astatements
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


public class Duckdetection {

    //declares label and models for TensorFlow Object Detection
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    //Vuforia key necessary for operation
    //may or may not be from rover ruckus season
    private static final String VUFORIA_KEY =
            "AeNvkLn/////AAABmdnuZ8WnxkB4gq/UchazF7Jhi0jxEt/8ogNMFqNQ1cnklNl2EcvV3Yv+uRdtuYUcMnx8HKeZ3UPMwvLEyY8I+a7q21AQXiIFqHT+WZzqe5/e7HPzARhbiZ5jg5hLtx8V8bKby/HjJcZt26/pvQ6XlbhFK1AH3nAs3R9GHFb6GJOxVvlYmAtrWKIyx4x/Au67uyiFytK5gWG666XrNrbvpuj8daNMrQ6xyuKvtIAPJ8/PuOVWfKL0S7Kn0VP8EAcOQV89O7WSW+35Ufqoh8Xjlfi3vEuvIne1RgfY4Q1ZXmKn43Sd1xyYubFmIRWf6QSuHfsskxKa+s4xq1LsWE7jmkvEbAstBQzhcvoRneWoq4VD";

    //Declares Vuforia
    private VuforiaLocalizer vuforia;

    //Declares TensorFlow
    private TFObjectDetector tfod;

    public int ObjectLocationDetection(){
        initVuforia();
        initTfod();
    }




    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         *
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
*/