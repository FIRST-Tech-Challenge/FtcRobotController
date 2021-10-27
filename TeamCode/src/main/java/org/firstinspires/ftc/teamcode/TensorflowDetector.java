package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Disabled
public class TensorflowDetector extends LinearOpMode {

    // Some constants
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    protected VuforiaLocalizer vuforia;
    protected TFObjectDetector tfod;
    private static final String VUFORIA_KEY =
            "ASJ84HD/////AAABmRoapDQCMUVDsobE/TsERteBT/byfHigXNEF2qLnOl7XfT98+cbIJA2YIN0aAWkQjcn5auEYl861a1geKL9WarkoFWWN9GlCUSzte7rr/bvtZ5QK0bYpGvN428j0jaYFd24N59osAY7JsiVrs8E4fjzudhN82YVsC0vs11yYcDzTtSxtIZ2JEZcaiLEjoyOhJCe0H8FwnqP7z3XMIo8JmxlqKcDOD4YCc6VImHR4D/hUADmzvv5w2Genyq143rT/E2Zx6FzqIy0zY/tbNBtz2irIqGizqiNlyl3/bVbBa+Wqzyfhsxv4Z6v8dkAMYJCPT23fcdsHhMCKIaxvIDhX44LF4/AqVdhQPjGRwr5XPI3o";
    public int leftBound;
    public int rightBound;
    public TensorflowDetector(int leftBound, int rightBound){


        this.rightBound = rightBound;
        this.leftBound = leftBound;


        initVuforia();
        initTfod();

        this.leftBound = leftBound;


        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (this.tfod != null) {
            this.tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }


    }


    public int recognizeObjects(){
        int mostConfidant = 0;
        int confidencePlace = 0;
        if (this.tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                telemetry.update();
                // Find the most confident duck
                for (int i = 0; i < updatedRecognitions.size(); i++) {
                    Recognition recognition = updatedRecognitions.get(i);
                    if (recognition.getLabel() == "Duck"){
                        if (recognition.getConfidence() > mostConfidant){
                            mostConfidant = (int) recognition.getConfidence();
                            confidencePlace = i;
                        }
                    }
                }

                Recognition correctDuck = updatedRecognitions.get(confidencePlace);
                int place = (int) correctDuck.getLeft();

                // If it's all the way to the right, 2.
                // If middle, 1.
                // If Left, 0.

                if (place > this.rightBound){
                    return 2;
                } else if (leftBound < place && place < rightBound){
                    return 1;
                } else{
                    return 0;
                }

            }
        }

    }


    public void runOpMode(){

    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        this.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        this.tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
