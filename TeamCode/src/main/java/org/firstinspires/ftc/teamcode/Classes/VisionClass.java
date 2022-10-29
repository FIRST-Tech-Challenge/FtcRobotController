package org.firstinspires.ftc.teamcode.Classes;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

// initializes a new vuforia and tensorflowlite program, still needs algorithms
public class VisionClass {


        String camera = null;

        HardwareMap hardwareMap;

    // constructor
        public VisionClass(String cameraName) {

             camera = cameraName;

        }


        //first init the file that we are using
        String TFOD_MODEL_ASSET = "customModel.tflite";

        // next init the name of vuforia and tfod variables
        VuforiaLocalizer vuforia;

        public TFObjectDetector tfod;

        // next init the vuforia key
        String VUFORIA_KEY = "AZ3E1e//////AAABmYYnnAVE7UQ3r8htn2k0qaxt+gAIhwTJimI+HGMl8mBiRRtW3AxF8CTljgMJfhjt06DnzmcRCnLde7m7wYlDxLE41DkFtLG8929BwtuF9hLIBgnIZFR8sqqSfSbd1G5g1tJ5TO8sItT58OLmRIyNwHbdMOj6Fdrc9fRZSu1Vn1ziuLshTmnNvv/2rBqDaQEUI+UnLrvSIJma9OIc04Rn983vgCj54k6oYvuUW9m1fFfWSrNabMvZa3fiT6GG5vyId9A/yHV/kkbulT2WZhxm2cs5poJiRb8tWX5F4LQvDLEnX12/qNDbyLi8IA2tKPnaaL4bFRfk2TvVXtj38FouN2t2+xeTjOy6Znxlvin872sc";

        // init the labels
        String[] LABELS = {
                "1 circle",
                "2 square",
                "3 star"
        };


        // init vuforia/tfod
        public void initVuforia () {
            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             */
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraName = hardwareMap.get(WebcamName.class, camera);

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);
        }

        public void initTfod () {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minResultConfidence = 0.75f;
            tfodParameters.isModelTensorFlow2 = true;
            tfodParameters.inputSize = 300;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

            // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
            // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
            // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
        }

        // still have to add algorithms



}




