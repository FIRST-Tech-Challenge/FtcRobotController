package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="CustomVisionTest")
public class DuckSighitng extends LinearOpMode {
    public static final String VUFORIA_KEY = "ASfLYBz/////AAABmYk1LysDykd/qmEkhSO3jwVr8HclkZRs2cZqASWoHjt8lPL/jacpgorb3/7EyIeUWBQ1HOj3v83gLZzf8BrbOXMWyD6nxsvkhDuopNeMn29OlHHByT4JV48X2+OMTz6UEp4ZaQW9vlDSE27cdUgIh1r8v2TxjZiwzd15B8gbihS1U7n5IjmyjXQ4i/7m+/hTZiS8Z+hD4sxLgqI0cVrrwH+IVVwV/ztSY2Am+47dvOgEPuQdUagVOuMf5n+Em99kc+ckzkJCoYDfGkwnvIOYLC1evm50I+dMubzhajimPFCaqWQOne0P8hMX0a8lmEKBKtuhg+Ihge8ozziU8Oc1/89u0LnOx29pWqS/bKNka8Rr";

    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/visiondata/cubeandduck.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    public static final String[] LABELS = {
            "Duck",
            "Cube"
            //"Poppy",
            //"Custom"
    };


    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public void runOpMode()
    {
        initVuforia();
        initTfod();

        if(tfod != null)
        {
            tfod.activate();
            tfod.setZoom(1.0, 16.0/9.0);
        }else{
            tfod.deactivate();
            return;
        }

        // vision results
        waitForStart();
        while (opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;

                        if(recognition.getLabel().equals("Duck")) {
                            if ((recognition.getRight() - recognition.getLeft())/2 + recognition.getLeft() < 0.0){
                                telemetry.addData("Position: ", "LEFT");
                            }else{
                                telemetry.addData("Position: ", "RIGHT");
                            }
                        }
                    }
                    telemetry.update();
                }
            }
        }

    }



    /**
     * Initialize the Vuforia localization engine.
     */
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
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }
}