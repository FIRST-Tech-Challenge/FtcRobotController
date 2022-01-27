package org.firstinspires.ftc.team6220_2021.TestClasses;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.team6220_2021.ResourceClasses.Constants;

import java.util.List;

@Disabled
@TeleOp(name = "TensorFlow Object Detection", group = "Concept")
public class TensorFlowObjectDetection extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = Constants.TENSORFLOW_MODEL_ASSET;
    private static final String[] LABELS = Constants.TENSORFLOW_LABELS;
    private static final String VUFORIA_KEY = Constants.VUFORIA_KEY;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    int barcode = 2;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.0, 16.0/9.0);
        }

        waitForStart();

        while (opModeIsActive()) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f", recognition.getRight(), recognition.getBottom());
                    i++;

                    if (recognition.getLabel().equals("TSE")) {
                        double TSELocation = (recognition.getLeft() + recognition.getRight()) / 2.0;

                        if (TSELocation > 200.0 && TSELocation <= 333.0) {
                            barcode = 0;
                            telemetry.addData("barcode: ", barcode);
                        } else if (TSELocation > 333.0 && TSELocation <= 467.0) {
                            barcode = 1;
                            telemetry.addData("barcode: ", barcode);
                        } else if (TSELocation > 467.0 && TSELocation <= 600.0) {
                            barcode = 2;
                            telemetry.addData("barcode: ", barcode);
                        }
                    }
                }
                telemetry.update();
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}