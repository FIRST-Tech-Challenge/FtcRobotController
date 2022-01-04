package org.firstinspires.ftc.team6220_2021.TestClasses;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.team6220_2021.MasterAutonomous;
import org.firstinspires.ftc.team6220_2021.ResourceClasses.Constants;

import java.util.List;

@Disabled
@Autonomous(name = "AutonomousTest", group = "Test")
public class AutonomousTest extends MasterAutonomous {
    private static final String TFOD_MODEL_ASSET = Constants.TENSORFLOW_MODEL_ASSET;
    private static final String[] LABELS = Constants.TENSORFLOW_LABELS;
    private static final String VUFORIA_KEY = Constants.VUFORIA_KEY;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    int barcode = 2;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        Initialize();
        initVuforia();
        initTfod();

        servoGrabber.setPosition(0.0);
        servoArm.setPosition(0.81);

        pauseMillis(500);

        servoArm.setPosition(0.4);
        motorArm.setPower(0.9);
        motorArm.setTargetPosition(1100);

        waitForStart();

        double startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < 2000) {
            if (tfod != null) {
                tfod.activate();
                tfod.setZoom(1.0, 16.0/9.0);

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

                            if (TSELocation > 0.0 && TSELocation <= 267.0) {
                                barcode = 0;
                            } else if (TSELocation > 267.0 && TSELocation <= 533.0) {
                                barcode = 1;
                            } else if (TSELocation > 533.0 && TSELocation <= 800.0) {
                                barcode = 2;
                            }
                        }
                    }
                    telemetry.update();
                }
            }
        }

        telemetry.addData("barcode: ", barcode);
        telemetry.update();

        driveInches(20, 0.25, false);
        pauseMillis(500);
        turnToAngle(-90);
        pauseMillis(500);
        driveInches(20, 0.25, false);
        pauseMillis(500);
        turnToAngle(-180);
        pauseMillis(500);
        driveInches(20, 0.25, false);
        pauseMillis(500);
        turnToAngle(90);
        pauseMillis(500);
        driveInches(20, 0.25, false);
        pauseMillis(500);
        turnToAngle(0);

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