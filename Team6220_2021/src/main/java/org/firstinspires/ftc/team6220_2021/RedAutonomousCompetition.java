package org.firstinspires.ftc.team6220_2021;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Red Autonomous Competition", group = "Competition")
public class RedAutonomousCompetition extends MasterAutonomous {
    private static final String TFOD_MODEL_ASSET = "model_20211128_184150.tflite";
    private static final String[] LABELS = {"TSE"};
    private static final String VUFORIA_KEY = "AXDNhID/////AAABmTzx9+zSP0cgsSvEBLeS2Y9I1y9lY1nEbJ" +
            "0/cUmIw6GzDXvrllKLQizl4X4T6iAxXFMJXR5zS8fcXuy6uS6lzlZJOBRnDXn3FusCpuunkIRPgIVyq+peMi" +
            "d0PN1gwSloq8A+nrV6W1LU10WzZ/Pez2F0to+5aV0bOBB+VhZIdN5ABnoSMPa6JxtR6QaCI3dg++wpGw+/X3" +
            "RwDJhllOoGVmsLE9DTEuBBAI+MtRIpFNrSR7mcv3TEHMf8YIc+qxED8YE7Az3PGK1xy/NzLqNtFdnNVFhp02" +
            "38Kaaqnu3DABLRXRjSJ1QRSHmE8mIur5Dk3OcqMv3fwTNt5CnhC2J/D5biVGixUQ+dveylNEVNmp0k";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    int barcode = -1;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        Initialize();
        initVuforia();
        initTfod();

        servoGrabber.setPosition(0.0);
        pauseMillis(500);
        servoArm.setPosition(0.81);

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0/9.0);
        }

        waitForStart();

        servoArm.setPosition(0.15);
        motorArm.setTargetPosition(555);
        motorArm.setPower(0.9);
        pauseMillis(500);

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

        pauseMillis(2000);

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                switch (barcode) {
                    case 0:
                        telemetry.addData("barcode: ", barcode);
                        telemetry.update();

                        servoArm.setPosition(0.4);
                        motorArm.setTargetPosition(300);
                        driveInches(6, 0.5, true);
                        turnDegrees(60);
                        driveInches(48 / Math.sqrt(3), 0.75, true);
                        turnDegrees(-60);
                        // move forward if necessary
                        servoGrabber.setPosition(0.34);
                        driveInches(-40, 1.0, false);
                        motorArm.setTargetPosition(1100);
                        driveInches(4, 0.5, true);
                        turnDegrees(-90);
                        driveInches(50, 1.0, true);
                        motorDuck.setPower(0.6);
                        pauseMillis(2000);
                        motorDuck.setPower(0.0);
                        driveInches(4, 0.5, false);
                        turnDegrees(180);
                        driveInches(96, 1, true);
                        break;

                    case 1:
                        telemetry.addData("barcode: ", barcode);
                        telemetry.update();

                        servoArm.setPosition(0.3);
                        motorArm.setTargetPosition(555);
                        driveInches(6, 0.5, true);
                        turnDegrees(60);
                        driveInches(48 / Math.sqrt(3), 0.75, true);
                        turnDegrees(-60);
                        // move forward if necessary
                        servoGrabber.setPosition(0.34);
                        driveInches(-40, 1.0, false);
                        motorArm.setTargetPosition(1100);
                        driveInches(4, 0.5, true);
                        turnDegrees(-90);
                        driveInches(50, 1.0, true);
                        motorDuck.setPower(0.6);
                        pauseMillis(2000);
                        motorDuck.setPower(0.0);
                        driveInches(4, 0.5, false);
                        turnDegrees(180);
                        driveInches(96, 1, true);
                        break;

                    case 2:
                        telemetry.addData("barcode: ", barcode);
                        telemetry.update();

                        servoArm.setPosition(0.15);
                        motorArm.setTargetPosition(900);
                        driveInches(6, 0.5, true);
                        turnDegrees(60);
                        driveInches(48 / Math.sqrt(3), 0.75, true);
                        turnDegrees(-60);
                        // move forward if necessary
                        servoGrabber.setPosition(0.34);
                        driveInches(-40, 1.0, false);
                        motorArm.setTargetPosition(1100);
                        driveInches(4, 0.5, true);
                        turnDegrees(-90);
                        driveInches(50, 1.0, true);
                        motorDuck.setPower(0.6);
                        pauseMillis(2000);
                        motorDuck.setPower(0.0);
                        driveInches(4, 0.5, false);
                        turnDegrees(180);
                        driveInches(96, 1, true);
                        break;
                }

                // This goes inside of the switch and is used just like above, but with cubes/ducks/spheres instead of the TSE
                // Used for cycling freight in autonomous
                /*if (tfod != null) {
                    tfod.activate();
                    tfod.setZoom(2.0, 16.0/9.0);

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
                }*/

                break;
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