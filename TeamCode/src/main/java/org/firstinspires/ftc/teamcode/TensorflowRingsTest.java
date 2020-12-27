package org.firstinspires.ftc.teamcode;

//import android.hardware.camera2.CameraDevice;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "Tensorflow Clipping Test", group = "Concept")

public class TensorflowRingsTest extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";


    private static final String VUFORIA_KEY =
            " Ac/2h37/////AAABmbXAvaZQqkPSlZv4583jp15xBpCuzySKMfid1ppM+8fZbZsGd93ri87TKmjKKCYA64DjBiSRboJvg0eldCw/QzbXtH/gNzdbd90bD226N+MA3p3b4CH+C8Pe+Q2SPV5d4e23K514g/DZGu5JEHHH5kl1guWLfc485PCIGE/wlhIprwSQmGM535rO6oif8Dka9K6zFPkiiSvsj4SoTdVJ9EMPnSYT1LNRUtcWWyN0aCVFJ2cmU2lCAtvS6t7GACGTQAbq+vURBnS0BLwkqgebDbvPPM6y4LOG904dFosYxQsSJw51CCTDNLXlunkQcEzp8DSjH79jiTb6BMwGtpRbFhyGrtSq+ugYlE6uf+C7V913 ";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
            tfod.setClippingMargins(0,0,0,0);
            //                      B T R L

        }


        final int maxWidth = 1280;
        final int maxHeight = 720;
        final int widthLimit = 960;
        final int heightLimit = 540;
        int leftClip = 0;
        int rightClip = 0;
        int topClip = 0;
        int bottomClip = 0;
        boolean a = false;
        boolean b = false;
        boolean x = false;
        boolean y = false;
        boolean rgtBumper = false;
        int clipIncrement = 30;
        boolean flashlight = true;

        CameraDevice.getInstance().setFlashTorchMode(flashlight);

        //waitForStart()
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

            while (opModeIsActive()) {

                if (gamepad1.right_bumper && !rgtBumper) {
                    flashlight = !flashlight;
                    CameraDevice.getInstance().setFlashTorchMode(flashlight);
                }


                if (gamepad1.a && !a) {
                    //bottomClip += clipIncrement;
                    bottomClip = Range.clip(bottomClip + clipIncrement, 0, heightLimit);
                }
                if (gamepad1.b && !b) {
                    //rightClip += clipIncrement;
                    rightClip = Range.clip(rightClip + clipIncrement, 0, widthLimit);
                }
                if (gamepad1.x && !x) {
                    //leftClip += clipIncrement;
                    leftClip = Range.clip(leftClip + clipIncrement, 0, widthLimit);
                }
                if (gamepad1.y && !y) {
                    //topClip += clipIncrement;
                    topClip = Range.clip(topClip + clipIncrement, 0, heightLimit);
                }
                a = gamepad1.a;
                b = gamepad1.b;
                x = gamepad1.x;
                y = gamepad1.y;
                rgtBumper = gamepad1.right_bumper;

                // Make sure sum of right and left (or top and bottom) is not more than max
                double extraHeight = (bottomClip + topClip) - heightLimit;
                if (extraHeight > 0) {
                    if (bottomClip > topClip) {
                        topClip -= clipIncrement;
                    } else {
                        bottomClip -= clipIncrement;
                    }
                }

                double extraWidth = (leftClip + rightClip) - widthLimit;
                if (extraWidth > 0) {
                    if (leftClip > rightClip) {
                        rightClip -= clipIncrement;
                    } else {
                        leftClip -= clipIncrement;
                    }
                }

                tfod.setClippingMargins(leftClip,topClip,rightClip,bottomClip);
                //                      L T R B

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("Clipping Amount", "L: %d, T: %d, R: %d, B: %d",
                                leftClip, topClip, rightClip, bottomClip);
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            //telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    //recognition.getLeft(), recognition.getTop());
                            //telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    //recognition.getRight(), recognition.getBottom());
                        }
                        telemetry.update();
                    }
                }
            }

        if (tfod != null) {
            tfod.shutdown();
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
        parameters.cameraDirection = CameraDirection.BACK;

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
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}