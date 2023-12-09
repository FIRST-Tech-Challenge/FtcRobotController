package org.firstinspires.ftc.teamcode.shared;

import static android.os.SystemClock.sleep;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.shared.GlobalState.ALLIANCE_POS;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.Arrays;
import java.util.List;

public class VisionHardware {

    public static boolean DEBUG = true;
    private LinearOpMode myOpMode = null;

    private ALLIANCE_POS alliancePos = null;

    public static double detectWait = 6.0;
    private ElapsedTime runtime = new ElapsedTime();
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private VisionPortal.Builder visionPortalBuilder;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "ModelSpheresClassWindowSLC.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/ModelSphereClassWindowSLC.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Red Sphere",
            "Blue Sphere"
    };
    public enum PropPosition {
        UNKNOWN,
        LEFT,
        MIDDLE,
        RIGHT
    }

    // Screen X axis division segments
    // From Left Alliance Positions
    private int ALLIANCE_LEFT_L = 400; // <=
    private int ALLIANCE_LEFT_M = 400; // >

    // From Right Alliance Positions
    private int ALLIANCE_RIGHT_R = 624; // >=
    private int ALLIANCE_RIGHT_M = 624; // <


    public VisionHardware(LinearOpMode opmode) { myOpMode = opmode; };
    public VisionHardware(LinearOpMode opmode, ALLIANCE_POS alliancePos) {
        myOpMode = opmode;
        this.alliancePos = alliancePos;
    }

    public void init() {
        // Create the TensorFlow processor the easy way.
        // tfod = TfodProcessor.easyCreateWithDefaults();
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortalBuilder = new VisionPortal.Builder();
            visionPortalBuilder.setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"));
            //visionPortalBuilder.setCameraResolution(new Size(640, 480));
            visionPortalBuilder.setCameraResolution(new Size(1280, 720));
            visionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
            visionPortalBuilder.enableLiveView(true);
            visionPortalBuilder.addProcessors(tfod);
            visionPortal = visionPortalBuilder.build();
            //visionPortal = VisionPortal.easyCreateWithDefaults(
            //    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);

        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }
    }

    public PropPosition detectProp() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        // Try to detect the prop for no more than "detectWait" seconds
        while ( timer.seconds() < detectWait ) {
            PropPosition pPosition;
            boolean pixelFound = false;

            List<Recognition> currentRecognitions = tfod.getRecognitions();

            if (currentRecognitions.size() > 0) {
                myOpMode.telemetry.addData("# Objects Detected", currentRecognitions.size());
                myOpMode.telemetry.update();
                debugWait();

                // Step through the list of recognitions and display info for each one.
                for (Recognition recognition : currentRecognitions) {
                    // Is this recognition one of our models labels?
                    if(Arrays.asList(LABELS).contains(recognition.getLabel())) {
                        double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                        double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;


                        switch(alliancePos) {
                            case LEFT:
                                if (x < ALLIANCE_LEFT_L) {
                                    myOpMode.telemetry.addData("Prop Left", "");
                                    myOpMode.telemetry.update();
                                    debugWait();
                                    return PropPosition.LEFT;
                                } else if (x > ALLIANCE_LEFT_M) {
                                    myOpMode.telemetry.addData("Prop Middle", "");
                                    myOpMode.telemetry.update();
                                    debugWait();
                                    //return PropPosition.RIGHT;
                                    return PropPosition.MIDDLE;
                                } else {
                                    myOpMode.telemetry.addData("Prop Right", "");
                                    myOpMode.telemetry.update();
                                    debugWait();
                                    return PropPosition.RIGHT;
                                }
                            case RIGHT:
                                if (x >= ALLIANCE_RIGHT_R) {
                                    myOpMode.telemetry.addData("Prop Right", "");
                                    myOpMode.telemetry.update();
                                    debugWait();
                                    return PropPosition.RIGHT;
                                } else if (x < ALLIANCE_RIGHT_M) {
                                    myOpMode.telemetry.addData("Prop Middle", "");
                                    myOpMode.telemetry.update();
                                    debugWait();
                                    //return PropPosition.RIGHT;
                                    return PropPosition.MIDDLE;
                                } else {
                                    myOpMode.telemetry.addData("Prop Left", "");
                                    myOpMode.telemetry.update();
                                    debugWait();
                                    return PropPosition.LEFT;
                                }
                        }


                        if (x < 400) {
                            myOpMode.telemetry.addData("Prop Left", "");
                            myOpMode.telemetry.update();
                            debugWait();
                            return PropPosition.LEFT;
                        } else if (x > 900) {
                            myOpMode.telemetry.addData("Prop Right", "");
                            myOpMode.telemetry.update();
                            debugWait();
                            //return PropPosition.RIGHT;
                            return PropPosition.RIGHT;
                        } else {
                            myOpMode.telemetry.addData("Prop Middle", "");
                            myOpMode.telemetry.update();
                            debugWait();
                            return PropPosition.MIDDLE;
                        }


                    }
                }
            }

        }
        // We didn't find the prop, so lets pick a location and go
        return PropPosition.UNKNOWN;
    }

    private void debugWait() {
        if (DEBUG) {
            sleep(5000);
        } else {
            sleep(1000);
        }
    }
}
