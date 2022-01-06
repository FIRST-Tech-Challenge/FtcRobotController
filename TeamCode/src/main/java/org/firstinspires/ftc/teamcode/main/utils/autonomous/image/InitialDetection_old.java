package org.firstinspires.ftc.teamcode.main.utils.autonomous.image;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.main.utils.resources.Resources;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Initial Object Detection (OLD)", group = "Concept")
@Deprecated
public class InitialDetection_old extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_DM.tflite";
    private static final String[] LABELS = {
            "Duck",
            "Marker"
    };

    private static final String OBJECT_TO_IDENT = "Duck";

    private String VUFORIA_KEY;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private List<InitialPositions> PossiblePositions = Arrays.asList(
            InitialPositions.POS1,
            InitialPositions.POS2,
            InitialPositions.POS3
    );

    private boolean isZoomed = false;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        VUFORIA_KEY = Resources.Misc.VuforiaKey;

        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();

            // tfod.setZoom(1.5, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            i++;

                            try {
                                if (PossiblePositions.get(0).evalPos((int) recognition.getLeft())) {
                                    if (recognition.getLabel().equals(OBJECT_TO_IDENT)) {
                                        PossiblePositions = Arrays.asList(InitialPositions.POS1);
                                    } else {
                                        PossiblePositions.remove(InitialPositions.POS1);
                                    }
                                } else if (PossiblePositions.get(1).evalPos((int) recognition.getLeft())) {
                                    if (recognition.getLabel().equals(OBJECT_TO_IDENT)) {
                                        PossiblePositions = Arrays.asList(InitialPositions.POS2);
                                    } else {
                                        PossiblePositions.remove(InitialPositions.POS2);
                                    }
                                } else if (PossiblePositions.get(2).evalPos((int) recognition.getLeft())) {
                                    if (recognition.getLabel().equals(OBJECT_TO_IDENT)) {
                                        PossiblePositions = Arrays.asList(InitialPositions.POS3);
                                    } else {
                                        PossiblePositions.remove(InitialPositions.POS3);
                                    }
                                }
                            } finally {

                            }
                        }
                        StringBuilder strPositions = new StringBuilder();
                        for (InitialPositions positions : PossiblePositions) {
                            strPositions.append(positions.getName()).append(" ");
                        }
                        telemetry.addData("Possible Positions: ", strPositions.toString());

                        telemetry.update();

                        if (gamepad1.cross) {
                            PossiblePositions = Arrays.asList(
                                    InitialPositions.POS1,
                                    InitialPositions.POS2,
                                    InitialPositions.POS3
                            );
                        }
                        if (gamepad1.circle) {
                            if (isZoomed) { tfod.setZoom(1.0, 16.0/9.0);}
                            else { tfod.setZoom(1.5, 16.0/9.0); }

                            isZoomed = !isZoomed;
                        }
                    }
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
        parameters.cameraName = hardwareMap.get(WebcamName.class, Resources.Misc.Webcam);

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
        tfodParameters.minResultConfidence = 0.5f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
