package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


public class UGObjectDetector {

    private static final String VUFORIA_KEY =
            "Aa2sV7z/////AAABmQ+v3uVv5U9jmm77CvwRYvVpMuoA5wivkwdUI7wygLVuYkVpEhXrPoXKFGQVYEDplmlUGXDFfToC63vFmrnhqS3yG6BsoCnyNhR5Jjn1Ik8lCnJR1tlLCINKKBWZJnzD7Dk0hX/je34MyKe1Xcf1gMHoFJsio5DiH//aFvtq67pDxEszk5c+TjsnqeJ4pkSIme1h3SNBg+BLREksvQTHS99BxLnwFAZdNCFHY60FDGXzDIfLUpW5R0d3SnQvBKkQoGfoWtBmi6BG3XwgIbUO3AcLJ44FQM43cN5CqcQoNOr8moCX+L5Jhjf9+yFor3Dry7CkzS47wP7MTWOtJCVVJNSkw2/2dj1kHUdKyiqY59PY";

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private VuforiaLocalizer vuforia;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;
    private TFObjectDetector tfod;
    private Telemetry telemetry;
    private LinearOpMode opMode;
    private HardwareMap hardwareMap;

    public enum ringStackState {NONE, SINGLE, QUAD}


    public void init(HardwareMap hardwareMapIn, Telemetry telemetryIn, LinearOpMode opModeIn) {
        telemetry = telemetryIn;
        opMode = opModeIn;
        hardwareMap = hardwareMapIn;
        initVuforia();
        initTfod();
        tfod.activate();
        tfod.setZoom(2.0,16.0/9.0);

    }

    public void shutdown() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }


    public ringStackState findRings() {
        List<Recognition> recognitions = tfod.getRecognitions();
        if (recognitions.isEmpty()) {
            return ringStackState.NONE;
        }
        for (Recognition ringsFound : recognitions) {
            if (ringsFound.getLabel() == LABEL_FIRST_ELEMENT) {
                return ringStackState.QUAD;
            }
        }
        return ringStackState.SINGLE;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

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
