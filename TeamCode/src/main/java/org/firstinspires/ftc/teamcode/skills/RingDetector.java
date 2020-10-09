package org.firstinspires.ftc.teamcode.skills;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

public class RingDetector {
    Telemetry telemetry;
    private TFObjectDetector tfod;
    private HardwareMap hardwareMap;
    private String targetZone = "";
    // can try floating point model and see what happens

    // with FIRST model
//    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
//    private static final String LABEL_A = "None";
//    private static final String LABEL_B = "Single";
//    private static final String LABEL_C = "Quad";
    // with custom
    private static final String TFOD_MODEL_ASSET = "customUGFloat.tflite";
    private static final String LABEL_A = "A";
    private static final String LABEL_B = "B";
    private static final String LABEL_C = "C";

    private static final String VUFORIA_KEY =
            "AZs0syj/////AAABmaxIME6H4k74lx12Yv3gnoYvtGHACOflWi3Ej36sE7Hn86xDafDA3vhzxSOgBtyNIQ1ua6KP2j3I2ScFedVw8n6MJ7PReZQP4sTdc8gHvoy17hD574exMmoUQ3rVUMkgU2fwN2enw2X+Ls2F3BLuCg/A4SBZjzG3cweO+owiKO/2iSIpeC4rBdUnPiTxqPHNa8UOxyncCGV0+ZFXresQm/rK7HbOKB9MEszi8eW2JNyfjTdKozwDxikeDRV7yPvoIhZ5A+mrrC1GgrEzYwNTVHeki2cg4Ea62pYwdscaJ+6IWHlBIDutqmgJu/Os3kAzZkOh0TJ8P3e29Ou4ZczTdDH0oqkPt78Nt4VdYbSbLRCw";

    private VuforiaLocalizer vuforia;

    public RingDetector(TFObjectDetector tf) {
        tfod = tf;
    }

    public RingDetector(HardwareMap hMap, Telemetry t) {
        hardwareMap = hMap;
        telemetry = t;
        initVuforia();
        initTfod();
        activateTfod();
    }

    public boolean detectRing(int timeout, Telemetry telemetry, LinearOpMode caller){
        boolean found = false;

        boolean stop = false;
        ElapsedTime runtime = new ElapsedTime();
        while (!stop && runtime.seconds() < timeout) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    for (Recognition recognition : updatedRecognitions) {
                        if ((recognition.getLabel().contentEquals(LABEL_B))) {
                            targetZone = "B";
                        } else if (recognition.getLabel().contentEquals(LABEL_C)) {
                            targetZone = "C";
                        } else {
                            targetZone = "A";
                        }

                        found = true;
                        telemetry.addData("Found", found);
                        break;

                    }
                    telemetry.update();
                }
            }
            if (found || !caller.opModeIsActive()){
                stop = true;
            }
        }

        return found;
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "wcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        // comment out if using FIRST model
        tfodParameters.isModelQuantized = false;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        // with FIRST model
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_C, LABEL_B);
        // with custom model
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_A, LABEL_B, LABEL_C);
    }

    protected void activateTfod(){
        if (tfod != null) {
            tfod.activate();
        }
        telemetry.addData("Info", "TF Activated");
    }

    public void stopDetection(){
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public String getTargetZone() {
        return targetZone;
    }
}
