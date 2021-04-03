package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


public class SkystoneObjectDetector {

    private static final String VUFORIA_KEY =
            "Aa2sV7z/////AAABmQ+v3uVv5U9jmm77CvwRYvVpMuoA5wivkwdUI7wygLVuYkVpEhXrPoXKFGQVYEDplmlUGXDFfToC63vFmrnhqS3yG6BsoCnyNhR5Jjn1Ik8lCnJR1tlLCINKKBWZJnzD7Dk0hX/je34MyKe1Xcf1gMHoFJsio5DiH//aFvtq67pDxEszk5c+TjsnqeJ4pkSIme1h3SNBg+BLREksvQTHS99BxLnwFAZdNCFHY60FDGXzDIfLUpW5R0d3SnQvBKkQoGfoWtBmi6BG3XwgIbUO3AcLJ44FQM43cN5CqcQoNOr8moCX+L5Jhjf9+yFor3Dry7CkzS47wP7MTWOtJCVVJNSkw2/2dj1kHUdKyiqY59PY";

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private VuforiaLocalizer vuforia;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;
    private TFObjectDetector tfod;
    private Telemetry telemetry;
    private LinearOpMode opMode;
    private HardwareMap hardwareMap;


    public void init(HardwareMap hardwareMapIn, Telemetry telemetryIn, LinearOpMode opModeIn) {
        telemetry = telemetryIn;
        opMode = opModeIn;
        hardwareMap = hardwareMapIn;
        initVuforia();
        initTfod();
        tfod.activate();

    }


    public void updateDetection() {
        List<Recognition> updatedRecognitions = tfod.getRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            for (Recognition recognition : updatedRecognitions) {
                double centerV = (recognition.getBottom() + recognition.getTop()) / 2;
                double centerH = (recognition.getLeft() + recognition.getRight()) / 2.0;
                double angle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                double confidence = recognition.getConfidence();
                telemetry.addData("Detected:", String.format("Object: %s,  HCenter %.0f   Angle: %.0f  Confidence: %.0f", recognition.getLabel(), centerH, angle, confidence * 100));
            }
        } else {
            telemetry.addData("# Object Detected", "none");
        }
    }

    public int choosePath() {
        Recognition skystone = findBestSkystone();
        if (skystone == null) {
            return (0);
        } else if (skystone.estimateAngleToObject(AngleUnit.DEGREES) > 10) {
            return (1);
        } else if (skystone.estimateAngleToObject(AngleUnit.DEGREES) < -10) {
            return (3);
        } else {
            return (2);
        }
    }

    public Recognition findBestSkystone() {
        ArrayList<Recognition> skystones = new ArrayList<Recognition>();
        List<Recognition> recognitions = tfod.getRecognitions();
        for (Recognition recognition : recognitions) {
            if (recognition.getLabel() == "Skystone") {
                skystones.add(recognition);
            }
        }
        //pick the best one.
        double bestWeight = 0;
        Recognition bestSkystone = null;
        for (Recognition skystone : skystones) {
            double weight = skystone.getHeight() * skystone.getWidth() * skystone.getConfidence();
            if (weight > bestWeight) {
                bestWeight = weight;
                bestSkystone = skystone;
            }
        }
        return (bestSkystone);
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
