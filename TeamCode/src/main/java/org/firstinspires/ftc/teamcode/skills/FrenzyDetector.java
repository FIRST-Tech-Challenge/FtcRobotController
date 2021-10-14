package org.firstinspires.ftc.teamcode.skills;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;
import org.firstinspires.ftc.teamcode.tfrec.Detector;
import org.firstinspires.ftc.teamcode.tfrec.classification.Classifier;

import java.util.List;

public class FrenzyDetector implements Runnable {
    Telemetry telemetry;
    private Detector tfDetector = null;
    private HardwareMap hardwareMap;

    private boolean isRunning = true;

    private String modelFileName = "frenzy_float.tflite";
    private String labelFileName = "frenzy_labels.txt";
    private static Classifier.Model MODEl_TYPE = Classifier.Model.FLOAT_EFFICIENTNET;
    private static final String LABEL_LEFT = "Left";
    private static final String LABEL_MIDDLE = "Middle";
    private static final String LABEL_RIGHT = "Right";

    // Default Zone
    private String targetZone = LABEL_MIDDLE;

    private String side = AutoRoute.NAME_RED;
    private LinearOpMode caller = null;


    public FrenzyDetector(HardwareMap hMap, String side, LinearOpMode caller, Telemetry t) throws Exception {
        hardwareMap = hMap;
        this.side = side;
        this.caller = caller;
        telemetry = t;
        initDetector();
        activateDetector();
    }

    public void detectFrenzyThread() {
        while (isRunning) {
            if (tfDetector != null) {
                List<Classifier.Recognition> results = tfDetector.getLastResults();
                if (results == null || results.size() == 0) {
                    telemetry.addData("Nada", "No results");
                } else {
                    for (Classifier.Recognition r : results) {
                        if (r.getConfidence() >= 0.8) {
                            targetZone = r.getTitle();
                            telemetry.addData("PrintZone", targetZone);
                        }
                    }
                }
            }
            telemetry.update();
        }
    }

    public void initDetector() throws Exception {
        tfDetector = new Detector(MODEl_TYPE, getModelFileName(), getLabelFileName(), hardwareMap.appContext, telemetry);
    }

    protected void activateDetector() throws Exception {
        if (tfDetector != null) {
            tfDetector.activate();
        }
        telemetry.addData("Info", "TF Activated");
    }

    public String returnZone() {
        return targetZone;
    }

    public void stopDetection() {
        stopThread();
        if (tfDetector != null) {
            tfDetector.stopProcessing();
        }
        tfDetector = null;
    }

    public void stopThread() {
        isRunning = false;
    }

    @Override
    public void run() {
        while(isRunning) {
            detectFrenzyThread();
        }
    }

    public String getModelFileName() {
        return modelFileName;
    }

    public void setModelFileName(String modelFileName) {
        this.modelFileName = modelFileName;
    }

    public String getLabelFileName() {
        return labelFileName;
    }

    public void setLabelFileName(String labelFileName) {
        this.labelFileName = labelFileName;
    }
}
