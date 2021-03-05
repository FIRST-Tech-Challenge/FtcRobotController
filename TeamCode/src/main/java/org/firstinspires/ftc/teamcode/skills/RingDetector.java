package org.firstinspires.ftc.teamcode.skills;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;
import org.firstinspires.ftc.teamcode.tfrec.Detector;
import org.firstinspires.ftc.teamcode.tfrec.classification.Classifier;
import org.firstinspires.ftc.teamcode.autonomous.AutoDot;

import java.util.ArrayList;
import java.util.List;

public class RingDetector implements Runnable{
    Telemetry telemetry;
    private Detector tfDetector = null;
    private HardwareMap hardwareMap;
    private Led lights;

    private boolean isRunning = true;

    private AutoDot recogZone = null;
//a
    private static String MODEL_FILE_NAME = "rings_float.tflite"; //"croppedRingRec.tflite";
    private static String LABEL_FILE_NAME = "labels.txt"; //"croppedLabels.txt";
    private static Classifier.Model MODEl_TYPE = Classifier.Model.FLOAT_EFFICIENTNET;
    private static final String LABEL_A = "None";
    private static final String LABEL_B = "Single";
    private static final String LABEL_C = "Quad";
    // default zone
    private String targetZone = LABEL_B;

    private String side = AutoRoute.NAME_RED;
    private LinearOpMode caller = null;

    private ArrayList<AutoDot> namedCoordinates = new ArrayList<>();

    private static AutoDot zoneA = new AutoDot("A", 75, 75, -1, AutoRoute.NAME_RED);
    private static AutoDot zoneB = new AutoDot("B", 65, 100, -1, AutoRoute.NAME_RED);
    private static AutoDot zoneC = new AutoDot("C", 75, 120, -1, AutoRoute.NAME_RED);

    public RingDetector(HardwareMap hMap, String side, LinearOpMode caller, ArrayList<AutoDot> namedCoordinates, Led led, Telemetry t) throws Exception {
        hardwareMap = hMap;
        telemetry = t;
        lights = led;
        initDetector();
        activateDetector();
        this.side = side;
        this.caller = caller;
        if (namedCoordinates != null) {
            this.namedCoordinates = namedCoordinates;
        }
        configZones(side);
    }

    protected void configZones(String side){
        if (this.namedCoordinates.size() > 0){
            for(AutoDot d : namedCoordinates){
                if (d.getFieldSide().equals(this.side)) {
                    if (d.getFieldSide().equals(side)) {
                        if (d.getDotName().equals("A")) {
                            zoneA = d;
                        } else if (d.getDotName().equals("B")) {
                            zoneB = d;
                        } else if (d.getDotName().equals("C")) {
                            zoneC = d;
                        }
                    }
                }
            }
        }
    }

    public AutoDot detectRing(int timeout, String side, Telemetry telemetry, LinearOpMode caller) {
        configZones(side);
        AutoDot zone = zoneB;

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        boolean fromConfig = this.namedCoordinates.size() > 0;
        while (runtime.seconds() <= timeout) {
            telemetry.addData("this.namedCoordinates.size() > 0", fromConfig);
            if (tfDetector != null) {
                List<Classifier.Recognition> results = tfDetector.getLastResults();
                if (results == null || results.size() == 0) {
                    telemetry.addData("Nada", "No results");
                } else {
                    for (Classifier.Recognition r : results) {
                        if (r.getConfidence() >= 0.8) {
                            telemetry.addData("PrintZone", r.getTitle());
                            if (r.getTitle().contains(LABEL_C)) {
                                zone = zoneC;
                            }
                            else if(r.getTitle().contains(LABEL_B)){
                                zone = zoneB;
                            }
                            else if(r.getTitle().contains(LABEL_A)){
                                zone = zoneA;
                            }
                            targetZone = zone.getDotName();
                            telemetry.addData("Zone", targetZone);
                            telemetry.addData("left", r.getLocation().left);
                            telemetry.addData("right", r.getLocation().right);
                        }
                    }
                }
            }
            telemetry.update();
        }

        return zone;
    }

    public void detectRingThread() {
        this.recogZone = zoneB;

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        boolean fromConfig = this.namedCoordinates.size() > 0;
        while (isRunning) {
            telemetry.addData("this.namedCoordinates.size() > 0", fromConfig);
            if (tfDetector != null) {
                List<Classifier.Recognition> results = tfDetector.getLastResults();
                if (results == null || results.size() == 0) {
                    telemetry.addData("Nada", "No results");
                } else {
                    for (Classifier.Recognition r : results) {
                        if (r.getConfidence() >= 0.8) {
                            telemetry.addData("PrintZone", r.getTitle());
                            if (r.getTitle().contains(LABEL_C)) {
                                this.recogZone = zoneC;
                            }
                            else if(r.getTitle().contains(LABEL_B)){
                                this.recogZone = zoneB;
                            }
                            else if(r.getTitle().contains(LABEL_A)){
                                this.recogZone = zoneA;
                            }
                            targetZone = this.recogZone.getDotName();
                            telemetry.addData("Zone", targetZone);
                            telemetry.addData("left", r.getLocation().left);
                            telemetry.addData("right", r.getLocation().right);
                        }
                    }
                }
            }
            telemetry.update();
        }

    }

    public void displayLights() {
        switch (targetZone) {
            case "C":
                this.lights.blue();
                break;
            case "B":
                this.lights.orange();
                break;
            case "A":
                this.lights.pink();
                break;
        }
    }

    public void initDetector() throws Exception {
        tfDetector = new Detector(MODEl_TYPE, MODEL_FILE_NAME, LABEL_FILE_NAME, hardwareMap.appContext, telemetry);
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
            detectRingThread();
        }
    }

    public AutoDot getRecogZone() {
        return recogZone;
    }

    public void setRecogZone(AutoDot recogZone) {
        this.recogZone = recogZone;
    }
}
