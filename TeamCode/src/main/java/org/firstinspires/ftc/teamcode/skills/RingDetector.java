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

public class RingDetector {
    Telemetry telemetry;
    private Detector tfDetector = null;
    private HardwareMap hardwareMap;
    private Led lights;

    private static String MODEL_FILE_NAME = "rings_float.tflite";
    private static String LABEL_FILE_NAME = "labels.txt";
    private static Classifier.Model MODEl_TYPE = Classifier.Model.FLOAT_EFFICIENTNET;
    private static final String LABEL_A = "None";
    private static final String LABEL_B = "Single";
    private static final String LABEL_C = "Quad";
    private String targetZone = LABEL_C;

    private ArrayList<AutoDot> namedCoordinates = new ArrayList<>();

    private static AutoDot zoneA = new AutoDot("A", 75, 75, -1, AutoRoute.NAME_RED);
    private static AutoDot zoneB = new AutoDot("B", 62, 105, -1, AutoRoute.NAME_RED);
    private static AutoDot zoneC = new AutoDot("C", 75, 120, -1, AutoRoute.NAME_RED);

    public RingDetector(HardwareMap hMap, Led led, Telemetry t) {
        hardwareMap = hMap;
        telemetry = t;
        lights = led;
        initDetector();
        activateDetector();
    }

    protected void configZones(String side){
        if (this.namedCoordinates.size() > 0){
            for(AutoDot d : namedCoordinates){
                if (d.getFieldSide().equals(side)){
                    if (d.getDotName().equals("A")){
                        zoneA = d;
                    }
                    else if (d.getDotName().equals("B")){
                        zoneB = d;
                    }
                    else if (d.getDotName().equals("C")){
                        zoneC = d;
                    }
                }
            }
        }
        else if (side.equals(AutoRoute.NAME_BLUE)){
            zoneA.setX(30);
            zoneA.setX(12);

            zoneB.setX(30);
            zoneB.setX(12);

            zoneC.setX(30);
            zoneC.setX(12);
        }
    }

    public AutoDot detectRing(int timeout, String side, Telemetry telemetry, LinearOpMode caller) {
        configZones(side);
        AutoDot zone = zoneB;
        boolean found = false;
        boolean stop = false;

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        boolean fromConfig = this.namedCoordinates.size() > 0;
        while (caller.opModeIsActive() && runtime.seconds() <= timeout) {
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
                                found = true;
                                this.lights.recognitionSignal(4);
                            }
                            else if(r.getTitle().contains(LABEL_B)){
                                zone = zoneB;
                                found = true;
                                this.lights.recognitionSignal(1);
                            }
                            else if(r.getTitle().contains(LABEL_A)){
                                zone = zoneA;
                                found = true;
                                this.lights.recognitionSignal(0);
                            }
                            targetZone = zone.getDotName();
                            telemetry.addData("Zone", targetZone);
                        }
                    }
                }
            }
            stop = found; //|| runtime.seconds() >= timeout;
            telemetry.update();
        }

        return zone;
    }

    public void initDetector() {
        tfDetector = new Detector(MODEl_TYPE, MODEL_FILE_NAME, LABEL_FILE_NAME, hardwareMap.appContext, telemetry);
    }

    protected void activateDetector() {
        if (tfDetector != null) {
            tfDetector.activate();
        }
        telemetry.addData("Info", "TF Activated");
    }

    public String returnZone() {
        return targetZone;
    }

    public void stopDetection() {
        if (tfDetector != null) {
            tfDetector.stopProcessing();
        }
    }

    public void setNamedCoordinates(ArrayList<AutoDot> namedCoordinates) {
        this.namedCoordinates = namedCoordinates;
    }
}
