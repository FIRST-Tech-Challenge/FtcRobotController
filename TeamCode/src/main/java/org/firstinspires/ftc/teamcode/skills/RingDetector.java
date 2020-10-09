package org.firstinspires.ftc.teamcode.skills;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.tfrec.Detector;
import org.firstinspires.ftc.teamcode.tfrec.classification.Classifier;
import org.firstinspires.ftc.teamcode.autonomous.AutoDot;
import java.util.ArrayList;
import java.util.List;

public class RingDetector {
    Telemetry telemetry;
    private Detector tfDetector = null;
    private HardwareMap hardwareMap;

    private static String MODEL_FILE_NAME = "rings_float.tflite";
    private static String LABEL_FILE_NAME = "labels.txt";
    private static Classifier.Model MODEl_TYPE = Classifier.Model.FLOAT_EFFICIENTNET;
    private static final String LABEL_A = "None";
    private static final String LABEL_B = "Single";
    private static final String LABEL_C = "Quad";

    public RingDetector(){

    }

    public RingDetector(HardwareMap hMap, Telemetry t) {
        hardwareMap = hMap;
        telemetry = t;
        initDetector();
        activateDetector();
    }

    public AutoDot detectRing(int timeout, Telemetry telemetry, LinearOpMode caller){
        AutoDot zone = new AutoDot();
        zone.setX(90);
        zone.setY(90);
        zone.setHeading(45);
        boolean found = false;

        boolean stop = false;
        ElapsedTime runtime = new ElapsedTime();
        while (!stop && runtime.seconds() < timeout) {
            if (tfDetector != null) {
                List<Classifier.Recognition> results = tfDetector.getLastResults();
                if (results == null || results.size() == 0){
                    telemetry.addData("Info", "No results");
                }
                else {
                    for (Classifier.Recognition r : results) {
                        String item = String.format("%s: %.2f", r.getTitle(), r.getConfidence());
                        telemetry.addData("Found", item);
                        if(r.getConfidence() > 0.8) {
                            if(r.getTitle().contentEquals(LABEL_C)){
                                zone.setX(90);
                                zone.setY(140);
                                zone.setHeading(45);
                                found = true;
                            }
                            if(r.getTitle().contentEquals(LABEL_B)){
                                zone.setX(70);
                                zone.setY(120);
                                zone.setHeading(45);
                                found = true;
                            }
                            if(r.getTitle().contentEquals(LABEL_A)){
                                zone.setX(90);
                                zone.setY(90);
                                zone.setHeading(45);
                                found = true;
                            }
                        }
                    }
                }
                telemetry.update();
            }
            if (found || !caller.opModeIsActive()){
                stop = true;
            }
        }

        return zone;
    }

    public void initDetector() {
        tfDetector = new Detector(MODEl_TYPE, MODEL_FILE_NAME, LABEL_FILE_NAME, hardwareMap.appContext, telemetry);
    }

    protected void activateDetector(){
        if (tfDetector != null) {
            tfDetector.activate();
        }
        telemetry.addData("Info", "TF Activated");
    }

    public void stopDetection(){
        if (tfDetector != null) {
            tfDetector.stopProcessing();
        }
    }
}
