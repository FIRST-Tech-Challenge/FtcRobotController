package org.firstinspires.ftc.teamcode.Systems.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Systems.RobotHardware;

import java.util.ArrayList;
import java.util.List;


public class Tensorflow implements Robot {

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private RobotHardware robot;

    public Tensorflow(RobotHardware r){
        robot = r;

        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(camera_zoom, 16.0 / 9.0);
            //camera_zoom must be >= 1.0
        }
    }

    public boolean canSee(String name) {
        return getData().contains(name);
    }

    public ArrayList<String> getUpdatedData() {
        return getData(true, -1000000.0, 1000000.0, -1000000.0, 1000000.0);
    }

    public ArrayList<String> getData() {
        return getData(false, -1000000.0, 1000000.0, -1000000.0, 1000000.0);
    }

    public ArrayList<String> getUpdatedData(double min_x, double max_x, double min_y, double max_y) {
        return getData(true, min_x, max_x, min_y, max_y);
    }

    public ArrayList<String> getData(double min_x, double max_x, double min_y, double max_y) {
        return getData(false, min_x, max_x, min_y, max_y);
    }

    public ArrayList<String> getData(boolean updated, double min_x, double max_x, double min_y, double max_y) {
        List<Recognition> recognitions;
        if (updated) {
            recognitions = getRawUpdatedData();
        } else {
            recognitions = getRawData();
        }
        ArrayList<String> data = new ArrayList<String>();
        if (recognitions != null) {
            for (Recognition recognition : recognitions) {
                //only add data if all points are within the boundary rectangle we chose
                if (((min_x < recognition.getLeft()) && (recognition.getRight() < max_x)) && ((min_y < recognition.getBottom()) && (recognition.getTop() < max_y))) {
                    data.add(recognition.getLabel());
                }
            }
        }
        return data;
    }

    public List<Recognition> getRawData(){
        if (tfod != null) {
            //returns all data
            List<Recognition> updatedRecognitions = tfod.getRecognitions();
            if (updatedRecognitions != null) {
                return updatedRecognitions;
            }
        }
        return null;
    }

    public List<Recognition> getRawUpdatedData(){
        if (tfod != null) {
            //only returns the NEW information
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                return updatedRecognitions;
            }
        }
        return null;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = robot.map.get(WebcamName.class, webcam_name);

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = robot.map.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", robot.map.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = min_confidence;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = input_size;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        if (useAsset) {
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        } else {
            tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
        }
    }

}