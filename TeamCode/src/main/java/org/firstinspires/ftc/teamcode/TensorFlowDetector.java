package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

// import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.sql.Array;
import java.util.ArrayList;
import java.util.List;


public class TensorFlowDetector {
    private static final boolean USE_WEBCAM = true;

    private String[] labels = {};
    private boolean isActive = false;
    private TfodProcessor detector;
    private VisionPortal visionPortal;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private String modelName;

    private List<Recognition> currentRecognitions;



    public TensorFlowDetector (String modelName, String[] labels, Telemetry telemetry, HardwareMap hardwaremap) {
        this.modelName = modelName;
        this.labels = labels;
        this.telemetry = telemetry;
        this.hardwareMap = hardwaremap;
    }

    public void initModel () throws InterruptedException {
        initDetector(modelName);
        telemetry.addLine("Model with name " + modelName + " successfully initialized");

        if (isActive) {
            while (isActive) {
                updateRecognitions();
                sleep(20); // @TODO make this use the opmode sleep rather than Thread.sleep
            }
        }
    }

    private void updateRecognitions() {
        currentRecognitions = detector.getRecognitions();
    }

    public int getNumRecognitions() {
        return currentRecognitions.size();
    }

    /**
     * Get the position of all recognitions in the format of a 2-D array:
     * [[x1, y1],
     * [x2, y2]]
     * @return the 2-D array shown above
     */
    public double[][] getRecognitionPositions() {
        double [][] recognitionPositions = new double[getNumRecognitions()][2];
        for (int i = 0; i < getNumRecognitions(); i++) {
            Recognition recognition = currentRecognitions.get(i);
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;
            recognitionPositions[i][0] = x;
            recognitionPositions[i][1] = y;
        }
        return recognitionPositions;
    }

    public double[][] getRecognitionPositions(String label) {
        int numRecognitions = 0;
        for (int i = 0; i < currentRecognitions.size(); i++) {
            Recognition recognition = currentRecognitions.get(i);
            if (recognition.getLabel().equals(label)) {
                // @TODO finish this method
            }
        }

        double [][] recognitionPositions = new double[getNumRecognitions()][2];
        for (int i = 0; i < getNumRecognitions(); i++) {
            Recognition recognition = currentRecognitions.get(i);
            if (recognition.getLabel().equals(label)) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;
                recognitionPositions[i][0] = x;
                recognitionPositions[i][1] = y;
            }
        }
        return recognitionPositions;
    }

    /**
     * Get the position of a recognition with a given index i
     * @param i the index of the recognition to get the coordinates of
     * @return a double[] of length 2 in the format [x, y]
     */
    public double[] getRecognitionPosition(int i) {
        double[] recognitionPosition = new double[2];

        Recognition recognition = currentRecognitions.get(i);
        double x = (recognition.getLeft() + recognition.getRight()) / 2;
        double y = (recognition.getTop() + recognition.getBottom()) / 2;
        recognitionPosition[0] = x;
        recognitionPosition[1] = y;

        return recognitionPosition;
    }

    public double[] getRecognitionPosition(String label) {
        double[] recognitionPosition = new double[2];
        Recognition recognition = getRecognition(label);

        double x = (recognition.getLeft() + recognition.getRight()) / 2;
        double y = (recognition.getTop() + recognition.getBottom()) / 2;
        recognitionPosition[0] = x;
        recognitionPosition[1] = y;

        return recognitionPosition;
    }

    public Recognition getRecognition(int i) {
        return currentRecognitions.get(i);
    }
    public Recognition getRecognition (String label) {
        for (Recognition recognition : currentRecognitions) {
            if (recognition.getLabel().equals(label)) {
                return recognition;
            }
        }
        return null;
    }

    public List<Recognition> getAllRecognitions () {
        return currentRecognitions;
    }

    public List<Recognition> getAllRecognitions (String label) {
        List<Recognition> recognitions = new ArrayList<>();
        for (Recognition recognition : currentRecognitions) {
            if (recognition.getLabel().equals(label)) {
                recognitions.add(recognition);
            }
        }
        return recognitions;
    }


    private void initDetector(String modelName) {
        detector = new TfodProcessor.Builder()
                .setModelAssetName(modelName)
                .setModelLabels (labels)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {

            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        //builder.setCameraResolution(new Size(640, 480));
        builder.enableCameraMonitoring(true);
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        //builder.setAutoStopLiveView(false);
        builder.addProcessor(detector);

        visionPortal = builder.build();

        //detector.setMinResultConfidence(0.75f); // can be called at any time
        //visionPortal.setProcessorEnabled(detector, true); // can be called at any time
    }

}
