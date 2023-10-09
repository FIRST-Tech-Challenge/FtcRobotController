package org.firstinspires.ftc.teamcode.components;

//import static java.lang.Thread.sleep;

//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

// import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.List;


public class TensorFlowDetector {
    // this should pretty much always be true
    private static final boolean USE_WEBCAM = true;

    private String[] labels = {};
    protected TfodProcessor detector;
    protected VisionPortal visionPortal;
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    private String modelName;

    private List<Recognition> currentRecognitions;

    private float confidenceThreshold = 0.75f;



    public TensorFlowDetector (String modelName, String[] labels, Telemetry telemetry,
                               HardwareMap hardwaremap) {
        this.modelName = modelName;
        this.labels = labels;
        this.telemetry = telemetry;
        this.hardwareMap = hardwaremap;
    }

    public TensorFlowDetector(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    /**
     * Sets the model name. Does not do anything after running initModel()
     * @param modelName A string representing the model name. EX: "Blue_Cone_Test_Model.tflite"
     */
    public void setModelName(String modelName) {
        this.modelName = modelName;

    }

    /**
     * Sets the labels. Does not do anything after running initModel()
     * @param labels An array of string that represent all the labels to search for
     */
    public void setLabels(String[] labels) {
        this.labels = labels;
    }

    /**
     * Initialize the model, and update the recognitions once.
     */
    public void initModel () {
        initDetector(modelName);
        telemetry.addLine("Model with name " + modelName + " successfully initialized");

        updateRecognitions();

    }

    // ********************************************************************************************
    // **************************** BEGIN TELEMETRY METHODS ***************************************
    // ********************************************************************************************

    /**
     * Updates the telemetry with information based on parameters. Will update for ALL recognitions
     * @param showNumDetections Display the total number of detections.
     * @param showXYPos Display the XY position of each recognition
     * @param showDetectionConfidences Display the confidence in each detection.
     * @param showSizes Display the sizes in pixels of each detection
     * @param showEstimatedAngle Display the ESTIMATED angle from the camera to each detection.
     */
    public void updateTelemetry(boolean showNumDetections, boolean showXYPos,
                                boolean showDetectionConfidences, boolean showSizes,
                                boolean showEstimatedAngle) {
        if (currentRecognitions.size() == 0) {
            telemetry.addLine("No objects currently detected");
        } else {
            if (showNumDetections) {
                telemetry.addData("Number of Detections: ", "%d",
                        getNumRecognitions());
            }
            for (Recognition recognition : currentRecognitions) {
                telemetry.addData("", " ");
                if (showDetectionConfidences) {
                    telemetry.addData("Image", "%s (%.0f %% Conf.)",
                            recognition.getLabel(), recognition.getConfidence() * 100);
                } else {
                    telemetry.addData("Image", "%s", recognition.getLabel());
                }
                if (showXYPos) {
                    double x = (recognition.getLeft() + recognition.getRight()) / 2;
                    double y = (recognition.getTop() + recognition.getBottom()) / 2;
                    telemetry.addData("- Position", "%.0f / %.0f", x, y);
                }
                if (showSizes) {
                    telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(),
                            recognition.getHeight());
                }
                if (showEstimatedAngle) {
                    telemetry.addData("- Estimated Angle", "%.0f degrees",
                            recognition.estimateAngleToObject(AngleUnit.DEGREES));
                }
            }
        }
    }

    /**
     * Updates the telemetry with NumDetections, XYPos, DetectionConfidences, and Sizes.
     * Will update for ALL recognitions.
     */
    public void updateTelemetry() {
        updateTelemetry(true, true, true, true, false);
    }

    /**
     * Updates the telemetry with information about a single detection with index i, based on
     * parameters.
     * @param i the index of the recognition to get. Will throw an exception if the recognition
     *          with that index doesn't exist.
     * @param showXYPos Display the XY position of the recognition
     * @param showDetectionConfidence Display the confidence of the recognition
     * @param showSize Display the size of the recognition
     * @param showEstimatedAngle Display the ESTIMATED angle to the recognition
     */
    public void updateTelemetry (int i, boolean showXYPos, boolean showDetectionConfidence,
                                 boolean showSize, boolean showEstimatedAngle){
        checkValidIndex(i);
        Recognition recognition = getRecognition(i);

        telemetry.addData("", " ");
        if (showDetectionConfidence) {
            telemetry.addData("Image", "%s (%.0f %% Conf.)",
                    recognition.getLabel(), recognition.getConfidence() * 100);
        } else {
            telemetry.addData("Image", "%s", recognition.getLabel());
        }
        if (showXYPos) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
        }
        if (showSize) {
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(),
                    recognition.getHeight());
        }
        if (showEstimatedAngle) {
            telemetry.addData("- Estimated Angle", "%.0f degrees",
                    recognition.estimateAngleToObject(AngleUnit.DEGREES));
        }
    }

    /**
     * Updates the telemetry with NumDetections, XYPos, DetectionConfidences, and Sizes for a single
     * recognition with index i.
     * @param i the index of the recognition to update
     */
    public void updateTelemetry (int i) {
        updateTelemetry (i, true, true, true, false);
    }

    /**
     * Updates the telemetry with information about a SINGLE detection with a given label
     * @param label the label to search for
     */
    public void updateTelemetry(String label) {
        int i = getRecognitionIndex(label);
        if (i == -1) {
            telemetry.addLine("No recognitions with label \"" + label + "\" were found.");
        } else {
            updateTelemetry(i);
        }
    }


    /**
     * Updates the telemetry with information about ALL detections with a given label
     * @param label the label to search for
     */
    public void updateTelemetry(String label, boolean showAll) {
        if (!showAll) {
            updateTelemetry(label);
        } else {
            List<Integer> indexes = getRecognitionIndexes(label);
            if (indexes.isEmpty()) {
                telemetry.addLine("No recognitions with label \"" + label + "\" were found.");
            }
            for (int i = 0; i < indexes.size(); i++) {
                updateTelemetry(indexes.get(i));
            }
        }
    }

    /**
     * Updates the telemetry with information about detections with a given label, based on
     * parameters
     * @param label the label to search for
     * @param showAll whether the telemetry should be updated for only the first detection with that
     *                label, or all detections with that label
     * @param showXYPos display the XY position of the detections
     * @param showDetectionConfidence display the detection confidence of the detections
     * @param showSize display the size of the detections
     * @param showEstimatedAngle display the ESTIMATED angle from the camera to the detections
     */
    public void updateTelemetry(String label, boolean showAll, boolean showXYPos,
                                boolean showDetectionConfidence, boolean showSize,
                                boolean showEstimatedAngle) {
        if (!showAll) {
            updateTelemetry(label, showXYPos, showDetectionConfidence, showSize, showEstimatedAngle);
        } else {
            List<Integer> indexes = getRecognitionIndexes(label);
            if (indexes.isEmpty()) {
                telemetry.addLine("No recognitions with label \"" + label + "\" were found.");
            }
            for (int i = 0; i < indexes.size(); i++) {
                updateTelemetry(indexes.get(i), showXYPos, showDetectionConfidence, showSize,
                        showEstimatedAngle);
            }
        }
    }

    /**
     * Updates the telemetry with information about a SINGLE detection with a given label, based on
     * parameters
     * @param label the label to search for
     * @param showXYPos display the XY position of the detections
     * @param showDetectionConfidence display the detection confidence of the detections
     * @param showSize display the size of the detections
     * @param showEstimatedAngle display the ESTIMATED angle from the camera to the detections
     */
    public void updateTelemetry(String label, boolean showXYPos, boolean showDetectionConfidence,
                                boolean showSize, boolean showEstimatedAngle) {
        int i = getRecognitionIndex(label);
        if (i == -1) {
            telemetry.addLine("No recognitions with label \"" + label + "\" were found.");
        } else {
            updateTelemetry(i, showXYPos, showDetectionConfidence, showSize, showEstimatedAngle);
        }
    }

    // ********************************************************************************************
    // **************************** BEGIN DETECTOR TUNING METHODS *********************************
    // ********************************************************************************************


    /**
     * This updates the recognitions. THIS METHOD SHOULD NOT BE CALLED IN A LOOP WITHOUT USING
     * SLEEP TO SHARE THE CPU TIME!
     */
    public void updateRecognitions() {
        currentRecognitions = detector.getRecognitions();
    }

    /**
     * set the minimum confidence for a detection to be considered valid. By default, 0.75f
     * @param confidenceThreshold a float describing the minimum confidence needed to count a
     *                            detection. By default, 0.75f
     */
    public void setConfidenceThreshold (float confidenceThreshold) {
        if (confidenceThreshold < 0f || confidenceThreshold > 1f) {
            throw new IllegalArgumentException("confidence threshold should be a float 0 < x < 1");
        } else if (confidenceThreshold == 0f || confidenceThreshold == 1f) {
            throw new IllegalArgumentException(
                    "confidence threshold should be a value between 0 and 1, non-inclusive"
            );
        }
        this.confidenceThreshold = confidenceThreshold;
        detector.setMinResultConfidence(confidenceThreshold);
    }

    public float getConfidenceThreshold () {
        return confidenceThreshold;
    }

    /**
     * Temporarily disable the detector (to save CPU resources)
     */
    public void disable () {
        visionPortal.setProcessorEnabled(detector, false);
    }

    /**
     * Re-enable the detector
     */
    public void enable () {
        visionPortal.setProcessorEnabled(detector, true);
    }

    /**
     * Permanently close the detector
     */
    public void close() {
        visionPortal.close();
    }

    // ********************************************************************************************
    // **************************** BEGIN DETECTOR METHODS ****************************************
    // ********************************************************************************************

    /**
     * Get the number of Recognitions since last updating Recognitions
     * @return an int of the number of Recognitions currently found.
     */
    public int getNumRecognitions() {
        return currentRecognitions.size();
    }

    /**
     * Get the position of ALL recognitions in the format of a 2-D array:
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

    /**
     * Get the position of ALL the Recognitions with a given label in the form of a 2-D array:
     * [[x1, y1],
     * [x2,y2]]
     * @param label the label to search for
     * @return the 2-D array shown above
     */
    public double[][] getRecognitionPositions(String label) {
        int numRecognitions = 0;
        List<Recognition> correctRecognitions = new ArrayList<>();
        for (int i = 0; i < currentRecognitions.size(); i++) {
            Recognition recognition = currentRecognitions.get(i);
            if (recognition.getLabel().equals(label)) {
                numRecognitions ++;
                correctRecognitions.add(recognition);
            }
        }

        double [][] recognitionPositions = new double[numRecognitions][2];
        for (int i = 0; i < numRecognitions; i++) {
            Recognition recognition = correctRecognitions.get(i);
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
     * @param i the index of the recognition to get the coordinates of, as an int
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

    /**
     * Get the position of a SINGLE recognition with the given label
     * @param label the label to be searched for, as a String
     * @return a double[] of length 2 in the format [x,y]
     */
    public double[] getRecognitionPosition(String label) {
        double[] recognitionPosition = new double[2];
        Recognition recognition = getRecognition(label);

        double x = (recognition.getLeft() + recognition.getRight()) / 2;
        double y = (recognition.getTop() + recognition.getBottom()) / 2;
        recognitionPosition[0] = x;
        recognitionPosition[1] = y;

        return recognitionPosition;
    }

    /**
     * get a recognition with a given index
     * @param i the index of the recognition to return
     * @return a Recognition
     */
    public Recognition getRecognition(int i) {
        checkValidIndex(i);
        return currentRecognitions.get(i);
    }

    /**
     * Get a SINGLE recognition with the given label.
     * @param label the label to search for.
     * @return the Recognition if one such Recognition was found, null otherwise.
     */
    public Recognition getRecognition (String label) {
        for (Recognition recognition : currentRecognitions) {
            if (recognition.getLabel().equals(label)) {
                return recognition;
            }
        }
        return null;
    }

    /**
     * Get all current Recognitions in the form of an ArrayList.
     * @return An ArrayList of all current Recognitions.
     */
    public List<Recognition> getAllRecognitions () {
        return currentRecognitions;
    }

    /**
     * Get ALL current recognitions with a given label in the form of an ArrayList.
     * @param label the label to search for.
     * @return an ArrayList of all Recognitions that match the label passed. Will return an empty
     *         ArrayList if no such Recognition is found.
     */
    public List<Recognition> getAllRecognitions (String label) {
        List<Recognition> recognitions = new ArrayList<>();
        for (Recognition recognition : currentRecognitions) {
            if (recognition.getLabel().equals(label)) {
                recognitions.add(recognition);
            }
        }
        return recognitions;
    }

    /**
     * get the index of a SINGLE recognition with a given label
     * @param label the label to search for
     * @return the index as an int of the recognition with that label
     */
    public int getRecognitionIndex (String label) {
        for (int i = 0; i < getNumRecognitions(); i++) {
            if (currentRecognitions.get(i).getLabel().equals(label)) {
                return i;
            }
        }
        return -1;
    }

    /**
     * get the index of ALL recognitions with a given label
     * @param label the label to search for
     * @return An ArrayList<Integer> with the indexes where the label was found. Will return an
     *         empty ArrayList if no such recognitions were found.
     */
    public List<Integer> getRecognitionIndexes (String label) {
        List <Integer> recognitionIndexes= new ArrayList<>();
        for (int i = 0; i < getNumRecognitions(); i++) {
            if (currentRecognitions.get(i).getLabel().equals(label)) {
                recognitionIndexes.add(i);
            }
        }
        return recognitionIndexes;
    }

    /**
     * returns the Recognition with the highest confidence
     * @return A Recognition with the highest confidence out of all recognitions
     */
    public Recognition getHighestConfidenceRecognition() {
        if (getNumRecognitions() == 0) {
            return null;
        }
        float maxConfidence = -1f;
        Recognition highestConfidence = getRecognition(0);
        for (int i = 0; i < getNumRecognitions(); i++) {
            if (getRecognition(i).getConfidence() > maxConfidence) {
                maxConfidence = getRecognition(i).getConfidence();
                highestConfidence = getRecognition(i);
            }
        }
        return highestConfidence;
    }

    /**
     * returns the Recognition with the highest confidence
     * @return A Recognition with the highest confidence out of all recognitions
     */
    public Recognition getHighestConfidenceRecognition(String label) {
        if (getNumRecognitions() == 0) {
            return null;
        }

        List<Recognition> recognitionsWithCorrectLabel = getAllRecognitions(label);
        float maxConfidence = -1f;
        Recognition highestConfidence = recognitionsWithCorrectLabel.get(0);
        for (int i = 0; i < recognitionsWithCorrectLabel.size(); i++) {
            if (recognitionsWithCorrectLabel.get(i).getConfidence() > maxConfidence) {
                maxConfidence = recognitionsWithCorrectLabel.get(i).getConfidence();
                highestConfidence = recognitionsWithCorrectLabel.get(i);
            }
        }
        return highestConfidence;
    }

    // ********************************************************************************************
    // **************************** BEGIN PRIVATE METHODS *****************************************
    // ********************************************************************************************

    /**
     * Initialize the detector with the given model name. Only needs to be run once
     * @param modelName the name of the model trained (ie. the name of the tflite file).
     *                  EX: "Blue_Cone_Test_Model.tflite"
     */
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

    private void checkValidIndex(int i) {
        if (i >= getNumRecognitions()) {
            throw new IndexOutOfBoundsException("There exists no Recognition with that index");
        } else if (i < 0) {
            throw new IllegalArgumentException("The index cannot be a negative number");
        }
    }

}