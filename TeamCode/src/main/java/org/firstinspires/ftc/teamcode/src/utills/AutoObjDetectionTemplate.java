package org.firstinspires.ftc.teamcode.src.utills;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;

import java.util.List;

/**
 * A template for all Autonomous opModes that use Vision, allows easy initialization
 */
@Disabled
public abstract class AutoObjDetectionTemplate extends AutonomousTemplate {

    /**
     * The file that is loaded to detect from
     */
    private static final String TFOD_MODEL_ASSET = "Trained Pink Team Marker Finder Mk2.tflite";

    /**
     * The labels in the file
     */
    private static final String[] LABELS = {"Pink Team Marker"};

    /**
     * The Vuforia licence Key
     */
    public static final String VUFORIA_KEY = "AWVWPbH/////AAABmbzQF0cF/EvRnE4ykZKAXvpbnJrPQs1aBJ2i7u5ADGzYU+x0dxqGlB/G8yCrcY4FP8cPEA1w+xTXCpbFDmlYcKMG6VL/6v+H0Es3H/1f8xpQG86nSCXKPLxEbYGHkBxAYSlxB0gueBpnxMYsURezlq2Q9e5Br5OIhY7gmZZNa3VPHupscQkrCrVdRMI9mPAbEjMBhVBWjVJEL0+u2tyvEQuK4tllgi8C7AKq5V5lFoKEQG0VD89xlgUfRZsDq89HToRXBOUE2mubPHUcplKiX+1EfB+801eEt+k7lLJ1VyfrXr2tjwyWPjafvTpnaf3C35ox0/TOPdak5pq2gXLpXzAxXc6+RH28m2572tYB58AN";

    /**
     * A object to lock on to for the thread safety, used in _initVuforia
     */
    private static final Object VF_Lock = new Object();

    /**
     * A object to lock on to for the thread safety, used in _initTfod
     */
    private static final Object TFOD_Lock = new Object();

    /**
     * vuforia object
     */
    public volatile VuforiaLocalizer vuforia;

    /**
     * Tensorflow Object Detection Object
     */
    public volatile TFObjectDetector tfod;

    /**
     * Initializes the Vuforia object
     *
     * @throws InterruptedException Throws exception if the opMode is stopped during initialization
     */
    public void initVuforia() throws InterruptedException {
        telemetry.addData("Vuforia initialization:", "Started");
        telemetry.update();

        checkStop();
        this._initVuforia();
        checkStop();

        telemetry.addData("Vuforia initialization:", "Complete");
        telemetry.update();

    }

    /**
     * Does the initialization, provides synchronization for thread safety
     *
     * @throws InterruptedException Throws exception if the opMode is stopped during initialization
     */
    private void _initVuforia() throws InterruptedException {
        //does the initialization
        VuforiaLocalizer Vuforia;

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //Waits for mutex to be available

        checkStop();
        synchronized (VF_Lock) {
            this.vuforia = ClassFactory.getInstance().createVuforia(parameters);
        }
        checkStop();


    }

    /**
     * Initializes the Tensorflow Object Detection Object object
     *
     * @throws InterruptedException Throws exception if the opMode is stopped during initialization
     */
    public void initTfod() throws InterruptedException {
        telemetry.addData("TFOD initialization:", "Started");
        telemetry.update();
        checkStop();

        this._initTfod();

        checkStop();
        telemetry.addData("TFOD initialization:", "Complete");
        telemetry.update();

    }

    /**
     * Does the initialization, provides synchronization for thread safety
     */
    private void _initTfod() {
        //Waits for mutex to be available
        synchronized (TFOD_Lock) {
            //Runs initialization Code
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minResultConfidence = 0.8f;
            tfodParameters.isModelTensorFlow2 = true;
            tfodParameters.inputSize = 320;
            this.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            this.tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        }

    }

    /**
     * Activates Tensorflow
     *
     * @throws InterruptedException Throws exception if the opMode is stopped during initialization
     */
    public void activateTF() throws InterruptedException {
        checkStop();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.4, 16.0 / 9.0);
        }
        checkStop();
    }


    /**
     * Initializes all fields provided by this class
     *
     * @throws InterruptedException Throws if OpMode is stopped during execution
     */
    public void initAll() throws InterruptedException {
        initVuforia();
        initTfod();
        activateTF();
        super.initAll();
    }

    /**
     * It uses the camera to determine where the object is on the screen
     *
     * @param arraySize The number of samples to take
     * @param sleepTime How long to wait between each sample
     * @return It returns the average position of all the samples
     * @throws InterruptedException Throws exception if the opMode is stopped during function execution
     */
    public BarcodePositions getAverageOfMarker(int arraySize, int sleepTime) throws InterruptedException {

        BarcodePositions[] markerPositions = new BarcodePositions[arraySize];

        for (int i = 0; i < arraySize; i++) {
            markerPositions[i] = this.findPositionOfMarker();
            Thread.sleep(sleepTime);
            checkStop();
        }

        int sum = 0;
        for (int i = 0; i < arraySize; i++) {
            switch (markerPositions[i]) {
                case NotSeen:
                    break;
                case Right:
                    sum++;
                    break;
                case Left:
                    sum = sum + 2;
                    break;
                case Center:
                    sum = sum + 3;
                    break;
            }
        }

        int result = (int) Math.round(sum / (double) arraySize);


        switch (result) {
            case 1:
                return BarcodePositions.Right;
            case 2:
                return BarcodePositions.Left;
            case 3:
                return BarcodePositions.Center;
            default:
                return BarcodePositions.NotSeen;
        }
    }

    /**
     * Uses the camera to determine where the object is on screen
     *
     * @return Where the marker is
     */
    public BarcodePositions findPositionOfMarker() {
        final int screenWidth = 1260;

        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if ((updatedRecognitions != null) && (updatedRecognitions.size() > 0)) {
            Recognition rec = updatedRecognitions.get(0);
            double rightLeftCenterLine = (rec.getRight() + rec.getLeft()) / 2.0;

            if (rightLeftCenterLine < 450) {
                return BarcodePositions.Left;
            } else if (rightLeftCenterLine < 820) {
                return BarcodePositions.Center;
            } else {
                return BarcodePositions.Right;
            }
        } else {
            return BarcodePositions.NotSeen;
        }
    }

}


