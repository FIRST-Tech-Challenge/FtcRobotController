package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ebotsenums.Alliance;
import org.firstinspires.ftc.teamcode.ebotsenums.BarCodePosition;
import org.firstinspires.ftc.teamcode.ebotsutil.AllianceSingleton;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.opencvpipelines.BarCodeScanner;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsColorSensor;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class StateOpenCVObserve implements EbotsAutonState{

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    private EbotsAutonOpMode autonOpMode;
    private Telemetry telemetry;
    StopWatch stopWatch = new StopWatch();
    HardwareMap hardwareMap;
    BarCodeScanner barCodeScanner;
    OpenCvCamera camera;
    private BarCodePosition observation;
    String logTag = "EBOTS";
    int loopCount = 0;

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Constructors
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public StateOpenCVObserve(EbotsAutonOpMode autonOpMode){
        Log.d(logTag, "Instantiating StateOpenCVObserve");
        this.autonOpMode = autonOpMode;
        this.hardwareMap = autonOpMode.hardwareMap;
        this.telemetry = autonOpMode.telemetry;

        BarCodeObservation.resetObservations();
        Log.d(logTag, "Observations reset");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        Log.d(logTag, "cameraMonitorViewId set");

        WebcamName webcamName = autonOpMode.getFrontWebcam().getWebcamName();
        // With live preview
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        Log.d(logTag, "camera instantiated");
        barCodeScanner = new BarCodeScanner();
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                Log.d(logTag, "The camera is now open...");
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(barCodeScanner);

            }
            @Override
            public void onError(int errorCode)
            {
                Log.d(logTag, "There was an error");
            }
        });

        Log.d(logTag, "Constructor complete");


    }

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    private String getColor(int hue){
        String color = "unknown";

        if (hue < 45 | hue > 150){
            color = "Red";
        } else if (hue > 60 && hue < 85){
            color = "Green";
        }
        return color;
    }


    @Override
    public boolean shouldExit() {
        loopCount++;
        //User Request Exit not implemented
        boolean userRequestedExit = autonOpMode.gamepad1.left_bumper && autonOpMode.gamepad1.right_bumper;
        return autonOpMode.isStopRequested() | autonOpMode.isStarted() | userRequestedExit;
    }

    @Override
    public void performStateActions() {
        // This accesses video stream processing occurring on a concurrent OpenCV pipeline
        // The pipeline process loops independent of the state machine
        // New observations are processed only if the pipeline reading has not been consumed
        // The pipeline process manages the consumption state of the reading in readingConsumed
        if (!barCodeScanner.isReadingConsumed()){
            double confidenceThreshold = 0.7f;

            if (AllianceSingleton.isBlue()) {
                // if on the blue side then robot sees middle and right spot
                observation = BarCodePosition.LEFT;
                if (barCodeScanner.getLeftConfidenceRed() > confidenceThreshold) {
                    observation = BarCodePosition.MIDDLE;
                } else if (barCodeScanner.getRightConfidenceRed() > confidenceThreshold) {
                    observation = BarCodePosition.RIGHT;
                }
            } else{
                // on red alliance and seeing left and middle
                observation = BarCodePosition.RIGHT;
                if (barCodeScanner.getLeftConfidenceRed() > confidenceThreshold) {
                    observation = BarCodePosition.LEFT;
                } else if (barCodeScanner.getRightConfidenceRed() > confidenceThreshold) {
                    observation = BarCodePosition.MIDDLE;
                }
            }

            new BarCodeObservation(observation);

            barCodeScanner.markReadingAsConsumed();
        }

        telemetry.addData("Observation Attempts", loopCount);
        telemetry.addData("Cummulative Observation Count", BarCodeObservation.getObservationCount());
        telemetry.addData("Camera Readings", barCodeScanner.getReadingCount());
        telemetry.addData("LeftConfidenceRed", String.format("%.2f", barCodeScanner.getLeftConfidenceRed()));
        telemetry.addData("RightConfidenceRed", String.format("%.2f", barCodeScanner.getRightConfidenceRed()));
        telemetry.addData("Current Observation", observation.name());
        // TODO: next line should be removed after debugging
        telemetry.addData("Barcode Position ", BarCodeObservation.giveBarCodePosition().name());
    }

    @Override
    public void performTransitionalActions() {
        BarCodePosition barCodePosition = BarCodeObservation.giveBarCodePosition();
        autonOpMode.setBarCodePosition(barCodePosition);
        telemetry.addData("Barcode Position ", barCodePosition);
        try {
            camera.stopStreaming();
        } catch (Exception e){
            Log.d(logTag, "There was an exception when shutting down the camera");
        }
    }

}
