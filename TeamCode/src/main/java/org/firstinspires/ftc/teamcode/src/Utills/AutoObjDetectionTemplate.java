package org.firstinspires.ftc.teamcode.src.Utills;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.concurrent.locks.ReentrantLock;


@Disabled
public abstract class AutoObjDetectionTemplate extends AutonomousTemplate {

    private static final String TFOD_MODEL_ASSET = "Trained Pink Team Marker Finder Mk2.tflite";
    private static final String[] LABELS = {"Pink Team Marker"};

    private static final String VUFORIA_KEY = "AWVWPbH/////AAABmbzQF0cF/EvRnE4ykZKAXvpbnJrPQs1aBJ2i7u5ADGzYU+x0dxqGlB/G8yCrcY4FP8cPEA1w+xTXCpbFDmlYcKMG6VL/6v+H0Es3H/1f8xpQG86nSCXKPLxEbYGHkBxAYSlxB0gueBpnxMYsURezlq2Q9e5Br5OIhY7gmZZNa3VPHupscQkrCrVdRMI9mPAbEjMBhVBWjVJEL0+u2tyvEQuK4tllgi8C7AKq5V5lFoKEQG0VD89xlgUfRZsDq89HToRXBOUE2mubPHUcplKiX+1EfB+801eEt+k7lLJ1VyfrXr2tjwyWPjafvTpnaf3C35ox0/TOPdak5pq2gXLpXzAxXc6+RH28m2572tYB58AN";

    public volatile VuforiaLocalizer vuforia;
    public volatile TFObjectDetector tfod;


    public void initVuforia() throws InterruptedException {
        telemetry.addData("Vuforia initialization:", "Started");
        telemetry.update();

        checkStop();
        this._initVuforia();
        checkStop();

        telemetry.addData("Vuforia initialization:", "Complete");
        telemetry.update();

    }

    private void _initVuforia() throws InterruptedException {
        //Waits for mutex to be available

        synchronized (InitThreadLockContainer.VF_Lock) {

            //does the initialization
            VuforiaLocalizer Vuforia;

            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

            //  Instantiate the Vuforia engine
            checkStop();
            Vuforia = ClassFactory.getInstance().createVuforia(parameters);
            checkStop();


            //Passes initialized obj back to caller class
            this.vuforia = Vuforia;
        }

    }

    public void initTfod() throws InterruptedException {
        telemetry.addData("TFOD initialization:", "Started");
        telemetry.update();
        checkStop();

        this._initTfod();

        checkStop();
        telemetry.addData("TFOD initialization:", "Complete");
        telemetry.update();

    }

    private void _initTfod() {
        //Waits for mutex to be available
        synchronized (InitThreadLockContainer.TFOD_Lock) {
            //Runs initialization Code
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector Tfod;
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minResultConfidence = 0.8f;
            tfodParameters.isModelTensorFlow2 = true;
            tfodParameters.inputSize = 320;
            Tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            Tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);


            //Passes initialized obj back to caller class
            this.tfod = Tfod;
        }

    }

    public void activate() throws InterruptedException {
        checkStop();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2, 16.0 / 9.0);
        }
        checkStop();
    }

    public MarkerPosition getAverageOfMarker(int arraySize, int sleepTime) throws InterruptedException {

        MarkerPosition[] markerPositions = new MarkerPosition[arraySize];

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
            }
        }

        int result = (int) Math.round(sum / (double) arraySize);


        switch (result) {
            case 0:
                return MarkerPosition.NotSeen;
            case 1:
                return MarkerPosition.Right;
            case 2:
                return MarkerPosition.Left;
        }
        return MarkerPosition.Left; //It never reaches this line
    }

    public MarkerPosition findPositionOfMarker() {
        if (tfod != null) {

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null && (updatedRecognitions.size() > 0)) {
                if (updatedRecognitions.get(0).getLeft() > 615) {
                    return MarkerPosition.Right;
                }
                if (updatedRecognitions.get(0).getLeft() <= 615) {
                    return MarkerPosition.Left;
                }
            }
        }
        return MarkerPosition.NotSeen;
    }

    private static class InitThreadLockContainer {
        private static final ReentrantLock VF_Lock = new ReentrantLock();
        private static final ReentrantLock TFOD_Lock = new ReentrantLock();
    }

    public enum MarkerPosition {
        Right,
        Left,
        NotSeen
    }
}


