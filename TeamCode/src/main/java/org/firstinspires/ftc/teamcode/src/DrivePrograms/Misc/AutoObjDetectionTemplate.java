package org.firstinspires.ftc.teamcode.src.DrivePrograms.Misc;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.src.Utills.AutonomousTemplate;

import java.util.List;


public class AutoObjDetectionTemplate extends AutonomousTemplate {


    private static final String TFOD_MODEL_ASSET = "Trained Pink Team Marker Finder.tflite";
    private static final String[] LABELS = {"Pink Team Marker"};


    private static final String VUFORIA_KEY =
            "AWVWPbH/////AAABmbzQF0cF/EvRnE4ykZKAXvpbnJrPQs1aBJ2i7u5ADGzYU+x0dxqGlB/G8yCrcY4FP8cPEA1w+xTXCpbFDmlYcKMG6VL/6v+H0Es3H/1f8xpQG86nSCXKPLxEbYGHkBxAYSlxB0gueBpnxMYsURezlq2Q9e5Br5OIhY7gmZZNa3VPHupscQkrCrVdRMI9mPAbEjMBhVBWjVJEL0+u2tyvEQuK4tllgi8C7AKq5V5lFoKEQG0VD89xlgUfRZsDq89HToRXBOUE2mubPHUcplKiX+1EfB+801eEt+k7lLJ1VyfrXr2tjwyWPjafvTpnaf3C35ox0/TOPdak5pq2gXLpXzAxXc6+RH28m2572tYB58AN";

    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;


    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        telemetry.addData("Vuforia initialization:", "Complete");
        telemetry.update();

    }

    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

        telemetry.addData("TFOD initialization:", "Complete");
        telemetry.update();

    }

    public void activate() {
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2, 16.0 / 9.0);
            //}
        }
        // public void deactivate(){}
    }

    public MarkerPosition findPositionOfMarker() {
        if (tfod != null) {

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null && (updatedRecognitions.size() > 0)) {
                if (updatedRecognitions.get(0).getTop() > 500) {
                    return MarkerPosition.Right;
                }
                if (updatedRecognitions.get(0).getTop() <= 500) {
                    return MarkerPosition.Center;
                }
            }
        }
        return MarkerPosition.Left;
    }

    public MarkerPosition getAverageOfMarker(int arraySize, int sleepTime) throws InterruptedException {

        MarkerPosition[] markerPositions = new MarkerPosition[arraySize];

        for (int i = 0; i < arraySize; i++) {
            markerPositions[i] = this.findPositionOfMarker();
            Thread.sleep(sleepTime);
        }

        int sum = 0;
        for (int i = 0; i < arraySize; i++) {
            switch (markerPositions[i]) {
                case Left:
                    sum = sum;
                    break;
                case Right:
                    sum++;
                    break;
                case Center:
                    sum = sum + 2;
                    break;
            }
        }

        int result = (int) Math.round(sum / (double) arraySize);


        switch (result) {
            case 0:
                return MarkerPosition.Left;
            case 1:
                return MarkerPosition.Right;
            case 2:
                return MarkerPosition.Center;
        }
        return MarkerPosition.Left; //It never reaches this line
    }

    enum MarkerPosition {
        Right,
        Center,
        Left
    }
}


