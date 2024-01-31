package org.firstinspires.ftc.teamcode.yise;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class TensorflowVision {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private static final String[] LABELS = {
            "Red Prop",
            "Blue Prop"
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
    // first.
    public TensorflowVision(HardwareMap hw) {
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName("redownloadedmodel.tflite")
                .setModelFileName("modelt.tflite")

                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();


        builder.setCamera(hw.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

        // Wait for the camera to be open
        /*if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while ( (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                //
            }
        }

        // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }

        exposureControl.setExposure(80, TimeUnit.MILLISECONDS);

        // Set Gain.
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(255);*/

    }

    public int getPropPosition() {

        // Wait for the camera to be open
        /*if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while ( (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                //
            }
        }*/

        // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
        /*ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }

        exposureControl.setExposure(100, TimeUnit.MILLISECONDS);

        // Set Gain.
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(255);*/

        List<Recognition> currentRecognitions = tfod.getRecognitions();

        //If no recognitions, it is far right pos
        if (currentRecognitions.isEmpty()) {
            return 2;
        }

        // Step through the list of recognitions and display info for each one.
        double x = 0;
        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight())/2;
        }

        if (x > 300) {
            return 0;
        } else {
            return 1;
        }
    }
}
