package org.firstinspires.ftc.teamcode.src.robotAttachments.Sensors.Camera;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.tfod.TFObjectDetectorImpl;

public class TFOD extends TFObjectDetectorImpl {

    // This class will access the file and label for the model made with the FTC ML tool

    // Trained Pink Team Marker Finder Mk2.tflite is the file for the model of our team marker for the Freight Frenzy season meet 2
    // "Pink Team Marker" is the label for our team marker model as of meet 2 of Freight Frenzy

    protected static final String TFOD_MODEL_ASSET = "Trained Pink Team Marker Finder Mk2.tflite";
    protected static final String[] LABELS = {"Pink Team Marker"};
    HardwareMap hardwareMap;
    VuforiaLocalizer vuforia;
    TFObjectDetector.Parameters tfodParameters;

    private TFObjectDetector tfod;

    public TFOD(TFObjectDetector.Parameters tfodParameters, VuforiaLocalizer vuforiaLocalizer, HardwareMap hardwareMap) {

        super(tfodParameters, vuforiaLocalizer);
        this.hardwareMap = hardwareMap;
        vuforia = vuforiaLocalizer;
        this.tfodParameters = tfodParameters;

    }


    public void initTfod() {
       /* int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

        */
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
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
            tfod.setZoom(2.5, 16.0 / 9.0);
        }
    }
}
