package org.firstinspires.ftc.teamcode.src.robotAttachments.Sensors.Camera;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.tfod.TFObjectDetectorImpl;

public class TFOD extends TFObjectDetectorImpl {
    private static final String TFOD_MODEL_ASSET = "Trained Pink Team Marker Finder.tflite";
    private static final String[] LABELS = {"Pink Team Marker"};
    HardwareMap hardwareMap;
    VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public TFOD(HardwareMap hardwareMap, Parameters parameters, VuforiaLocalizer vuforiaLocalizer) {

        super(parameters, vuforiaLocalizer);
        hardwareMap = hardwareMap;
        vuforia = vuforiaLocalizer;

    }

    protected void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
