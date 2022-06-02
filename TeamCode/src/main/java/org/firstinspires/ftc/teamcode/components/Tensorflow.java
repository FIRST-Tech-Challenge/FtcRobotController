package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.components.Vuforia.CameraChoice;
import org.firstinspires.ftc.teamcode.params.VuforiaParams;


public class Tensorflow {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY = VuforiaParams.VUFORIA_KEY;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public Tensorflow(WebcamName name, int tfodMonitorId) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = name;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorId);
        tfodParameters.minimumConfidence = 0.3;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        tfod.activate();
    }

    public List<Recognition> getInference() {
        if (tfod != null) {
            return tfod.getUpdatedRecognitions();
        }
        return null;
    }

    public void activate() {
        tfod.activate();
    }

    public void shutdown() {
        tfod.shutdown();
    }
}