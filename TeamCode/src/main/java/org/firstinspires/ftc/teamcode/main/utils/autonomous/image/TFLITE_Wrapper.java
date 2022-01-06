package org.firstinspires.ftc.teamcode.main.utils.autonomous.image;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.main.utils.resources.Resources;

import java.util.ArrayList;
import java.util.List;

public class TFLITE_Wrapper {
    private final String VUFORIA_KEY;
    private final String CAMERA_NAME;
    private ArrayList<Detection> Detections;
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    public String TFOD_MODEL_ASSET;
    public String[] LABELS;
    public HardwareMap hardwareMap;
    public float confidence = 0.5f;

    public TFLITE_Wrapper(HardwareMap hardwareMap) {
        this(   "AcQbfNb/////AAABmUoZxvy9bUCeksf5rYATLidV6rQS+xwgakOfD4C+LPj4FmsvqtRDFihtnTBZUUxxFbyM7CJMfiYTUEwcDMJERl938oY8iVD43E/SxeO64bOSBfLC0prrE1H4E5SS/IzsVcQCa9GsNaWrTEushMhdoXA3VSaW6R9KrrwvKYdNN/SbaN4TPslQkTqSUr63K60pkE5GqpeadAQuIm8V6LK63JD1TlF665EgpfsDZeVUBeAiJE86iGlT1/vNJ9kisAqKpBHsRyokaVClRnjlp28lmodjVRqeSk8cjCuYryn74tClfxfHQpkDDIsJO+7IYwJQCZQZZ+U9KJaMUeben4HOj0JTnQaEE6MZLaLQzY+C/6MS",
                "FreightFrenzy_BC.tflite",
                new String[]{"Ball", "Cube"},
                hardwareMap,
                Resources.Misc.Webcam
        );
    }

    public TFLITE_Wrapper(String VUFORIA_KEY, String TFOD_MODEL_ASSET, String[] LABELS, HardwareMap hardwareMap, String cameraName) {
        this.VUFORIA_KEY = VUFORIA_KEY;
        this.TFOD_MODEL_ASSET = TFOD_MODEL_ASSET;
        this.LABELS = LABELS;
        this.hardwareMap = hardwareMap;
        this.CAMERA_NAME = cameraName;
    }

    public void init() {
        initVuforia();
        initTFOD();
    }

    public void setZoom(double magnification, double aspectRatio) {
        tfod.setZoom(magnification, aspectRatio);
    }

    public void activate() {
        tfod.activate();
    }

    public List<Recognition> getUpdatedRecognitions() {
        return tfod.getUpdatedRecognitions();
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, CAMERA_NAME);

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTFOD() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = confidence;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    public ArrayList<Detection> getDetections() {
        return Detections;
    }

    public void clearDetections() {
        Detections.clear();
    }

    public void registerDetection(Detection detection) {
        Detections.add(detection);
    }

    public ArrayList<Detection> searchDetections(String name) {
        ArrayList<Detection> returnList = new ArrayList();
        for (Detection detection : Detections) {
            if (detection.friendlyName.equals(name)) {
                returnList.add(detection);
            }
        }
        return returnList;
    }

    public class Detection {
        public Detection(float x, float y, float imageHeight, float imageWidth, String friendlyName, float angle) {
            this.x = x;
            this.y = y;
            this.imageHeight = imageHeight;
            this.imageWidth = imageWidth;
            this.friendlyName = friendlyName;
            this.angle = angle;
        }

        public float x;
        public float y;
        public float imageHeight;
        public float imageWidth;
        public String friendlyName;
        public float angle;
    }
}
