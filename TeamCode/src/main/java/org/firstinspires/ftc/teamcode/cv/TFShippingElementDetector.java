package org.firstinspires.ftc.teamcode.cv;

import android.os.Environment;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.globals.Levels;
import org.opencv.core.Point;
import org.opencv.core.Rect;

import java.util.List;

public class TFShippingElementDetector {

    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/Vuforia/models/ftc_ml_tse_g_o_20220124_234629.tflite";
    private static final String[] LABELS = {
            "TSE_Green",
            "TSE_Orange"
    };

    private WebcamName webCamName;
    private static final String VUFORIA_KEY =
            "AWTJEXH/////AAABmbuVZOxvSE4FlyBk+KqcsosKyyBW4u6IeGmWn9xhW5LSSyEnwY5onmj8zZoi9hrQtpH8yqnsQN4mjGhEXfA1GGsIdnwFblzJ5RSVMCdoFBb9hR88M4kzu40QMEpM159aXk5wHLpWjUaIh1V8x4rcDZI0X9//Yw5oTvc5k7IS+w0mB2P2282wjFqSrM7Fsq7B37XYwfm74aFEbKQuPSXM3y73gVl1kFgHvdjF95eDkQw8pN/Y5/75fc+S8VXjhUxCnrx3jnCjRVPgzkrW6r3sgonNNXYrhixp1GX66GF2N/5egEmt0e4iQbBfC+nEk/iZ6TyXR839XGFxv16HAwjRUOwFj1FrpZbQhX9uQlZGHhNb";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private static final String ID_NAME = "cameraMonitorViewId";
    private static final String ID_DEF_TYPE = "id";

    private static final float MIN_RESULT_CONFIDENCE = 0.8f;
    private static final boolean IS_MODEL_TF2 = true;
    private static final int FRAME_SIZE = 320;

    private Rect maxRect = new Rect(600,1,1,1);

    private Telemetry telemetry;


    public TFShippingElementDetector(HardwareMap hwMap, VuforiaLocalizer vf){

        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                ID_NAME, ID_DEF_TYPE, hwMap.appContext.getPackageName());

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = MIN_RESULT_CONFIDENCE;
        tfodParameters.isModelTensorFlow2 = IS_MODEL_TF2;
        tfodParameters.inputSize = FRAME_SIZE;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);

        vuforia = vf;
    }

    public TFShippingElementDetector(HardwareMap hwMap, VuforiaLocalizer vf, Telemetry telemetry){

        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                ID_NAME, ID_DEF_TYPE, hwMap.appContext.getPackageName());

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = MIN_RESULT_CONFIDENCE;
        tfodParameters.isModelTensorFlow2 = IS_MODEL_TF2;
        tfodParameters.inputSize = FRAME_SIZE;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

        vuforia = vf;

        this.telemetry = telemetry;
    }


    public void activate(){
        if(tfod != null)
            tfod.activate();
    }

    public void zoom(double maginification,double aspectRation){
        if(tfod != null)
            tfod.setZoom(maginification, aspectRation);
    }

    public void detectLevel(){
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            // step through the list of recognitions and display boundary info.
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {

                String objLabel = recognition.getLabel();
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());

                if(objLabel == "TSE_Orange" || objLabel == "TSE_Green")
                {
                    maxRect = new Rect(new Point(recognition.getTop(),recognition.getLeft()),
                            new Point(recognition.getRight(),recognition.getBottom()));

                    if( getRectMidpointXY().x > 80 &&  getRectMidpointXY().x < 159 ) {
                        telemetry.addData("Found Level","Level One found");
                        Levels.getInstance().setTSELocation(Levels.TSELocation.LEVEL_1);

                    } else if( getRectMidpointXY().x > 160 &&  getRectMidpointXY().x < 220 ) {
                        telemetry.addData("Found Level","Level Two found");
                        Levels.getInstance().setTSELocation( Levels.TSELocation.LEVEL_2);
                    } else {
                        telemetry.addData("Found Level","Level Three found");
                        Levels.getInstance().setTSELocation(Levels.TSELocation.LEVEL_3);

                    }

                }
                telemetry.update();

                i++;
            }
        }
    }

    private int getRectHeight() {
        return maxRect.height;
    }

    private int getRectWidth() {
        return maxRect.width;
    }

    private int getRectX() {
        return maxRect.x;
    }

    private int getRectY() {
        return maxRect.y;
    }

    private double getRectMidpointX() {
        return getRectX() + (getRectWidth() / 2.0);
    }

    private double getRectMidpointY() {
        return getRectY() + (getRectHeight() / 2.0);
    }

    private Point getRectMidpointXY() {
        return new Point(getRectMidpointX(), getRectMidpointY());
    }
}
