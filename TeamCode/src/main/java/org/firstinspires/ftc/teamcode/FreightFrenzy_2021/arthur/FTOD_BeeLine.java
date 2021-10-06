package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.arthur;

import android.content.res.Resources;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

@TeleOp(name = "FT Object Detection: BeeLine", group = "Linear OpMode")
public class FTOD_BeeLine extends LinearOpMode {
    private static final String VUFORIA_KEY = "AbbXw07/////AAABmb3Btlsgk0kds/dIfBHuXJpPmJG5blwxr150leKi1IwYvrDK9zsrm72lOhm7a6szhUJLWov316p/3cDMerJne7Ah7GicPOIq5pgAEpOEnFZwtKwwF3xnAZ4Spy9J0TYgeDR6qkcnAQJUdOKfICqCumWMIPJS/LP2F8NS083qqTJQ68bD1D7wxuCCtYfVdtEPNuj1M4rGb3uhWTRHjSO5yjVrkjV+iBnnQoaDLSXHNH0pp5VY5PbBj734yCf1TF6YmBuaC9tbFNEZUmCmTjgsLBXqIGD6LTQmiZa/pay1+NIv0kv/GfWjk9Q6MwhEMg9oDXuEv6jMJmNHEECJga/iiUKepl7OFj5PpvyrlrBsfTtD";

    private static final String TFOD_MODEL = "FreightFrenzy_BCDM.tflite";
    private static final String[] TFOD_LABELS = {"Ball", "Cube", "Duck", "Marker"};

    private VuforiaLocalizer vuforia;
    private TFObjectDetector detector;

    @Override
    public void runOpMode() {
        initVuforia();
        initDetector();

        // wait until the user pressed
        waitForStart();
        // only go into the loop if the detector has been initialized yet
        while (opModeIsActive() && detector != null) {
            execute();
        }
    }

    // the function of a single loop execution
    public void execute() {
        displayRecognitions();
    }

    public void displayRecognitions() {
        List<Recognition> updates = detector.getUpdatedRecognitions();
        boolean hasUpdated = updates != null;
        if (hasUpdated) { // only re-render telemetry data if there actually were updates
            for (int i = 0; i < updates.size(); i++) {
                Recognition curr = updates.get(i);
                telemetry.addLine(String.format(Locale.ENGLISH, "label no.%d: '%s'", i, curr.getLabel()));
                float left = curr.getLeft(), right = curr.getRight(), top = curr.getTop(), bottom = curr.getBottom();
                telemetry.addLine(String.format(Locale.ENGLISH, "left: %f, right: %f, top: %f, bottom: %f", left, right, top, bottom));
            }
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();
        // use the vuforia license for configuration
        params.vuforiaLicenseKey = VUFORIA_KEY;
        // use the camera from the hardware map
        params.cameraName = hardwareMap.get(WebcamName.class, "webcam");
        // create the vuforia instance
        vuforia = ClassFactory.getInstance().createVuforia(params);
    }

    // requires initVuforia to have run before
    private void initDetector() {
        // retrieve the resources and the package name of the application
        Resources res = hardwareMap.appContext.getResources();
        String pkg = hardwareMap.appContext.getPackageName();
        int viewID = res.getIdentifier("TFODMonitorViewID", "id", pkg);
        TFObjectDetector.Parameters params = new TFObjectDetector.Parameters(viewID);
        // configure the data
        params.minResultConfidence = 0.8f;
        params.isModelTensorFlow2 = true;
        params.inputSize = 320;
        // create the TF object detector instance
        detector = ClassFactory.getInstance().createTFObjectDetector(params, vuforia);
        detector.loadModelFromAsset(TFOD_MODEL, TFOD_LABELS);
    }
}
