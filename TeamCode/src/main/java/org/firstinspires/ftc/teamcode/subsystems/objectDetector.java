package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.robotcore.external.tfod.TfodParameters.CurrentGame.LABELS;

import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;


public class objectDetector implements Subsystem {
    public static int    NUM_TRY = 10;
    public static double DUCK_LEFT_THRESHOLD = 800;
    public static double CONFIDENCE = 0.7;
    public static String LABEL_STR = "TSE"; //"Duck"

    private static final boolean USE_WEBCAM = true;
    //private TfodCurrentGame tfodFreightFrenzy;
    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    Recognition recognition;
    Telemetry telemetry;
    Robot     robot;


    private void initTfod() {
        String LOCAL_MODEL_ASSET = "model_20231126_redCube.tflite";
        String[] LOCAL_LABELS = {
                "redCube"
        };

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(LOCAL_MODEL_ASSET)
                .setModelLabels(LOCAL_LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(robot.hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    public void close() {
        visionPortal.stopStreaming();
        tfod.shutdown();
    }



    private void displayInfo (int i, Recognition recognition){
        // Display label info.
        // Display the label and index number for the recognition.
        Log.i("Index", "%0d" + i);
        Log.i("Label %s", recognition.getLabel());
        // Display upper corner info.
        // Display the location of the top left corner
        // of the detection boundary for the recognition
        Log.i("Left, Top ", Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getTop(), 0)));
        // Display lower corner info.
        // Display the location of the bottom right corner
        // of the detection boundary for the recognition
        Log.i("Right, Bottom ", Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0)) + ", " + Double.parseDouble(JavaUtil.formatNumber(recognition.getBottom(), 0)));
        // Display Recognition of Duck or Team Element
        if (recognition.getLabel().equals("Duck")||recognition.getLabel().equals("Cube") ) {

            telemetry.addData("Object Detected: Duck", "Duck");
        } else {
        }
    }



    // if Lable is Duck, and left < threshold, pos = 1
    // if Label is Duck, and left >= threshold pos = 2
    // if Label is not Duck, or nothing, pos = 3
    // recognitions.size() == 0
    // recognition.getLabel().equals("Duck")
    // recognition.getLeft()
    public int checkDuckPresence() {
        int pos = 3;
        List<Recognition> recognitions;
        int index;

        double left = 0;
        double right=0;
        double middle = 0;
        boolean foundDuck = false;

        for (int i=0; i< NUM_TRY; i++) {
            recognitions = tfod.getRecognitions();
            //If list is empty, inform the user. Otherwise, go
            //through list and display info for each recognition.
            if (recognitions.size() == 0) {
                telemetry.addData("TFOD", "No items detected.");
                pos = 3;
            } else {

                // Iterate through list and check if Label is "Duck"
                // If Lable found, check left, break
                index = 0;
                for (Recognition recognition_item : recognitions) {
                    recognition = recognition_item;
                    // Display info.
                    displayInfo(index, recognition_item);

                    telemetry.addData("Label", recognition.getLabel());
                    // Check if label is Duck
                    if (recognition.getLabel().equals(LABEL_STR)) {
                        left = Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0));
                        right = Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0));
                        middle = (left + right)/2.0;
                        telemetry.addData("middle", middle);
                        if (middle < DUCK_LEFT_THRESHOLD) {
                            pos = 1;
                        } else {
                            pos = 2;
                        }
                        foundDuck = true;
                        break;
                    }
                    // Increment index.
                    index = index + 1;
                }
                if (foundDuck == false) {
                    pos = 3;
                }
            }
            //sleep(10);
            telemetry.update();
        }



        return (pos);
    }


    @Override
    public void update(TelemetryPacket packet) {

    }
}