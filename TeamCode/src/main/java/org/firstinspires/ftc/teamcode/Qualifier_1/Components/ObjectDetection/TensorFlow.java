/**
 * Tensor Flow Class. This is for detecting the
 * number of rings in autonomous. It uses
 * the Vuforia Engine.
 *
 * @author  Aamod
 * @version 1.0
 * @since   2020-November-5
 * @status: Fully working
 */


package org.firstinspires.ftc.teamcode.Qualifier_1.Components.ObjectDetection;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class TensorFlow extends Thread{
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private int numberOfRings;

    private static final String VUFORIA_KEY =
            "ATUOrmn/////AAABmVLVlWBtWUpnh9+EekIwR4lmMDXtnMrh/37lRyh+1m4oZJv1ANDvpS7D/Es9GNQ0wAkJ4YOHVWFjjsE5ptAFY2NRCAAwEY4VtvXEvSr3j/a0WR54dNfoCHRsnEaL5oQu25MoyOo7VrmhkE3xb2J9cNbsJzeqNaZWdQQpHkrgzEotos4i2tf/z+IMQxQ5nwH7Daiar93yoFv6FKeTh9MfI3bxVKR0nF+vrMzmNPC6YLk3yjqAKLqSgAvV0t07MBz9BjT2r58njS6qCo2U1H3sQXBlUcMdeKi4iclQaM+Oac+mZrzrhMvSEW7gC9mDhoL8l3zf2yMLPV9oGtnirNWn7ov/mupDtDecOUI4MPDNi9dt";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private LinearOpMode op;

    public TensorFlow(LinearOpMode opMode) {
        op = opMode;
    }

    public void initTensorFlow() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = op.hardwareMap.get(WebcamName.class, "WebcamFront");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        int tfodMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
             tfod.setZoom(2.5, 1.78);
        }
    }

    public void runTensorFlow () {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                int recognitions = updatedRecognitions.size();
                op.telemetry.addData("# aa Object Detected", recognitions);

                // step through the list of recognitions and display boundary info.
                numberOfRings = 0;
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    op.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    op.telemetry.addData("Confidence", "%3.3f", recognition.getConfidence());
                    op.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    op.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    if (recognition.getLabel() == "Quad" ) {
                        numberOfRings = 4;
                    }
                    else if (recognition.getLabel() == "Single" ) {
                        numberOfRings = 1;
                    }
                    else{
                        numberOfRings = 0;
                    }
                }
            } else {
                numberOfRings = -1;
                op.telemetry.addData("InElse", 0);
            }
            //op.telemetry.update();
        }
    }

    public int getNumberOfRings () {
        return numberOfRings;
    }

    public void stopTensorFlow () {
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}