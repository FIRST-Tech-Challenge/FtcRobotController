package vision;

import com.google.blocks.ftcrobotcontroller.runtime.obsolete.VuforiaLocalizerAccess;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.ClassFactory;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection, using
 * the easiest way.
 *
 * Totally my code
 */

@TeleOp
public class TensorFlowObjectDetection extends LinearOpMode {

    int button2X = 0;
    int button2A = 0;
    int button2B = 0;
    int button2Y = 0;
    int buttonA = 0;
    int buttonB = 0;
    int buttonX = 0;
    int buttonY = 0;

    boolean but2Acheck = false;
    boolean but2Ycheck = false;
    boolean but2Xcheck = false;
    boolean but2Bcheck = false;

    boolean butAcheck = false;
    boolean butYcheck = false;
    boolean butXcheck = false;
    boolean butBcheck = false;

    int selectedmodel = 1;

    private static final String[] LABELS = {
            "blueobject",
            "redobject",
    };

    /* The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        if (gamepad2.a && !but2Acheck) {
            visionPortal.close();
            button2A += 1;
            but2Acheck = true;
        }
        if (!gamepad2.a) {
            but2Acheck = false;
        }

        if (!but2Acheck) {
            selectedmodel = button2A-button2B;
        }

        if (gamepad2.b && !but2Bcheck) {
            visionPortal.close();
            button2B += 1;
            but2Bcheck = true;
        }
        if (!gamepad2.b) {
            but2Bcheck = false;
        }

        if (!but2Bcheck) {
            selectedmodel = button2A-button2B;
        }

        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        telemetry.addData("Selected Model", "T-T.tflite");

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double imagecentreX = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double imagecentreY = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData("Camera 1"," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", imagecentreX, imagecentreY);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        tfod = new TfodProcessor.Builder()
                .setModelAssetName("T-T.tflite")
                .setModelLabels(LABELS)
                .build();

        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);

    }   // end method initTfod()
}   // end class
