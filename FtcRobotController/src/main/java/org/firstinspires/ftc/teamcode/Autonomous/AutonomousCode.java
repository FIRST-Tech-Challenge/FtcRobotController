package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;


/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp
        (name = "madnessAutonomous", group = "Concept")
public class AutonomousCode                                extends LinearOpMode {
    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "ARHVqML/////AAABmdwV3COoyUo6te5Z9nV9Xbs58R8vE55rTErE0ztbuXhfaoos0oD/3ZcFBeJ+b0gLISGqWdDOBM9m4cv6rMlzbJ2qLTB9KX5EpbWfKO2fo9LUIYHLWbre2dui4BfhgLuvKxP8nT/yBsEjAUVz61Bzf3gIEFPTaF8jAnVLwUmYO2Y7/8bXyCNTCoYnC74qHS9D0mqbg+LlGVelz4Zg3zpFfIgwYi56uvaTpdVAxYmPao5JQ0h9FJYLuvfPs9znZEU6QNkS83GVoRm5/cd4S52lWr1jcoeFWg2Haqn7wxKfFGgS7fB41O9wxOe/FHO5Yz4RV0jfYp7M97PxUOvE+c6tOipsSIJnL0aZwYHPRBbX48jA ";

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

//    DcMotor motoryay;
    DcMotor topLeftMotor;
    DcMotor topRightMotor;
    DcMotor bottomLeftMotor;
    DcMotor bottomRightMotor;
    Boolean recognized;
    TouchSensor touchsensor1;
    TouchSensor touchsensor2;
    int numOfMarker;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        //motoryay = hardwareMap.get(DcMotor.class, "motor1");
        topLeftMotor = hardwareMap.get(DcMotor.class, "tLMotor");
        topRightMotor = hardwareMap.get(DcMotor.class, "tRMotor");
        bottomLeftMotor = hardwareMap.get(DcMotor.class, "bLMotor");
        bottomRightMotor = hardwareMap.get(DcMotor.class, "bRMotor");
        touchsensor1 = hardwareMap.get(TouchSensor.class, "touchSensor1");
        touchsensor2 = hardwareMap.get(TouchSensor.class, "touchSensor2");


        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
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

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                recognized = false;
                while (recognized == false) {
                    while (tfod == null){
                        bottomLeftMotor.setPower(0.1);
                        bottomRightMotor.setPower(0.1);
                        topLeftMotor.setPower(0.1);
                        topRightMotor.setPower(0.1);
                    }
                    if (tfod != null) {
                        bottomLeftMotor.setPower(0.0);
                        bottomRightMotor.setPower(0.0);
                        topLeftMotor.setPower(0.0);
                        topRightMotor.setPower(0.0);
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            if (updatedRecognitions.size() == 0) {
//                                motoryay.setPower(0.0);
                            }
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                                i++;
                                if (recognition.getLabel().equals("Duck")  /** && marker position is number relative to the right**/) {
//                                    motoryay.setPower(1.0);
//                                    telemetry.addData("Motor power: ", motoryay.getPower());
                                    autonomousDuck(recognition);
                                    recognized = true;
                                } else if (recognition.getLabel().equals("teamMarker")){ //Will copy Greg's teamMarker code
//                                    motoryay.setPower(0.0);
//                                    telemetry.addData("Motor power: ", motoryay.getPower());
                                    autonomousTeamMarker(recognition);
                                    recognized = true;
                                }
                            }
                            telemetry.update();
                        }
                    }
                }
            }
        }
    }
    /**
     * If it recognizes block, start this function
     * @param recognition
     */
    public void autonomousDuck(Recognition recognition){
        /**
         *    if (recognition.getRight > 0){
         *             Move up a bit rotate 90 degrees to the right, then drive stragitforward towards the middle, then rotate again 90 degrees to the right and drive right before the goal
         *             releaseHigh();
         *             cargo();
         *             parking();
         *
         *         }
         *         else if (recogntion.getLeft() < 0){
         *             Move up a bit rotate 90 degrees to the right, then drive stragitforward towards the middle, then rotate again 90 degrees to the right and drive right before the goal
         *             releaseLow();
         *             cargo();
         *             parking();
         *         }
         *         else if (recognition.getMiddle() == 0){
         *             Move up a bit rotate 90 degrees to the right, then drive stragitforward towards the middle, then rotate again 90 degrees to the right and drive right before the goal
         *             releaseMiddle();
         *             cargo();
         *             parking();
         *         }
         */
    }
        /**
         *
         * If it recognizes team marker, start this function
         */
    public void autonomousTeamMarker(Recognition recognition) {
/**
        if (recognition.getRight > 0){
            Move up a bit rotate 90 degrees to the right, then drive stragitforward towards the middle, then rotate again 90 degrees to the right and drive right before the goal
            releaseHigh();
            cargo();
            parking();

        }
        else if (recognition.getLeft() < 0){
            Move up a bit rotate 90 degrees to the right, then drive stragitforward towards the middle, then rotate again 90 degrees to the right and drive right before the goal
            releaseLow();
            cargo();
            parking();
        }
        else if (recognition.getMiddle() == 0){
            Move up a bit rotate 90 degrees to the right, then drive stragitforward towards the middle, then rotate again 90 degrees to the right and drive right before the goal
            releaseMiddle();
            cargo();
            parking();
        }
**/

    }

    /**Functions for top, middle, and bottom goal**/
    public void releaseLow() throws InterruptedException {
        wait(200);
        int timer = 0;
        while(timer < 150){
            //outtakeMotorLift.setPower(1);
            if (timer < 150){
                timer += 1;
            }
            //wait(100);
        }

        //set power 1
        //wait 100 ms
        //set power 0
        wait(200);
        //outtakeMotor.setPower(0);
        wait(1000);
        release(300);
    }

    public void releaseHigh() throws InterruptedException{
        wait(200);
        int timer = 0;
        while(timer < 500){
            //outtakeMotor.setPower(1);
            if (timer < 500){
                timer += 1;
            }
        }
        wait(200);
        if (timer == 500){
            //outtakeMotor.setPower(0);
        }
        wait(1000);
        release(500);
    }

    public void releaseMiddle() throws InterruptedException {
        wait(200);
        int timer = 0;
        while(timer < 350){
            //outtakeMotor.setPower(1);
            if (timer < 350){
                timer += 1;
            }

        }
        wait(200);
        //outtakeMotor.setPower(0);

        wait(1000);
        release(500);
    }


    public void release(int counter) throws InterruptedException {
        wait(200);
        int timer = 0;
        while(timer < counter){
            //outtakeMotor.setPower(1);
            if (timer < counter){
                timer += 1;
            }
        }
        wait(200);
        //outtakeMotor.setPower(0);
        wait(1000);
    }
    public void cargo() {
         /**Make sure that the robot turns 180 degrees so it is facing down
          * Drive down until the edge, then rotate 90 degrees to the left.
          * Drive towards the beams and collect the cargo block
          * Turn 180 degrees again, drive until it is out of the beams, and drop the off
          **/
    }
    public void parking() throws InterruptedException {
        int counter = 0;
        while(touchsensor1.isPressed() == false && touchsensor2.isPressed() == false){
            bottomLeftMotor.setPower(1);
            bottomRightMotor.setPower(1);
            topLeftMotor.setPower(1);
            topRightMotor.setPower(1);
        }
        wait(200);

        bottomLeftMotor.setPower(0);
        bottomRightMotor.setPower(0);
        topLeftMotor.setPower(0);
        topRightMotor.setPower(0);
        wait(1000);
        carousel();
        wait(1000);
        //rotate 90 degrees to the right and drive towards the parking lot

    }
    public void carousel() throws InterruptedException{
        int counter = 0;
        while (counter < 500){
               //carouselMotor.setPower(1);
                if (counter < 500){
                    counter += 1;
                }
            }
            wait(200);
            //carouselMotor.setPower(0);
            wait(1000);

        }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.65f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
