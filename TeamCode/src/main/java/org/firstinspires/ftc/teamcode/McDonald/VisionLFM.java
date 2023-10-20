package org.firstinspires.ftc.teamcode.McDonald;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Locale;
    /*
     * This OpMode illustrates the basics of TensorFlow Object Detection, using
     * the easiest way.
     *
     * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
     */
    @TeleOp(name = "Auto Mode LFM (by time)", group = "Auto")
    public class VisionLFM extends LinearOpMode {

        private boolean DEBUG = false;

        private double MOVE_INDEX = 2;
        private double MOVE_MIDDLE = 0.7;
        private double MOVE_LEFT = 2;
        private double MOVE_RIGHT = 2;

        static final double     FORWARD_SPEED = 0.2;
        static final double     TURN_SPEED    = 0.2;

        private ElapsedTime runtime = new ElapsedTime();

        private DcMotor frontLeftMotor = null;
        private DcMotor backLeftMotor = null;
        private DcMotor frontRightMotor = null;
        private DcMotor backRightMotor = null;

        private Servo leftGripper;
        private Servo rightGripper;

        private Servo wrist;


        private enum PixelPosition {
            UNKNOWN,
            LEFT,
            MIDDLE,
            RIGHT
        }

        private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
        // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
        // this is only used for Android Studio when using models in Assets.
        private static final String TFOD_MODEL_ASSET = "model_20231015_125021.tflite";
        // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
        // this is used when uploading models directly to the RC using the model upload interface.
        private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/model_20231015_125021.tflite";
        // Define the labels recognized in the model for TFOD (must be in training order!)
        private static final String[] LABELS = {
                "Red Horseshoe",
                "Blue Horseshoe"
        };


        private TfodProcessor tfod;
        private VisionPortal visionPortal;
        private VisionPortal.Builder visionPortalBuilder;

        @Override
        public void runOpMode() {

            double  drive           = 0;        // Desired forward power/speed (-1 to +1) +ve is forward
            double  turn            = 0;        // Desired turning power/speed (-1 to +1) +ve is CounterClockwise

            frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
            backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
            frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
            backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

            frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            initTfod();

            leftGripper = hardwareMap.get(Servo.class, "leftGripper");
            rightGripper = hardwareMap.get(Servo.class, "rightGripper");
            wrist = hardwareMap.servo.get("wristServo");
            wrist.setPosition(.4);
            leftGripper.setPosition(1); // Adjust the position value as needed
            rightGripper.setPosition(0); // Adjust the position value as needed

            justWait(1000);

            leftGripper.setPosition(1); // Adjust the position value as needed
            rightGripper.setPosition(0); // Adjust the position value as needed

            // Wait for the DS start button to be touched.
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();
            waitForStart();
            moveRobot("", 1);
            if (opModeIsActive()) {
                while (opModeIsActive()) {

                    telemetryTfod();

                    // Push telemetry to the Driver Station.
                    telemetry.update();

                    // Save CPU resources; can resume streaming when needed.
                    if (gamepad1.dpad_down) {
                        visionPortal.stopStreaming();
                    } else if (gamepad1.dpad_up) {
                        visionPortal.resumeStreaming();
                    }

                    // Share the CPU.
                    sleep(20);
                }
            }

            // Save more CPU resources when camera is no longer needed.
            visionPortal.close();

        }   // end runOpMode()

        /**
         * Initialize the TensorFlow Object Detection processor.
         */
        private void initTfod() {

            // Create the TensorFlow processor the easy way.
            // tfod = TfodProcessor.easyCreateWithDefaults();
            // Create the TensorFlow processor by using a builder.
            tfod = new TfodProcessor.Builder()

                    // With the following lines commented out, the default TfodProcessor Builder
                    // will load the default model for the season. To define a custom model to load,
                    // choose one of the following:
                    //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                    //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                    .setModelAssetName(TFOD_MODEL_ASSET)
                    //.setModelFileName(TFOD_MODEL_FILE)

                    // The following default settings are available to un-comment and edit as needed to
                    // set parameters for custom models.
                    .setModelLabels(LABELS)
                    //.setIsModelTensorFlow2(true)
                    //.setIsModelQuantized(true)
                    //.setModelInputSize(300)
                    //.setModelAspectRatio(16.0 / 9.0)

                    .build();

            // Create the vision portal the easy way.
            if (USE_WEBCAM) {
                visionPortalBuilder = new VisionPortal.Builder();
                visionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
                visionPortalBuilder.setCameraResolution(new Size(640, 480));
                visionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
                visionPortalBuilder.enableLiveView(true);
                visionPortalBuilder.addProcessors(tfod);
                visionPortal = visionPortalBuilder.build();
                //visionPortal = VisionPortal.easyCreateWithDefaults(
                //    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);

            } else {
                visionPortal = VisionPortal.easyCreateWithDefaults(
                        BuiltinCameraDirection.BACK, tfod);
            }

        }   // end method initTfod()

        /**
         * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
         */
        private void telemetryTfod() {
            PixelPosition pPosition;
            boolean pixelFound = false;

            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());

            //Quickly search and see if we found a Pixel
            if(currentRecognitions.contains("Blue Horseshoe") || currentRecognitions.contains("Red Horseshoe")) {
                pixelFound = true;
                telemetry.addData("Object Found %s", " Horseshoe");
                telemetry.update();
            }

            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {

                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

                telemetry.addData(""," ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                //telemetry.addData("- Color", "%s", recognition.get)
                justWait(1000);

                // Asuming a configuration of 640x480 pixels
                // Pixel/Prop found with x < 214 = Pixel/Prop on left strike line
                // Pixel/Prop found with 214 < x < 428 = Pixel/Prop on middle strike line
                // Pixel/Prop found with 428 < x = Pixel/Prop on right strike line
                telemetry.addData("", " ");
                if (x < 100) {
                    pPosition = PixelPosition.LEFT;
                    telemetry.addData("Pixel Position: ", "Left Line");
                    telemetry.update();
                    justWait(1000);
                } else if (x > 428) {
                    pPosition = PixelPosition.RIGHT;
                    telemetry.addData("Pixel Position: ", "Right Line");
                    telemetry.update();
                    justWait(1000);
                } else {
                    pPosition = PixelPosition.MIDDLE;
                    telemetry.addData("Pixel Position: ", "Middle Line");
                    telemetry.update();
                    justWait(1000);
                }

                //index to center of strike lines
                moveRobot("Index To Strike Lines", MOVE_INDEX);

                switch(pPosition) {
                    case LEFT:
                        turnRobot("left", MOVE_LEFT);
                        telemetry.addData("Turn: Left", "");
                        telemetry.update();
                        justWait(1000);
                        moveArm();
                        break;
                    case RIGHT:
                        turnRobot("right", MOVE_RIGHT);
                        telemetry.addData("Turn: Right", "");
                        telemetry.update();
                        justWait(1000);
                        moveArm();
                        break;
                    case MIDDLE:
                        moveRobot("Center Line", MOVE_MIDDLE);
                        telemetry.addData("Move: Forward", "");
                        telemetry.update();
                        justWait(1000);
                        moveArm();
                        break;
                    case UNKNOWN:
                        break;
                }

                if(pixelFound) {
                    break;
                }
            }   // end for() loop

        }   // end method telemetryTfod()

        private void moveRobot(String path, double time) {
            frontLeftMotor.setPower(-FORWARD_SPEED);
            backLeftMotor.setPower(-FORWARD_SPEED);
            frontRightMotor.setPower(-FORWARD_SPEED);
            backRightMotor.setPower(-FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < time)) {
                telemetry.addData("Path", "%s: %4.1f S Elapsed", path, runtime.seconds());
                telemetry.update();
            }

            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
        }

        // tank turn
        // String direction = left or right
        private void turnRobot(String direction, double time) {
            switch(direction.toLowerCase(Locale.ROOT)) {
                case "left":
                    frontLeftMotor.setPower(TURN_SPEED);
                    backLeftMotor.setPower(TURN_SPEED);
                    frontRightMotor.setPower(-TURN_SPEED);
                    backRightMotor.setPower(-TURN_SPEED);
                    runtime.reset();

                    while (opModeIsActive() && (runtime.seconds() < time)) {
                        telemetry.addData("Path", "%s: %4.1f S Elapsed", direction, runtime.seconds());
                        telemetry.update();
                    }
                    frontLeftMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backRightMotor.setPower(0);
                    break;
                case "right":
                    frontLeftMotor.setPower(-TURN_SPEED);
                    backLeftMotor.setPower(-TURN_SPEED);
                    frontRightMotor.setPower(TURN_SPEED);
                    backRightMotor.setPower(TURN_SPEED);
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() < time)) {
                        telemetry.addData("Path", "%s: %4.1f S Elapsed", direction, runtime.seconds());
                        telemetry.update();
                    }
                    frontLeftMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backRightMotor.setPower(0);
                    break;
            }
        }

        // 1 = camera position
        // 2 = pixel drop position
        private void moveArm() {
            telemetry.addData("Dropping Pixel", "");
            telemetry.update();
            leftGripper.setPosition(0.9); // Adjust the position value as needed
            rightGripper.setPosition(0.1); // Adjust the position value as needed
            telemetry.addData("Pixel Dropped", "");
            telemetry.update();

        }

        private void justWait(int milliseconds) {
            double currTime = getRuntime();
            double waitUntil = currTime + (double)(milliseconds/1000);
            while (getRuntime() < waitUntil) {

            }
        }

    }   // end class
