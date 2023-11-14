package org.firstinspires.ftc.teamcode.McDonald;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection, using
 * the easiest way.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Config
@Autonomous(name = "Auto w/Park LFM (by time)", group = "Auto")
public class VisionLFM2 extends LinearOpMode {

    public static boolean DEBUG = false;

    // All Starting Quadrants
    public static double MOVE_INDEX = 1;
    public static double MOVE_MIDDLE = 0.1;
    public static double MOVE_LEFT = 1.18;
    public static double MOVE_RIGHT = 1;
    public static double MOVE_LEFT_LINE = 1;
    public static double MOVE_RIGHT_LINE = .8;
    public static double MOVE_FORWARD = 2;

    // Left Red Starting Quadrants
    public static double MOVE_BACK_OPP_FIELD = 3.5;
    public static double MOVE_FWD_OPP_FIELD = 3.2;
    public static double MOVE_FIELD_MIDDLE = 1.8;
    public static double MOVE_OPP_FIELD = 3;
    public static double MOVE_BOTTOM_FIELD = 3;
    public static double MOVE_PARK = 1.5;



    public static final double     FORWARD_SPEED = 0.4;
    public static final double     TURN_SPEED    = 0.4;

    public static double detectWait = 6.0;

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    private Servo leftGripper;
    private Servo rightGripper;

    private Servo wrist;

    private enum StartQuad {
        REDLEFT,
        REDRIGHT,
        BLUELEFT,
        BLUERIGHT
    }

    private enum PropPosition {
        UNKNOWN,
        LEFT,
        MIDDLE,
        RIGHT
    }

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "model_cube_props-102823.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/model_20231015_125021.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "RED CUBE",
            "BLUE CUBE"
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


        if (opModeIsActive()) {
            while (opModeIsActive() && !isStopRequested()) {

                // Find prop
                PropPosition pPosition = detectProp();
                telemetry.update();

                // Drive to prop
                propDropOff(pPosition);
                telemetry.update();

                // Go Park
                //park(pPosition);
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);

                break;
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


    private PropPosition detectProp() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        // Try to detect the prop for no more than "detectWait" seconds
        while ( timer.seconds() < detectWait ) {
            PropPosition pPosition;
            boolean pixelFound = false;

            List<Recognition> currentRecognitions = tfod.getRecognitions();

            if (currentRecognitions.size() > 0) {
                telemetry.addData("# Objects Detected", currentRecognitions.size());
                telemetry.update();
                debugWait();

                // Step through the list of recognitions and display info for each one.
                for (Recognition recognition : currentRecognitions) {
                    // Is this recognition one of our models labels?
                    if(Arrays.asList(LABELS).contains(recognition.getLabel())) {
                        double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                        double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

                        if (x < 300) {
                            telemetry.addData("Prop Left", "");
                            telemetry.update();
                            debugWait();
                            return PropPosition.LEFT;
                        } else if (x > 300) {
                            telemetry.addData("Prop Right", "");
                            telemetry.update();
                            debugWait();
                            //return PropPosition.RIGHT;
                            return PropPosition.MIDDLE;
                        } else {
                            telemetry.addData("Prop Middle", "");
                            telemetry.update();
                            debugWait();
                            return PropPosition.RIGHT;
                        }
                    }
                }
            }

        }
        // We didn't find the prop, so lets pick a location and go
        return PropPosition.UNKNOWN;
    }

    private void propDropOff(PropPosition pPosition) {
        moveRobot("Index Robot", MOVE_INDEX);
        debugWait();

        switch(pPosition) {
            case LEFT:
                moveRobot("", MOVE_MIDDLE);
                turnRobot("left", MOVE_LEFT_LINE);
                moveRobot("", MOVE_MIDDLE);
                telemetry.addData("Turn: Left", "");
                telemetry.update();
                debugWait();
                dropProp();
                break;
            case RIGHT:
                moveRobot("", MOVE_MIDDLE);
                turnRobot("right", MOVE_RIGHT_LINE);
                moveRobot("", .1);
                telemetry.addData("Turn: Right", "");
                telemetry.update();
                debugWait();
                dropProp();
                break;
            case MIDDLE:
                moveRobot("Center Line", .06);
                telemetry.addData("Move: Forward", "");
                telemetry.update();
                debugWait();
                dropProp();
                break;
            case UNKNOWN:
                moveRobot("", MOVE_MIDDLE);
                turnRobot("right", MOVE_RIGHT_LINE);
                moveRobot("", .1);
                telemetry.addData("Turn: Right", "");
                telemetry.update();
                debugWait();
                dropProp();
                break;

        }

    }

    private void park(PropPosition pPosition) {

        switch(pPosition) {
            case LEFT:
                moveRobot("back", MOVE_BACK_OPP_FIELD);
                //turnRobot("right", MOVE_LEFT_LINE);
                //moveRobot("middle field y", MOVE_FIELD_MIDDLE);
                //turnRobot("right", MOVE_RIGHT);
                //moveRobot("other side of field", MOVE_OPP_FIELD);
                //turnRobot("right", MOVE_RIGHT);
                //moveRobot("bottom field y", MOVE_BOTTOM_FIELD);
                //turnRobot("left", MOVE_LEFT);
                //moveRobot("park", MOVE_PARK);
                break;
            case RIGHT:
                moveRobot("fwd opp field", MOVE_FWD_OPP_FIELD);
                //turnRobot("left", MOVE_RIGHT_LINE);
                //moveRobot("middle field y", MOVE_FIELD_MIDDLE);
                //turnRobot("right", MOVE_RIGHT);
                //moveRobot("other side of field", MOVE_OPP_FIELD);
                //turnRobot("right", MOVE_RIGHT);
                //moveRobot("bottom field y", MOVE_BOTTOM_FIELD);
                //turnRobot("left", MOVE_LEFT);
                //moveRobot("park", MOVE_PARK);
                break;
            case MIDDLE:
                moveRobot("middle field y", MOVE_FIELD_MIDDLE);
                turnRobot("right", MOVE_RIGHT);
                moveRobot("other side of field", MOVE_OPP_FIELD);
                turnRobot("right", MOVE_RIGHT);
                moveRobot("bottom field y", MOVE_BOTTOM_FIELD);
                turnRobot("left", MOVE_LEFT);
                moveRobot("park", MOVE_PARK);
                break;
            case UNKNOWN:
                moveRobot("fwd opp field", MOVE_FWD_OPP_FIELD);
                //turnRobot("left", MOVE_RIGHT_LINE);
                //moveRobot("middle field y", MOVE_FIELD_MIDDLE);
                //turnRobot("right", MOVE_RIGHT);
                //moveRobot("other side of field", MOVE_OPP_FIELD);
                //turnRobot("right", MOVE_RIGHT);
                //moveRobot("bottom field y", MOVE_BOTTOM_FIELD);
                //turnRobot("left", MOVE_LEFT);
                //moveRobot("park", MOVE_PARK);
                break;
        }
    }




    private void moveRobot(String path, double time) {

        switch(path.toLowerCase(Locale.ROOT)) {
            case "back":
                frontLeftMotor.setPower(FORWARD_SPEED);
                backLeftMotor.setPower(FORWARD_SPEED);
                frontRightMotor.setPower(FORWARD_SPEED);
                backRightMotor.setPower(FORWARD_SPEED);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < time)) {
                    telemetry.addData("Path", "%s: %4.1f S Elapsed", path, runtime.seconds());
                    telemetry.update();
                }

                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
                break;
            default:
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
                //frontLeftMotor.setPower(FORWARD_SPEED);
                //frontRightMotor.setPower(FORWARD_SPEED);
                //backLeftMotor.setPower(FORWARD_SPEED);
                //backRightMotor.setPower(FORWARD_SPEED);

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
                //frontRightMotor.setPower(TURN_SPEED);
                //backRightMotor.setPower(TURN_SPEED);
                //frontLeftMotor.setPower(-TURN_SPEED);
                //backLeftMotor.setPower(-TURN_SPEED);

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
    private void dropProp() {
        telemetry.addData("Dropping Pixel", "");
        telemetry.update();
        leftGripper.setPosition(0.9); // Adjust the position value as needed
        rightGripper.setPosition(0.1); // Adjust the position value as needed
        telemetry.addData("Pixel Dropped", "");
        telemetry.update();
        justWait(1000);

    }

    private void justWait(int milliseconds) {
        double currTime = getRuntime();
        double waitUntil = currTime + (double)(milliseconds/1000);
        while (getRuntime() < waitUntil) {

        }
    }

    private void debugWait() {
        if(DEBUG) {
            justWait(5000);
        } else {
            justWait(1000);
        }
    }

}   // end class
