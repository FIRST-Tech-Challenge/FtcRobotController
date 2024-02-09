package org.firstinspires.ftc.teamcode.shared;

import static android.os.SystemClock.sleep;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.shared.GlobalConfig.ALLIANCE_POS;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class VisionHardware {

    public static boolean DEBUG = false;

    public GlobalConfig globalConfig = null;
    private LinearOpMode myOpMode = null;

    private ALLIANCE_POS alliancePos = null;

    public static double detectWait = 6.0;
    private ElapsedTime runtime = new ElapsedTime();
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private VisionPortal.Builder visionPortalBuilder;

    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;

    private int     myExposure  ;
    private int     minExposure ;
    private int     maxExposure ;
    private int     myGain      ;
    private int     minGain ;
    private int     maxGain ;

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "ModelSpheresClassWindowSLC.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/ModelSphereClassWindowSLC.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Red Sphere",
            "Blue Sphere"
    };
    public enum PropPosition {
        UNKNOWN,
        LEFT,
        MIDDLE,
        RIGHT
    }

    // Screen X axis division segments
    // From Left Alliance Positions
    private int ALLIANCE_LEFT_L = 400; // <=
    private int ALLIANCE_LEFT_M = 400; // >

    // From Right Alliance Positions
    private int ALLIANCE_RIGHT_R = 624; // >=
    private int ALLIANCE_RIGHT_M = 624; // <


    public VisionHardware(LinearOpMode opmode) { myOpMode = opmode; };

    public VisionHardware(LinearOpMode opmode, ALLIANCE_POS alliancePos) {
        myOpMode = opmode;
        this.alliancePos = alliancePos;
    }
    public VisionHardware(LinearOpMode opmode, ALLIANCE_POS alliancePos, GlobalConfig globalConfig) {
        myOpMode = opmode;
        this.alliancePos = alliancePos;
        this.globalConfig = globalConfig;
    }

    public void init() {

        // Create the AprilTag processor by using a builder.
        //aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag = new AprilTagProcessor.Builder()
                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagFamily(AprilTagProcessor.TagFamily.)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                //.setLensIntrinsics(1288.42, 2488.42, 644.036, 595.953)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        /**
        myExposure = Math.min(5, minExposure);
        myGain = maxGain;
        setManualExposure(myExposure, myGain);
        myOpMode.telemetry.addData("Exposure: ", myExposure);
        myOpMode.telemetry.addData("Gain: ", myGain);
        myOpMode.telemetry.update();
        sleep(5000);
        setManualExposure(250, 8);
         **/
        resetExposureMode();

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
            visionPortalBuilder.setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"));
            //visionPortalBuilder.setCameraResolution(new Size(640, 480));
            visionPortalBuilder.setCameraResolution(new Size(1280, 720));
            visionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
            visionPortalBuilder.enableLiveView(true);
            visionPortalBuilder.addProcessors(tfod);
            //visionPortalBuilder.addProcessor(aprilTag);
            visionPortal = visionPortalBuilder.build();
            //visionPortal = VisionPortal.easyCreateWithDefaults(
            //    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);

        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }
    }


    public PropPosition detectProp() {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        // Try to detect the prop for no more than "detectWait" seconds
        while ( timer.seconds() < detectWait ) {
            PropPosition pPosition;
            boolean pixelFound = false;

            List<Recognition> currentRecognitions = tfod.getRecognitions();

            if (currentRecognitions.size() > 0) {
                myOpMode.telemetry.addData("# Objects Detected", currentRecognitions.size());
                myOpMode.telemetry.update();
                debugWait();

                // Step through the list of recognitions and display info for each one.
                for (Recognition recognition : currentRecognitions) {
                    // Is this recognition one of our models labels?
                    if(Arrays.asList(LABELS).contains(recognition.getLabel())) {
                        double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                        double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;


                        switch(alliancePos) {
                            case LEFT:
                                if (x < ALLIANCE_LEFT_L) {
                                    myOpMode.telemetry.addData("Prop Left", "");
                                    myOpMode.telemetry.update();
                                    debugWait();
                                    return PropPosition.LEFT;
                                } else if (x > ALLIANCE_LEFT_M) {
                                    myOpMode.telemetry.addData("Prop Middle", "");
                                    myOpMode.telemetry.update();
                                    debugWait();
                                    //return PropPosition.RIGHT;
                                    return PropPosition.MIDDLE;
                                } else {
                                    myOpMode.telemetry.addData("Prop Right", "");
                                    myOpMode.telemetry.update();
                                    debugWait();
                                    return PropPosition.RIGHT;
                                }
                            case RIGHT:
                                if (x >= ALLIANCE_RIGHT_R) {
                                    myOpMode.telemetry.addData("Prop Right", "");
                                    myOpMode.telemetry.update();
                                    debugWait();
                                    return PropPosition.RIGHT;
                                } else if (x < ALLIANCE_RIGHT_M) {
                                    myOpMode.telemetry.addData("Prop Middle", "");
                                    myOpMode.telemetry.update();
                                    debugWait();
                                    //return PropPosition.RIGHT;
                                    return PropPosition.MIDDLE;
                                } else {
                                    myOpMode.telemetry.addData("Prop Left", "");
                                    myOpMode.telemetry.update();
                                    debugWait();
                                    return PropPosition.LEFT;
                                }
                        }


                        if (x < 400) {
                            myOpMode.telemetry.addData("Prop Left", "");
                            myOpMode.telemetry.update();
                            debugWait();
                            return PropPosition.LEFT;
                        } else if (x > 900) {
                            myOpMode.telemetry.addData("Prop Right", "");
                            myOpMode.telemetry.update();
                            debugWait();
                            //return PropPosition.RIGHT;
                            return PropPosition.RIGHT;
                        } else {
                            myOpMode.telemetry.addData("Prop Middle", "");
                            myOpMode.telemetry.update();
                            debugWait();
                            return PropPosition.MIDDLE;
                        }


                    }
                }
            }

        }
        // We didn't find the prop, so lets pick a location and go
        return PropPosition.UNKNOWN;
    }


    public boolean detectTag(int DESIRED_TAG_ID, MotionHardware robot, int DESIRED_DISTANCE) {


        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        desiredTag  = null;

        while ( timer.seconds() < detectWait ) {
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        myOpMode.telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    myOpMode.telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                myOpMode.telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
                myOpMode.telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                myOpMode.telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                myOpMode.telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                myOpMode.telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                myOpMode.telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(-headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                myOpMode.telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else {


            }
            myOpMode.telemetry.update();

            // Apply desired axes motions to the drivetrain.
            robot.moveRobotTag(drive, strafe, turn);
            sleep(10);
        }

        /**
        // Try to detect the prop for no more than "detectWait" seconds
        while ( timer.seconds() < detectWait ) {
            targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            if (currentDetections.size() > 0) {
                for (AprilTagDetection detection : currentDetections) {
                    // Look to see if we have size info on this tag.
                    if (detection.metadata != null) {
                        //  Check to see if we want to track towards this tag.
                        if ((desiredTagId < 0) || (detection.id == desiredTagId)) {
                            // Yes, we want to use this tag.
                            targetFound = true;
                            desiredTag = detection;
                            return desiredTag;
                        } else {
                            // This tag is in the library, but we do not want to track it right now.
                            myOpMode.telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                        }
                    } else {
                        // This tag is NOT in the library, so we don't have enough information to track to it.
                        myOpMode.telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                    }
                }
            } else {
                //TODO Put in some code to move the robot closer to the board and stop.
            }
        }
    return desiredTag;
    **/
      return true;
    }


    public void resetExposure() {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode((ExposureControl.Mode.Auto));
        sleep(50);
    }


    public boolean resetExposureMode() {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            myOpMode.telemetry.addData("Camera", "Waiting");
            myOpMode.telemetry.update();
            while (!myOpMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            myOpMode.telemetry.addData("Camera", "Ready");
            myOpMode.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!myOpMode.isStopRequested())
        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            exposureControl.setMode(ExposureControl.Mode.Auto);
            sleep(50);

            return (true);
        } else {
            return (false);
        }
    }
    /*
        Manually set the camera gain and exposure.
        Can only be called AFTER calling initAprilTag();
        Returns true if controls are set.
     */
    public boolean    setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            myOpMode.telemetry.addData("Camera", "Waiting");
            myOpMode.telemetry.update();
            while (!myOpMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            myOpMode.telemetry.addData("Camera", "Ready");
            myOpMode.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!myOpMode.isStopRequested())
        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);

            // Set Gain.
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);

            WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
            whiteBalanceControl.setMode(WhiteBalanceControl.Mode.AUTO);
            return (true);
        } else {
            return (false);
        }
    }

    /*
        Read this camera's minimum and maximum Exposure and Gain settings.
        Can only be called AFTER calling initAprilTag();
     */
    private void getCameraSetting() {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            myOpMode.telemetry.addData("Camera", "Waiting");
            myOpMode.telemetry.update();
            while (!myOpMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            myOpMode.telemetry.addData("Camera", "Ready");
            myOpMode.telemetry.update();
        }

        // Get camera control values unless we are stopping.
        if (!myOpMode.isStopRequested()) {

            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            minExposure = (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposure = (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            minGain = gainControl.getMinGain();
            maxGain = gainControl.getMaxGain();
        }
    }

    private void debugWait() {
        if (DEBUG) {
            sleep(5000);
        } else {
            sleep(1000);
        }
    }
}
