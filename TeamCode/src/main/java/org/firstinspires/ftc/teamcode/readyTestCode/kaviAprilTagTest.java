package org.firstinspires.ftc.teamcode.readyTestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.*;
import java.util.List;
import java.util.concurrent.TimeUnit;



@Autonomous(name="Autonomous Test", group="Test")
@com.qualcomm.robotcore.eventloop.opmode.Disabled
public class kaviAprilTagTest extends LinearOpMode {
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    //moveForward
    @Override
    public void runOpMode() {
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "motorFL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFR");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "motorBL");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorBR");
/*
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        */
        HashMap<String, Integer> aprilTagSpikeStripCorrelation = new HashMap<String, Integer>();
        /*
        aprilTagMap.put("Blue Alliance Left", 1);
        aprilTagMap.put("Blue Alliance Center", 2);
        aprilTagMap.put("Blue Alliance Right", 3);
        aprilTagMap.put("Red Alliance Left", 4);
        aprilTagMap.put("Red Alliance Center", 5);
        aprilTagMap.put("Red Alliance Right", 6);
        aprilTagMap.put("Large Red Alliance Audience Wall Target", 7);
        aprilTagMap.put("Small Red Alliance Audience Wall Target", 8);
        aprilTagMap.put("Large Blue Alliance Audience Wall Target", 9);
        aprilTagMap.put("Small Blue Alliance Audience Wall Target", 10);
        */
        aprilTagSpikeStripCorrelation.put("Blue Front Left", 1);
        aprilTagSpikeStripCorrelation.put("Blue Front Center", 2);
        aprilTagSpikeStripCorrelation.put("Blue Front Right", 3);
        aprilTagSpikeStripCorrelation.put("Blue Back Left", 1);
        aprilTagSpikeStripCorrelation.put("Blue Back Center", 2);
        aprilTagSpikeStripCorrelation.put("Blue Back Right", 3);
        aprilTagSpikeStripCorrelation.put("Red Front Left", 4);
        aprilTagSpikeStripCorrelation.put("Red Front Center", 5);
        aprilTagSpikeStripCorrelation.put("Red Front Right", 6);
        aprilTagSpikeStripCorrelation.put("Red Back Left", 4);
        aprilTagSpikeStripCorrelation.put("Red Back Center", 5);
        aprilTagSpikeStripCorrelation.put("Red Back Right", 6);
        telemetry.addData("Hashmap: ", aprilTagSpikeStripCorrelation);
        if (USE_WEBCAM) {
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        }
        initAprilTag();
        telemetry.addLine("Waiting for start");
        String spikePosition = null;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            //Tensor Flow Object Detection Code
            //Return appropriate spikePosition
            telemetry.addData("Status", "Running");
            telemetry.update();
            spikePosition = "Red Front Left"; //TFOD sim
            telemetry.addData("Spike Position is: ", spikePosition);
            int correspondingAprilTag = aprilTagSpikeStripCorrelation.get(spikePosition);
            telemetry.addData("Corresponding April Tag is: ", correspondingAprilTag);
            boolean correspondingAprilTagFound = false;



            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null)){
                    telemetry.addData("Found AprilTag: ", detection.id);
                    telemetry.addData("Found AprilTag: ", detection.metadata);
                    desiredTag = detection;
                    telemetry.addData("desiredTag: ", desiredTag);
                    telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                    telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                    telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
                    telemetry.update();
                    if (detection.id == correspondingAprilTag) {
                        correspondingAprilTagFound = true;
                        telemetry.addLine("Corresponding April Tag Found");
                    }
                } else {
                    telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
                }
            }
            if (correspondingAprilTagFound) {
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            moveRobot(drive, strafe, turn);
            sleep(10);
        }
        telemetry.update();
    }
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }



}