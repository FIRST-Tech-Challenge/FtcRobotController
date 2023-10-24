package org.firstinspires.ftc.teamcode.developingTestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "TEST Disalign Auton Blue Back Align")
public class V2AutonBlueBackAlign extends LinearOpMode {

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private double Ticks_Per_Inch = 45.2763982107824;

    private int leftFrontDriveTickTracker = 0;
    private int rightFrontDriveTickTracker = 0;
    private int leftBackDriveTickTracker = 0;
    private int rightBackDriveTickTracker = 0;

    final double DESIRED_DISTANCE = 3.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 2;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;

    @Override
    public void runOpMode() {

        DcMotor leftFrontDrive = hardwareMap.dcMotor.get("motorFL");
        DcMotor leftBackDrive = hardwareMap.dcMotor.get("motorBL");
        DcMotor rightFrontDrive = hardwareMap.dcMotor.get("motorFR");
        DcMotor rightBackDrive = hardwareMap.dcMotor.get("motorBR");

        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initAprilTag();

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            int Rounded_Encoder_Ticks = calculateTicksForLateralMovement(3);

            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            rightFrontDrive.setTargetPosition(-1680);
            rightFrontDrive.setPower(0.5);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftBackDrive.setTargetPosition(-1680);
            leftBackDrive.setPower(0.5);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFrontDrive.setTargetPosition(1680);
            leftFrontDrive.setPower(0.5);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rightBackDrive.setTargetPosition(1680);
            rightBackDrive.setPower(0.5);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("Front Left Target Position",leftFrontDrive.getTargetPosition());
            telemetry.addData("Front Left Current Position Position",leftFrontDrive.getCurrentPosition());

            telemetry.addData("Front Right Target Position",rightFrontDrive.getTargetPosition());
            telemetry.addData("Front Right Current Position Position",rightFrontDrive.getCurrentPosition());

            telemetry.addData("Back Left Target Position",leftBackDrive.getTargetPosition());
            telemetry.addData("Back Left Current Position Position",leftBackDrive.getCurrentPosition());

            telemetry.addData("Back Right Target Position",rightBackDrive.getTargetPosition());
            telemetry.addData("Back Right Current Position Position",rightBackDrive.getCurrentPosition());

            telemetry.update();

            while (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy()) {}
            rightFrontDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);
            }

            int leftFrontDriveNecessaryTicks = calculateTicksForLateralMovement(70); //2000
            int rightFrontDriveNecessaryTicks = calculateTicksForLateralMovement(70);
            int leftBackDriveNecessaryTicks = calculateTicksForLateralMovement(70);
            int rightBackDriveNecessaryTicks = calculateTicksForLateralMovement(70);

            int leftFrontDriveTargetTicks =  leftFrontDriveNecessaryTicks;
            int rightFrontDriveTargetTicks = rightFrontDriveNecessaryTicks;
            int leftBackDriveTargetTicks = leftBackDriveNecessaryTicks;
            int rightBackDriveTargetTicks = rightBackDriveNecessaryTicks;

            rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

            leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + leftFrontDriveTargetTicks);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontDrive.setPower(0.5);

            rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + rightFrontDriveTargetTicks);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setPower(0.5);

            leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + leftBackDriveTargetTicks);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setPower(0.5);

            rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + rightBackDriveTargetTicks);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setPower(0.5);

            telemetry.addData("Front Left Target Position",leftFrontDrive.getTargetPosition());
            telemetry.addData("Front Left Current Position Position",leftFrontDrive.getCurrentPosition());

            telemetry.addData("Front Right Target Position",rightFrontDrive.getTargetPosition());
            telemetry.addData("Front Right Current Position Position",rightFrontDrive.getCurrentPosition());

            telemetry.addData("Back Left Target Position",leftBackDrive.getTargetPosition());
            telemetry.addData("Back Left Current Position Position",leftBackDrive.getCurrentPosition());

            telemetry.addData("Back Right Target Position",rightBackDrive.getTargetPosition());
            telemetry.addData("Back Right Current Position Position",rightBackDrive.getCurrentPosition());

            telemetry.update();

            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            rightFrontDrive.setTargetPosition(-100);
            rightFrontDrive.setPower(0.5);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftBackDrive.setTargetPosition(-100);
            leftBackDrive.setPower(0.5);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFrontDrive.setTargetPosition(100);
            leftFrontDrive.setPower(0.5);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rightBackDrive.setTargetPosition(100);
            rightBackDrive.setPower(0.5);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive()) {
            targetFound = false;
            desiredTag = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) &&
                        ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))) {
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData(">", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData(">", "Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else {

                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                drive = -gamepad1.left_stick_y / 2.0;  // Reduce drive rate to 50%.
                strafe = -gamepad1.left_stick_x / 2.0;  // Reduce strafe rate to 50%.
                turn = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
                telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            sleep(10);
        }
    }


    public int calculateTicksForLateralMovement(double inches) {
        double Calculated_Encoder_Ticks = (inches * Ticks_Per_Inch);
        int Rounded_Encoder_Ticks = (int)Math.round(Calculated_Encoder_Ticks);
        return Rounded_Encoder_Ticks;
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

    /**
     * Initialize the AprilTag processor.
     */
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

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
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
}
