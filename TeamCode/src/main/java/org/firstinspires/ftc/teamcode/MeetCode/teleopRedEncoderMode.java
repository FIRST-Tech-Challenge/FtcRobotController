package org.firstinspires.ftc.teamcode.MeetCode;

//Aman Sulaiman, 23-24 CenterStage

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

//speed up everything
//faster when driving regularly, slower when picking up and dropping pixel
//make the cascade go up when driving and get rid of the bug with the encoders and the claw/wrist
@TeleOp(name = "encoderRedTeleop")
public class teleopRedEncoderMode extends LinearOpMode {
    double cascadeMotorPower;
    final double DESIRED_DISTANCE = 8.5; //  this is how close the camera should get to the target (inches)
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.025;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.025;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.025;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.55;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.55;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.55;

    public Hardware robot = new Hardware();
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private int DESIRED_TAG_ID;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;
    private int cascadePosition = 0;
    public void runOpMode() {
        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)
        robot.init(hardwareMap);
        initAprilTag();
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm.setTargetPosition(0);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(.3);
        robot.claw.setPosition(0);
        sleep(500);
        robot.wrist.setPosition(0.675);
        robot.resetEncodersCascade();
        robot.turnOnEncoders();
        robot.turnOnEncodersCascade();
        robot.cascadeMotorRight.setTargetPosition(0);
        robot.cascadeMotorLeft.setTargetPosition(0);
        robot.cascadeMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.cascadeMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.cascadeMotorLeft.setPower(.6);
        robot.cascadeMotorRight.setPower(.6);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (USE_WEBCAM)
            setManualExposure(6, 250);



        waitForStart();
        while (opModeIsActive()) {
//            robot.cascadeLock(cascadePosition);
            if (gamepad1.x)
                DESIRED_TAG_ID = 4;
            else if (gamepad1.y)
                DESIRED_TAG_ID = 5;
            else if (gamepad1.b)
                DESIRED_TAG_ID = 6;
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
            cascadeMotorPower = Range.clip(cascadeMotorPower, -.1, .8);
//            robot.frontRight.setPower(.8 * ((gamepad1.left_stick_y + gamepad1.right_stick_x) + gamepad1.left_stick_x));
//            robot.frontLeft.setPower(.8 * ((gamepad1.left_stick_y - gamepad1.right_stick_x) - gamepad1.left_stick_x));
//            robot.backRight.setPower(.8 * ((gamepad1.left_stick_y + gamepad1.right_stick_x) - gamepad1.left_stick_x));
//            robot.backLeft.setPower(.8 * ((gamepad1.left_stick_y - gamepad1.right_stick_x) + gamepad1.left_stick_x));

            telemetry.addData("Cascade power: ", robot.cascadeMotorLeft.getPower());
            telemetry.addData("arm: ", robot.arm.getCurrentPosition());
            telemetry.addData("Wrist: ", robot.wrist.getPosition());
            telemetry.addData("CascadeLeft: ", robot.cascadeMotorLeft.getCurrentPosition()); //800
            telemetry.addData("CascadeRight: ", robot.cascadeMotorRight.getCurrentPosition()); //800

            telemetry.update();
            if (gamepad1.left_bumper && targetFound) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;
                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
//                telemetry.update();
                // Apply desired axes motions to the drivetrain.
            } else {
                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                drive = gamepad1.left_stick_y / 1;  // Reduce drive rate to 50%.
                strafe = gamepad1.left_stick_x / 1;  // Reduce strafe rate to 50%.
                turn = -gamepad1.right_stick_x / 1.5;  // Reduce turn rate to 33%.
//                if (robot.wrist.getPosition() == .385) {
//                    drive = gamepad1.left_stick_x
//                }
//                telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }

            telemetry.update();
            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            if (Math.abs(gamepad2.left_stick_y) >= .3) {
                cascadePosition = cascadePosition + (int) (-32 * gamepad2.left_stick_y);
                cascadePosition = Math.min(2000, Math.max(0, cascadePosition));
                robot.cascadeMotorRight.setTargetPosition(cascadePosition);
                robot.cascadeMotorLeft.setTargetPosition(cascadePosition);
                sleep(15);
            }
            if (gamepad2.right_bumper) {
                robot.claw.setPosition(0);
            }
            else if (gamepad2.left_bumper) {
                robot.claw.setPosition(0.58);
            }
            if (gamepad2.a) {
                robot.claw.setPosition(robot.claw.getPosition() + 0.01);
                sleep(200);
//                robot.setPowerOfAllMotorsTo(0);
//                if (robot.claw.getPosition()!=0)
//                    robot.claw.setPosition(0);
//                cascadePosition = 940;
//                robot.cascadeLock(cascadePosition);
//                while (robot.cascadeMotorLeft.getCurrentPosition() < 890)
//                    sleep(100);
//                robot.arm.setTargetPosition(545);
//                robot.arm.setMode(RUN_TO_POSITION);
//                robot.arm.setPower(0.3);
//                sleep(1000);
//                    robot.cascadeMotorLeft.setPower(0);
//                    robot.cascadeMotorRight.setPower(0);
//                    robot.turnOffEncodersCascade();

            }
            if (gamepad2.x && robot.cascadeMotorLeft.getCurrentPosition() > 940) {

                robot.arm.setTargetPosition(540);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(.3);


            }
            else if (gamepad2.b && robot.cascadeMotorLeft.getCurrentPosition() > 940) {
                robot.arm.setTargetPosition(0);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(.3);
            }
            else if (gamepad2.y) {
                robot.claw.setPosition(0.47);
//                sleep(125);
//                robot.claw.setPosition(0);
            }

            if (gamepad2.dpad_up) {
                robot.wrist.setPosition(.675);
            }
            else if (gamepad2.dpad_down) {
                robot.wrist.setPosition(.385);
            }
//            if (gamepad2.left_trigger > .3) {
//            robot.wrist.setPosition(robot.wrist.getPosition() - 0.01);
//            sleep(200);
//            }
//            else if (gamepad2.right_trigger > .3) {
//                robot.wrist.setPosition(robot.wrist.getPosition() + 0.01);
//                sleep(200);
//            }
            else if (gamepad1.dpad_up) {
                robot.launch.setPosition(1);
            }
            else if (gamepad2.dpad_left) {
                robot.wrist.setPosition(.485);
            }
            else if (gamepad1.dpad_right) {
                robot.dropper.setPosition(.3);
            }
            else if (gamepad1.dpad_left) {
                robot.dropper.setPosition(.8);
            }
        }
    }

    public void moveRobot ( double x, double y, double yaw){
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;
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
        robot.frontLeft.setPower(rightBackPower);
        robot.frontRight.setPower(leftBackPower);
        robot.backLeft.setPower(rightFrontPower);
        robot.backRight.setPower(leftFrontPower);
        /*if (desiredTag.ftcPose.range > DESIRED_DISTANCE) {
            while (desiredTag.ftcPose.range > DESIRED_DISTANCE) {
                robot.setPowerOfAllMotorsTo(-.3);
            }
            robot.setPowerOfAllMotorsTo(0);
        } else if (desiredTag.ftcPose.range < DESIRED_DISTANCE) {
            while (desiredTag.ftcPose.range < DESIRED_DISTANCE) {
                robot.setPowerOfAllMotorsTo(.3);
            }
            robot.setPowerOfAllMotorsTo(0);
        }*/
    }
    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag () {
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

    private void setManualExposure ( int exposureMS, int gain){
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
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
