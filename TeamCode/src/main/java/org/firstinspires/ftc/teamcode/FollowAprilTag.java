package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name="Follow April Tag", group = "Concept")
public class FollowAprilTag extends LinearOpMode {
    final double DESIRED_DISTANCE = 12.0;

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    final double SPEED_GAIN =   0.02 ;
    final double TURN_GAIN  =   0.01 ;

    final double MAX_AUTO_SPEED = 0.5;
    final double MAX_AUTO_TURN  = 0.25;

    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = 13; // -1 for any tag
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    private NinjaBot robot = null;

    @Override public void runOpMode()
    {
        boolean targetFound     = false;
        double  drive           = 0;
        double  turn            = 0;

        initAprilTag();

        this.robot = new NinjaBot(hardwareMap, this);

        if (USE_WEBCAM)
            // Use low exposure time to reduce motion blur
            setManualExposure(6, 250);

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            targetFound = false;
            desiredTag  = null;

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    targetFound = true;
                    desiredTag = detection;
                    break;
                } else {
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            }

            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d", desiredTag.id);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            if (gamepad1.left_bumper && targetFound) {
                // Drive automatically

                double  rangeError   = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError = desiredTag.ftcPose.bearing;

                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn  = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;

                telemetry.addData("Auto","Drive %5.2f, Turn %5.2f", drive, turn);
            } else {
                drive = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
                turn  = -gamepad1.right_stick_x / 4.0;  // Reduce turn rate to 25%.
                telemetry.addData("Manual","Drive %5.2f, Turn %5.2f", drive, turn);
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, turn);
            sleep(10);
        }
    }

    public void moveRobot(double x, double yaw) {
        // Calculate left and right wheel powers.
        double leftPower    = x - yaw;
        double rightPower   = x + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max >1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        // Send powers to the wheels.
        this.robot.leftDrive.setPower(leftPower);
        this.robot.rightDrive.setPower(rightPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "cam"))
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
        if (visionPortal == null) {
            return;
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (!isStopRequested()) {
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
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
    }
}
