package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import java.lang.Math;

@TeleOp
public class KDRobotCodeV4 extends LinearOpMode {
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    @Override
    public void runOpMode() {
        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        CRServo intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        IMU imu = hardwareMap.get(IMU.class, "imu");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initTfod();
        initAprilTag();
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );


        telemetry.addData("Hello",", Team KryptoDragons");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double driveTrainSpeed = 0.1;
        double armMotorSpeed = 0;
        double leftsticky = 0.1;
        double leftstickx = 0.1;
        double reverseRight = 0;
        double forwardLeft = 0;
        double reverseLeft = 0;
        double forwardRight = 0;
        int armPosition = 0;

        armMotor.setVelocity(250);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        armMotor.setVelocity(armMotorSpeed);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            armMotor.setVelocity(250);

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.REVERSE);

            if(gamepad1.b) {
                // Stop
                backLeft.setPower(0);
                backRight.setPower(0);
            } if (gamepad1.left_stick_y == -1) {
                // Move forward
                frontLeft.setPower(driveTrainSpeed);
                frontRight.setPower(driveTrainSpeed);
                backLeft.setPower(driveTrainSpeed);
                backRight.setPower(driveTrainSpeed);
                telemetry.addData("Motor running forward at", driveTrainSpeed);
                telemetry.update();
            } if (gamepad1.left_stick_y == 1) {
                // Move backward
                frontLeft.setPower(-driveTrainSpeed);
                frontRight.setPower(-driveTrainSpeed);
                backLeft.setPower(-driveTrainSpeed);
                backRight.setPower(-driveTrainSpeed);
                telemetry.addData("Motor running backward at", driveTrainSpeed);
                telemetry.update();
            } if (gamepad1.left_stick_x == -1) {
                //Strafe Left
                frontLeft.setPower(-driveTrainSpeed);
                frontRight.setPower(driveTrainSpeed);
                backLeft.setPower(driveTrainSpeed);
                backRight.setPower(-driveTrainSpeed);
                telemetry.addData("Motor running at", driveTrainSpeed);
                telemetry.update();
            } if (gamepad1.left_stick_x == 1) {
                //Strafe Right
                frontLeft.setPower(driveTrainSpeed);
                frontRight.setPower(-driveTrainSpeed);
                backLeft.setPower(-driveTrainSpeed);
                backRight.setPower(driveTrainSpeed);
                telemetry.addData("Motor running at", driveTrainSpeed);
                telemetry.update();
            } if (gamepad1.left_trigger == 1) {
                //Turn Left
                frontLeft.setPower(-driveTrainSpeed);
                frontRight.setPower(driveTrainSpeed);
                backLeft.setPower(-driveTrainSpeed);
                backRight.setPower(driveTrainSpeed);
            }if (gamepad1.right_trigger == 1) {
                //Turn Right
                frontLeft.setPower(driveTrainSpeed);
                frontRight.setPower(-driveTrainSpeed);
                backLeft.setPower(driveTrainSpeed);
                backRight.setPower(-driveTrainSpeed);
            } if (gamepad1.right_bumper) {
                // Increase power of driveTrain by 0.1
                sleep(250);
                driveTrainSpeed = Range.clip(driveTrainSpeed + 0.1, 0.1, 1.0);
                telemetry.addData("Motor power ", driveTrainSpeed);
                telemetry.update();
            } if (gamepad1.left_bumper) {
                // Reduce power of driveTrain by 0.1
                sleep(250);
                driveTrainSpeed = Range.clip(driveTrainSpeed - 0.1, 0.1, 1.0);
                telemetry.addData("Motor power ", driveTrainSpeed);
                telemetry.update();
            } if (forwardLeft > 1.25) {
                //Strafe Diagonal Forward Left
                frontRight.setPower(driveTrainSpeed);
                backLeft.setPower(driveTrainSpeed);
            } if (forwardRight > 1.25) {
                //Strafe Diagonal Forward Right
                frontLeft.setPower(driveTrainSpeed);
                backRight.setPower(driveTrainSpeed);
            } if (reverseLeft > 1.25) {
                //Strafe Diagonal Reverse Left
                frontLeft.setPower(-driveTrainSpeed);
                backRight.setPower(-driveTrainSpeed);
            } if (reverseRight >= 1.25) {
                //Strafe Diagonal Reverse Right
                frontRight.setPower(-driveTrainSpeed);
                backLeft.setPower(-driveTrainSpeed);
            } if (gamepad2.left_bumper) {
                sleep(5);
                armPosition = Range.clip(armPosition - 3, 0, 550);
                telemetry.addData("armposition",armPosition);
                telemetry.update();
            } if (gamepad2.right_bumper) {
                sleep(5);
                armPosition = Range.clip(armPosition + 3, 0, 550);
                telemetry.addData("armposition",armPosition);
                telemetry.update();
            } if (gamepad2.left_trigger == 1.0) {
                // Intake OR Outake
                intakeServo.setPower(1);
                telemetry.addData("Servo position", "1");
                telemetry.update();
            } if (gamepad2.right_trigger == 1.0) {
                // Intake OR Outake
                intakeServo.setPower(-1);
                telemetry.addData("Servo position", "-1");
                telemetry.update();
            } if (gamepad2.b) {
                intakeServo.setPower(0);
            }
            armMotor.setTargetPosition(armPosition);
            armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            telemetry.addData("armPosition",armPosition);
            telemetry.addData("is at target", !armMotor.isBusy());

            reverseLeft = -leftstickx + leftsticky;
            forwardRight = leftstickx + -leftsticky;
            forwardLeft = -leftstickx + -leftsticky;
            reverseRight = leftstickx + leftsticky;
            leftsticky = gamepad1.left_stick_y;
            leftstickx = gamepad1.left_stick_x;

            Coordinate position = this.getPixelPosition(tfod);
            if (position == null) {
                telemetry.addData("position", "no objects detected");
            } else {
                telemetry.addData("position", "%.0f / %.0f", position.x, position.y);
            }


            imu.resetYaw();
            double yawAngleAngularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES).xRotationRate;
            double pitchAngleAngularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES).yRotationRate;
            double rollAngleAngularVelocity =  imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;

            double YawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double PitchAngle = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
            double RollAngle = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);

            YawAngle = (double) Math.round(YawAngle * 10) / 10.0;
            PitchAngle = (double) Math.round(PitchAngle * 10) / 10.0;
            RollAngle = (double) Math.round(RollAngle * 10) / 10.0;
            yawAngleAngularVelocity = (double) Math.round(yawAngleAngularVelocity * 10) / 10.0;
            pitchAngleAngularVelocity = (double) Math.round(pitchAngleAngularVelocity * 10) / 10.0;
            rollAngleAngularVelocity = (double) Math.round(rollAngleAngularVelocity * 10) / 10.0;

            telemetry.addData("yawAngle",YawAngle);
            telemetry.addData("pitchAngle",PitchAngle);
            telemetry.addData("rollAngle",RollAngle);
            telemetry.addData("yawAngleVelocity",yawAngleAngularVelocity);
            telemetry.addData("pitchAngleVelocity",pitchAngleAngularVelocity);
            telemetry.addData("rollAngleVelocity",rollAngleAngularVelocity);
            telemetry.update();
        }
    }
    public class Coordinate {
        double x = 0;
        double y = 0;

        Coordinate(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    private void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }

    }
    public Coordinate getPixelPosition(TfodProcessor tfod) {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        Coordinate position = null;
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            //telemetry.addData("", " ");
            //telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            //telemetry.addData("- Position", "%.0f / %.0f", x, y);

            position = new Coordinate(x, y);
        }   // end for() loop}
        //telemetry.update();
        return position;
    }
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
}