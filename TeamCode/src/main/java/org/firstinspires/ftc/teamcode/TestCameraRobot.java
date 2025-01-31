package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp(name = "TestCameraRobot", group = "Test")
public class TestCameraRobot extends LinearOpMode {

    public DcMotor motorRightFront = null;
    public DcMotor motorRightBack = null;
    public DcMotor motorLeftBack = null;
    public DcMotor motorLeftFront = null;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        // Set these so that motorRightFront actually maps to the right front motor
        // I don't think I have it right here
        // Even better would be to rename these in the hardware config to have descriptive names
        // instead of m0, m1, m2, m3
        //  -Nic
        motorLeftBack = hardwareMap.dcMotor.get("m2");
        motorLeftFront = hardwareMap.dcMotor.get("m3");
        motorRightFront = hardwareMap.dcMotor.get("m0");
        motorRightBack = hardwareMap.dcMotor.get("m1");

        // Calling setPower with a positive value should rotate the wheel forward
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        initAprilTag();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0) {
                directDriveControl(0.5);
            } else {
                directDriveControl();
            }

            telemetryAprilTag();
            telemetry.update();

            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            sleep(20);  // Sleeping here allows the CPU to catch up with other tasks
        }
        visionPortal.close();
    }

    private void directDriveControl() {
        directDriveControl(1.0);
    }

    private void directDriveControl(double speedMultiplier) {
        double max;

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double r = gamepad1.right_stick_x;

        // Check out TeamCode/src/main/java/org/firstinspires/ftc/teamcode/docs/OmniWheelControlDerivation.md
        // for the derivation of these equations.
        double powerRightFront = -x + y - r;
        double powerRightBack  =  x + y - r;
        double powerLeftBack   = -x + y + r;
        double powerLeftFront  =  x + y + r;

        max = Math.max(Math.max(Math.abs(powerLeftFront), Math.abs(powerRightBack)),
                Math.max(Math.abs(powerLeftBack), Math.abs(powerLeftFront)));

        // If any individual motor power is greater than 1.0, scale all values to fit in the range [-1.0, 1.0]
        if (max > 1.0) {
            powerRightFront  /= max;
            powerRightBack   /= max;
            powerLeftBack    /= max;
            powerLeftFront   /= max;
        }

        motorRightFront.setPower(powerRightFront * speedMultiplier);
        motorRightBack.setPower(powerRightBack * speedMultiplier);
        motorLeftBack.setPower(powerLeftBack * speedMultiplier);
        motorLeftFront.setPower(powerLeftFront * speedMultiplier);
    }

    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getPosition().z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }
}
