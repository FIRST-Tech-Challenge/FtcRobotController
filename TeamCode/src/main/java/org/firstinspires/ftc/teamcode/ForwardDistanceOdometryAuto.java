package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Autonomous(name = "ForwardDistanceOdometryAuto", preselectTeleOp = "IronEagle-GoBilda-Strafer")
public class ForwardDistanceOdometryAuto extends LinearOpMode {

    private DcMotor rightFrontDrive;
    private DcMotor rightRearDrive;
    private DcMotor leftRearDrive;
    private DcMotor leftFrontDrive;
    private DcMotor paraDeadWheelLeft;
    private DcMotor paraDeadWheelRight;
    private DcMotor perpDeadWheel;
private VisionPortal visionPortal;
private TfodProcessor tfod;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    private int paraPositionLeft;
    private int paraPositionRight;
    private int perpPosition;
    private double current_forward_inches;
    private int target_forward_inches;

    double COUNTS_PER_INCH;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int COUNTS_PER_ENCODER_REV;
        int GEAR_REDUCTION;
        double WHEEL_CIRCUMFERENCE_INCHES;
        int COUNTS_PER_WHEEL_REV;
        double drivePower;
        initAprilTag();

        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        paraDeadWheelLeft = hardwareMap.get(DcMotor.class, "leftRearDrive");
        paraDeadWheelRight = hardwareMap.get(DcMotor.class, "rightRearDrive");
        perpDeadWheel = hardwareMap.get(DcMotor.class, "rightFrontDrive");

        // REV Robotics Through Bore Encoder specs
        COUNTS_PER_ENCODER_REV = 8192;
        // Dead Wheel to Encoder (no gears, so ratio is 1:1)
        GEAR_REDUCTION = 1;
        // Dual Omni 35mm (1.38 inches)
        WHEEL_CIRCUMFERENCE_INCHES = 1.38 * Math.PI;
        // Math to determine COUNTS_PER_INCH
        COUNTS_PER_WHEEL_REV = COUNTS_PER_ENCODER_REV * GEAR_REDUCTION;
        COUNTS_PER_INCH = COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_INCHES;

        // Set motor directions
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);

        // The 'Dead Wheels' will be the encoders we will use, so we disable the encoders on the drives themselves
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set 'Dead Wheels' to use encoder
        paraDeadWheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        paraDeadWheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpDeadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Establish current and target position
        getCurrentPositions();
        target_forward_inches = 36;

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.addData("WHEEL_CIRCUMFERENCE_INCHES", WHEEL_CIRCUMFERENCE_INCHES);
        telemetry.addData("COUNTS_PER_WHEEL_REV", COUNTS_PER_WHEEL_REV);
        telemetry.addData("COUNTS_PER_INCH", COUNTS_PER_INCH);

        List<Recognition> updatedRecognitions = tfod.getRecognitions();
        telemetry.addData("# Object Detected", updatedRecognitions.size());
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Move forward until target position is reached
            drivePower = 0.25;
            rightFrontDrive.setPower(drivePower);
            rightRearDrive.setPower(drivePower);
            leftRearDrive.setPower(drivePower);
            leftFrontDrive.setPower(drivePower);

            while (current_forward_inches < target_forward_inches) {
                getCurrentPositions();
                addPositionTelemetryData();
                telemetry.update();
            }

            // Position reached, stop movement
            addPositionTelemetryData();
            telemetry.addLine("Position reached, stop movement");
            telemetry.update();

            drivePower = 0;
            rightFrontDrive.setPower(drivePower);
            rightRearDrive.setPower(drivePower);
            leftRearDrive.setPower(drivePower);
            leftFrontDrive.setPower(drivePower);
            // Sleep for the remainder of auto
            sleep(30000);
        }
    }
    private void initAprilTag() {
        tfod=TfodProcessor.easyCreateWithDefaults();

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(848, 480))
                .addProcessor(aprilTag)
                .addProcessor(tfod)
                .build();

        /*
        // Add the AprilTagProcessor instance to the Vision Portal
        visionPortal.addProcessor(aprilTag);
        // Set the camera used for detection
        visionPortal.setCamera(new WebcamName("Webcam 1"), BuiltinCameraDirection.BACK);
        // Set the maximum number of objects to detect
        aprilTag.setMaximumObjects(1);
        // Set the minimum confidence required to register a detection
        aprilTag.setMinimumConfidence(0.0);
        // Set the minimum distance between two separate objects
        aprilTag.setSeparationDistance(0);
        // Set the minimum number of pixels for an object to register
        aprilTag.setMinimumPixelCount(0);
        // Set the maximum number of pixels for an object to register
        aprilTag.setMaximumPixelCount(Integer.MAX_VALUE);
        // Set the minimum aspect ratio for an object to register
        aprilTag.setMinimumAspectRatio(0);
        // Set the maximum aspect ratio for an object to register
        aprilTag.setMaximumAspectRatio(Integer.MAX_VALUE);
        // Set the minimum width for an object to register
        aprilTag.setMinimumWidth(0);
        // Set the maximum width for an object to register
        aprilTag.setMaximumWidth(Integer.MAX_VALUE);
        // Set the minimum height for an object to register
        aprilTag.setMinimumHeight(0);
        // Set the maximum height for an object to register
        aprilTag.setMaximumHeight(Integer.MAX_VALUE);
        // Set the minimum angle for an object to register
        aprilTag.setMinimumAngle(0);
        // Set the maximum angle for an object to register
        aprilTag.setMaximumAngle(Integer.MAX_VALUE);
        // Set the minimum perimeter for an object to register
        aprilTag.setMinimumPerimeter(0);
        // Set the maximum perimeter for an object to register
        aprilTag.setMaximumPerimeter(Integer.MAX_VALUE);
        // Set the minimum area for an object to register
        aprilTag.setMinimumArea(0);
        // Set the maximum area for an object to register
        aprilTag.setMaximumArea(Integer.MAX_VALUE);
        // Set the minimum width/height ratio for an object to register
        aprilTag.setMinimumWidthHeightRatio(0);
        // Set the maximum width/height ratio for an object to register
        aprilTag.setMaximumWidthHeightRatio(Integer.MAX
        */
    }
    private void getCurrentPositions() {
        paraPositionLeft = paraDeadWheelLeft.getCurrentPosition();
        paraPositionRight = paraDeadWheelRight.getCurrentPosition();
        perpPosition = perpDeadWheel.getCurrentPosition();
        current_forward_inches = paraPositionLeft / COUNTS_PER_INCH;
    }

    private void addPositionTelemetryData() {
        telemetry.addData("para position left", paraPositionLeft);
        telemetry.addData("para position right", paraPositionRight);
        telemetry.addData("perp position", perpPosition);
        telemetry.addData("current forward inches", current_forward_inches);
        telemetry.addData("target forward inches", target_forward_inches);
    }
}