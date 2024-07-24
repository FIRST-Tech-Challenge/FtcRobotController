
import com.qualcomm.hardware.bosch.BNO055IMU;
import java.util.Arrays;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import java.util.concurrent.TimeUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.concurrent.TimeUnit;

import java.util.List;
import java.util.ArrayList;
@Disabled
@TeleOp(name = "Main")
public class MainUnorganised extends LinearOpMode {

    private static ElapsedTime timer = new ElapsedTime();
    //vision
    private Servo gimbalPitch = null;
    private Servo gimbalYaw = null;

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;

    private List<Integer> aprilTagIds;
    private int currentIndex = 0;
    private int decimation = 2;

    private IMU imu;

    private double lastTime = 0; // create last varriables for traction control
    private double lastSpeedLeftFrontDrive = 0;
    private double lastSpeedLeftBackDrive = 0;
    private double lastSpeedRightBackDrive = 0;
    private double lastSpeedRightFrontDrive = 0;
    private final double accelerationLimit = 20; //RPSS revolutions per second  second 
    private boolean TractionControlstatus = false;

    private DcMotor leftFrontDrive = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive = null;  //  Used to control the right back drive wheel

    private DcMotorEx leftFrontDriveEx = null;  //  Used to control the left front drive wheel
    private DcMotorEx rightFrontDriveEx = null;  //  Used to control the right front drive wheel
    private DcMotorEx leftBackDriveEx = null;  //  Used to control the left back drive wheel
    private DcMotorEx rightBackDriveEx = null;  //  Used to control the right back drive wheel

    @Override
    public void runOpMode() {
        gamepad1.rumble(0.5, 0.5, 1000);
        gamepad2.rumble(0.5, 0.5, 1000);

        // Initialize the list with predefined AprilTag IDs
        aprilTagIds = Arrays.asList(583, 584, 585, 586);

        double currentPitch = 0.5;
        double currentYaw = 0.5;
        double sameXAprilTag = 0;
        double errorsCatchedAmmount = 0;

        initAprilTag();

        gimbalPitch = hardwareMap.get(Servo.class, "Servo_Port_0_CH");
        gimbalPitch.setPosition(currentPitch);
        gimbalYaw = hardwareMap.get(Servo.class, "Servo_Port_1_CH");
        gimbalYaw.setPosition(currentYaw);

        if (USE_WEBCAM) {
            setManualExposure(6, 250);
        }

        leftFrontDrive = hardwareMap.get(DcMotor.class, "Motor_Port_1_CH");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Motor_Port_0_CH");
        leftBackDrive = hardwareMap.get(DcMotor.class, "Motor_Port_2_CH");
        rightBackDrive = hardwareMap.get(DcMotor.class, "Motor_Port_3_CH");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        if (true) {
            leftFrontDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_1_CH");
            rightFrontDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_0_CH");
            leftBackDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_2_CH");
            rightBackDriveEx = hardwareMap.get(DcMotorEx.class, "Motor_Port_3_CH");

            leftFrontDriveEx.setDirection(DcMotorEx.Direction.FORWARD);
            leftBackDriveEx.setDirection(DcMotorEx.Direction.FORWARD);
            rightFrontDriveEx.setDirection(DcMotorEx.Direction.REVERSE);
            rightBackDriveEx.setDirection(DcMotorEx.Direction.REVERSE);
        }

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Wait for the start button to be pressed
        imu.resetYaw();
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a && TractionControlstatus) {
                TractionControlstatus = false;
                telemetry.addData("Tractioncontrol = ", TractionControlstatus);
            } else if (gamepad1.a && !TractionControlstatus) {
                TractionControlstatus = true;
                telemetry.addData("Tractioncontrol = ", TractionControlstatus);
            }

            for (AprilTagDetection detection : aprilTag.getDetections()) {
                try {
                    Orientation rot = Orientation.getOrientation(detection.rawPose.R, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Original source data
                    double poseX = detection.rawPose.x;
                    double poseY = detection.rawPose.y;
                    double poseZ = detection.rawPose.z;

                    double poseAX = rot.firstAngle;
                    double poseAY = rot.secondAngle;
                    double poseAZ = rot.thirdAngle;
                    telemetry.addData("xTag", poseX);
                    telemetry.addData("zTag", poseZ);
                    telemetry.addData("yTag", poseY);
                    telemetry.addData("compensation", Math.pow(Math.toDegrees(Math.asin(poseX / poseZ) / 270), 3));
                    telemetry.addData("compensation", -Math.pow(Math.toDegrees(Math.asin(poseY / poseZ)) / 270, 3));
                    telemetry.addData("same?", true && sameXAprilTag != poseX);
                    telemetry.addData("cycle time", System.currentTimeMillis() - lastTime);
                    lastTime = timer.time();
                    //telemetry.update();
                    if (true && sameXAprilTag != poseX) {

                        double yawAngleOffset = Math.toDegrees(Math.asin(poseX / poseZ));
                        currentYaw += Math.pow(yawAngleOffset / 270, 3);

                        double pitchAngleOffset = Math.toDegrees(Math.asin(poseY / poseZ));
                        currentPitch -= Math.pow(pitchAngleOffset / 270, 3);
                    }
                    sameXAprilTag = poseX;
                } catch (Exception e) {
                    errorsCatchedAmmount += 1;
                }
            }

            // Check controller inputs to navigate through the list
            if (gamepad1.dpad_up) {
                currentIndex = (currentIndex - 1 + aprilTagIds.size()) % aprilTagIds.size();
                sleep(250); // Prevent rapid cycling
            } else if (gamepad1.dpad_down) {
                currentIndex = (currentIndex + 1) % aprilTagIds.size();
                sleep(250); // Prevent rapid cycling
            }
            if (gamepad1.dpad_left) {
                decimation = Math.max(1, decimation - 1);
                aprilTag.setDecimation(decimation);
                sleep(250); // Prevent rapid cycling
            } else if (gamepad1.dpad_right) {
                decimation = Math.min(5, decimation + 1);
                aprilTag.setDecimation(decimation);
                sleep(250); // Prevent rapid cycling
            }
            gimbalPitch.setPosition(currentPitch);
            gimbalYaw.setPosition(currentYaw);

            // Get the acceleration data from the IMU
            // Display acceleration data on telemetry
            telemetry.addData("gyro", imu.getRobotYawPitchRollAngles());

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if (!gamepad1.left_bumper) { // Field centric drive when in manual mode (autodrive button not pressed)
                double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double x = drive * Math.cos(heading) - strafe * Math.sin(heading);
                double y = drive * Math.sin(heading) + strafe * Math.cos(heading);

                moveRobot(x, y, turn);
            } else { // Inputs for automatic mode driving
                moveRobot(drive, strafe, turn);
            }
            telemetry.update();
        }
    }

    public void moveRobot(double drive, double strafe, double turn) {
        // Calculates raw power to motors
        double leftFrontPowerRaw = drive + strafe + turn;
        double leftBackPowerRaw = drive - strafe + turn;
        double rightFrontPowerRaw = drive + strafe - turn;
        double rightBackPowerRaw = drive - strafe - turn;

        // Calculate the maximum absolute power value for normalization
        double maxRawPower = Math.max(Math.max(Math.abs(leftFrontPowerRaw), Math.abs(leftBackPowerRaw)),
                Math.max(Math.abs(rightFrontPowerRaw), Math.abs(rightBackPowerRaw)));

        double max = Math.max(maxRawPower, 1.0);
        double maxradian = 1972.92;

        // Calculate wheel speeds normalized to the wheels.
        double leftFrontRawSpeed = (leftFrontPowerRaw / max * maxradian / 1.2039);
        double leftBackRawSpeed = (leftBackPowerRaw / max * maxradian);
        double rightFrontRawSpeed = (rightFrontPowerRaw / max * maxradian / 1.2039);
        double rightBackRawSpeed = (rightBackPowerRaw / max * maxradian);
        if (TractionControlstatus) {
            tractionControl(leftFrontRawSpeed, leftBackRawSpeed, rightFrontRawSpeed, rightBackRawSpeed);
        } else {
            leftBackDriveEx.setVelocity(leftBackRawSpeed);
            leftFrontDriveEx.setVelocity(leftFrontRawSpeed);
            rightBackDriveEx.setVelocity(rightBackRawSpeed);
            rightFrontDriveEx.setVelocity(rightFrontRawSpeed);
        }

    }

    private void tractionControl(double speedLeftFrontDrive, double speedLeftBackDrive, double speedRightBackDrive, double speedRightFrontDrive) {
        double deltaTime = System.currentTimeMillis();
        double deltaSpeedLeftFrontDrive = speedLeftFrontDrive;
        double deltaSpeedLeftBackDrive = speedLeftBackDrive;
        double deltaSpeedRightFrontDrive = speedRightFrontDrive;
        double deltaSpeedRightBackDrive = speedRightBackDrive;
        double maxDelta = deltaTime * accelerationLimit;
        double maxMultiplier = 1;

        if (Math.abs(deltaSpeedLeftBackDrive / maxDelta) > Math.abs(maxMultiplier)) {
            maxMultiplier = deltaSpeedLeftBackDrive / maxDelta;
        }
        if (Math.abs(deltaSpeedLeftFrontDrive / maxDelta) > Math.abs(maxMultiplier)) {
            maxMultiplier = deltaSpeedLeftFrontDrive / maxDelta;
        }
        if (Math.abs(deltaSpeedRightBackDrive / maxDelta) > Math.abs(maxMultiplier)) {
            maxMultiplier = deltaSpeedRightBackDrive / maxDelta;
        }
        if (Math.abs(deltaSpeedRightFrontDrive / maxDelta) > Math.abs(maxMultiplier)) {
            maxMultiplier = deltaSpeedRightFrontDrive / maxDelta;
        }

        leftBackDriveEx.setVelocity(speedLeftBackDrive - deltaSpeedLeftBackDrive + deltaSpeedLeftBackDrive / maxMultiplier);
        leftFrontDriveEx.setVelocity(speedLeftFrontDrive - deltaSpeedLeftFrontDrive + deltaSpeedLeftFrontDrive / maxMultiplier);
        rightBackDriveEx.setVelocity(speedRightBackDrive - deltaSpeedRightBackDrive + deltaSpeedRightBackDrive / maxMultiplier);
        rightFrontDriveEx.setVelocity(speedRightFrontDrive - deltaSpeedRightFrontDrive + deltaSpeedRightFrontDrive / maxMultiplier);

        lastTime = System.currentTimeMillis();
        lastSpeedLeftFrontDrive = speedLeftFrontDrive;
        lastSpeedLeftBackDrive = speedLeftBackDrive;
        lastSpeedRightBackDrive = speedRightBackDrive;
        lastSpeedRightFrontDrive = speedRightFrontDrive;

        telemetry.addData("left back wanted speed", speedLeftBackDrive);
        telemetry.addData("left back got speed", speedLeftBackDrive - deltaSpeedLeftBackDrive + deltaSpeedLeftBackDrive / maxMultiplier);
        telemetry.addData("left front wanted speed", speedLeftFrontDrive);
        telemetry.addData("left front got speed", speedLeftFrontDrive - deltaSpeedLeftFrontDrive + deltaSpeedLeftFrontDrive / maxMultiplier);
    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(decimation);

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
