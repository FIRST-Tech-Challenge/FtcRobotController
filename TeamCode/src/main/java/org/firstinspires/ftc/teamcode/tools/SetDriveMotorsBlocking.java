package org.firstinspires.ftc.teamcode.tools;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;

@TeleOp
@Disabled
public class SetDriveMotorsBlocking extends OpMode {

    public final double DEADZONE_MIN_Y = 0.1;
    public final double DEADZONE_MIN_X = 0.25;
    public final double AUTOBRAKE_DISTANCE = 0.5;
    private final DeadzoneSquare horizontalFastDeadzone;
    private final DeadzoneSquare verticalFastDeadzone;
    private final DeadzoneSquare horizontalSlowDeadzone;
    private final DeadzoneSquare verticalSlowDeadzone;
    private final DcMotor backLeftMotor;
    private final DcMotor frontLeftMotor;
    private final DcMotor backRightMotor;
    private final DcMotor frontRightMotor;
    private final Gamepad gamepad1;
    private final IMU imu;
    public double powerValues[] = new double[4];
    private SampleMecanumDrive drive;

    protected enum DriveMode{
        FIELD_CENTRIC,
        ROBOT_CENTRIC
    }

    private DriveMode driveMode = DriveMode.FIELD_CENTRIC;

    static final double BACKDROP_APPROACH_SPEED = -0.25;

    //map the motors and run the op mode
    public SetDriveMotorsBlocking(HardwareMap hardwareMap, Gamepad gamepad1) {
        // Initialize SampleMecanumDrive
        //drive = new SampleMecanumDrive(hardwareMap);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        /*if (AutoDataStorage.comingFromAutonomous){
            drive.setPoseEstimate(AutoDataStorage.currentPose);
        }

        else {
            //Pose2d rightParkFinalBlueLeft = new Pose2d(-11.5, 50, Math.toRadians(270.00));
            drive.setPoseEstimate(new Pose2d(-11.5, 50, Math.toRadians(270.00)));
        }*/

        this.gamepad1 = gamepad1;
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");

        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse right side motor directions
        // This may need to be flipped to the left side depending on your motor rotation direction
        //frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double deadzone = 0.1;

        horizontalFastDeadzone = new DeadzoneSquare(deadzone, DEADZONE_MIN_X, 0.9); //10% joystick deadzone. 0.25 is minimum power to strafe. 0.75 is the max power.
        verticalFastDeadzone = new DeadzoneSquare(deadzone, DEADZONE_MIN_Y, 0.7);
        horizontalSlowDeadzone = new DeadzoneSquare(deadzone, DEADZONE_MIN_X, 0.5);
        verticalSlowDeadzone = new DeadzoneSquare(deadzone, DEADZONE_MIN_Y, 0.4);
    }
    public void driveCommands(double horizontal, double vertical, double turn, boolean goFast, boolean switchDriveMode) {
        // Read pose


        //deadzones
        if (goFast) {
            horizontal = horizontalFastDeadzone.computePower(horizontal);
            vertical = verticalFastDeadzone.computePower(vertical);
        } else {
            horizontal = horizontalSlowDeadzone.computePower(horizontal);
            vertical = verticalSlowDeadzone.computePower(vertical);
            turn *= 0.6;
        }

        if(switchDriveMode) {
            driveMode = driveMode== DriveMode.ROBOT_CENTRIC? DriveMode.FIELD_CENTRIC : DriveMode.ROBOT_CENTRIC;
        }

        if(driveMode == DriveMode.ROBOT_CENTRIC){


            double rotX = horizontal;
            double rotY = vertical;

            double rotationalCorrection = 1.1; // original value of code on site was 1.1

            rotX = rotX * rotationalCorrection;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);

            // uses the values calculated via the imu's yaw to create accurate movement
            double frontLeftPower = (rotY + rotX + turn) / denominator;
            double backLeftPower = (rotY - rotX + turn) / denominator;
            double frontRightPower = (rotY - rotX - turn) / denominator;
            double backRightPower = (rotY + rotX - turn) / denominator;

            // Set motor power based on the calculated values
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
        else {
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = horizontal * Math.cos(-botHeading) - vertical * Math.sin(-botHeading);
            double rotY = horizontal * Math.sin(-botHeading) + vertical * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
            double frontLeftPower = (rotY + rotX + turn) / denominator;
            double backLeftPower = (rotY - rotX + turn) / denominator;
            double frontRightPower = (rotY - rotX - turn) / denominator;
            double backRightPower = (rotY + rotX - turn) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }

    private boolean isAutoBrake(double distanceToWallMeters, Pose2d poseEstimate) {
        return distanceToWallMeters != 0 &&
                distanceToWallMeters < AUTOBRAKE_DISTANCE &&
                poseEstimate.getY() > 36 &&
                ((poseEstimate.getX() > -57 && poseEstimate.getX() < -15) ||
                        (poseEstimate.getX() < 57 && poseEstimate.getX() > 15));
    }

    public void update() {
            drive.update();
    }


    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}