package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.Tunables;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;

/**
 * four wheel mecanum drive train for auto op mode, that hold references to odometers
 */
public class AutoFourWheelMecanumDriveTrain extends FourWheelMecanumDrive {

    private final Pose2d INITIAL_POSE = new Pose2d(0, 0, new Rotation2d(0));

    private DcMotor leftOdometry, rightOdometry;
    private DcMotor centerOdometry;

//    private final HolonomicOdometry odometry;
    private final DifferentialDriveOdometry odometry;

    private MecanumDriveKinematics driveKinematics;

    IMU imu;

    public AutoFourWheelMecanumDriveTrain(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        super(hardwareMap, gamepad, telemetry, feedback);

//        odometry = new HolonomicOdometry(
//                () -> motorEncoderTicksToMeters(leftOdometry.getCurrentPosition()),
//                () -> motorEncoderTicksToMeters(rightOdometry.getCurrentPosition()),
//                this::getHeading,
//                Tunables.TRACK_WIDTH,
//                Tunables.CENTER_WHEEL_OFFSET);

        odometry = new DifferentialDriveOdometry(new Rotation2d(getHeadingRadians()), INITIAL_POSE);

        Translation2d frontLeftWheelMeters = new Translation2d(Tunables.TRACK_WIDTH / 2, Tunables.WHEEL_BASE / 2);
        Translation2d frontRightWheelMeters = new Translation2d(-Tunables.TRACK_WIDTH / 2, Tunables.WHEEL_BASE / 2);
        Translation2d rearLeftWheelMeters = new Translation2d(Tunables.TRACK_WIDTH / 2, -Tunables.WHEEL_BASE / 2);
        Translation2d rearRightWheelMeters = new Translation2d(-Tunables.TRACK_WIDTH / 2, -Tunables.WHEEL_BASE / 2);

        driveKinematics = new MecanumDriveKinematics(
                frontLeftWheelMeters, frontRightWheelMeters,
                rearLeftWheelMeters, rearRightWheelMeters);
    }

    @Override
    protected void createAndInitHardwares(HardwareMap hardwareMap) {
        super.createAndInitHardwares(hardwareMap);
        leftOdometry = hardwareMap.dcMotor.get("BL"); //new Motor(hardwareMap, "LO");
        rightOdometry = hardwareMap.dcMotor.get("BR"); //new Motor(hardwareMap, "RO");
//        centerOdometry = hardwareMap.dcMotor.get("FR"); //.dcMotor.get("CO"); //new Motor(hardwareMap, "CO");

        leftOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdometry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        centerOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        centerOdometry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(imuParameters);
        imu.resetYaw();
    }

    public Pose2d getCurrentPose() {
        return odometry.getPoseMeters();
    }

    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }

    public void driveFieldCentric(double strafeSpeed, double forwardSpeed,
                                  double turnSpeed) {
        drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, 0);
    }

    public void updatePose() {
        odometry.update(
                new Rotation2d(getHeadingRadians()),
                motorEncoderTicksToMeters(leftOdometry.getCurrentPosition()),
                motorEncoderTicksToMeters(rightOdometry.getCurrentPosition()));

    }

    public void resetPose(Pose2d pose) {
        leftOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdometry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOdometry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        odometry.resetPosition(pose, new Rotation2d(getHeadingRadians()));
    }

    public MecanumDriveKinematics getDriveKinematics() {
        return driveKinematics;
    }

    private static double odometryPodTicksToMeters(double ticks) {
        return (ticks / Tunables.ODOMETER_POD_TICKS_PER_REVOLUTION) * Tunables.ODOMETER_POD_WHEEL_DIAMETER_MM * Math.PI / 1000;
    }

    private static double motorEncoderTicksToMeters(double ticks) {
        return (ticks / Tunables.MOTOR_ENCODER_TICKS_PER_REVOLUTION) * Tunables.MECANUM_WHEEL_DIAMETER_MM * Math.PI / 1000;
    }

    protected double getHeadingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}
