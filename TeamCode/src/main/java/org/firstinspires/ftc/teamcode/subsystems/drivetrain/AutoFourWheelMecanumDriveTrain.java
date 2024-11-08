package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.Tunables;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.util.Units;

/**
 * four wheel mecanum drive train for auto op mode, that hold references to odometers
 */
public class AutoFourWheelMecanumDriveTrain extends FourWheelMecanumDrive {

    private MecanumDriveKinematics driveKinematics;
    GoBildaPinpointDriver odo;

    IMU internalIMU;

    public AutoFourWheelMecanumDriveTrain(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        super(hardwareMap, gamepad, telemetry, feedback);

        Translation2d frontLeftWheelMeters = new Translation2d(-Tunables.TRACK_WIDTH / 2, Tunables.WHEEL_BASE / 2);
        Translation2d frontRightWheelMeters = new Translation2d(Tunables.TRACK_WIDTH / 2, Tunables.WHEEL_BASE / 2);
        Translation2d rearLeftWheelMeters = new Translation2d(-Tunables.TRACK_WIDTH / 2, -Tunables.WHEEL_BASE / 2);
        Translation2d rearRightWheelMeters = new Translation2d(Tunables.TRACK_WIDTH / 2, -Tunables.WHEEL_BASE / 2);

        driveKinematics = new MecanumDriveKinematics(
                frontLeftWheelMeters, frontRightWheelMeters,
                rearLeftWheelMeters, rearRightWheelMeters);
    }

    @Override
    protected void createAndInitHardwares(HardwareMap hardwareMap) {
        super.createAndInitHardwares(hardwareMap);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        internalIMU = hardwareMap.get(IMU.class, "imu");
        internalIMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));
        internalIMU.resetYaw();
    }

    private void initOdo(Runnable sleeper) {
        boolean initResult = odo.initialize();
        sleeper.run();
        odo.resetPosAndIMU();
        sleeper.run();
        sleeper.run();
        RobotLog.i("GoBildaPinpointDriver init result: " + initResult);
        telemetry.addData("init result", initResult);
        telemetry.update();

        telemetry.addData("odo status after init", odo.getDeviceStatus());
        telemetry.update();
        odo.setOffsets(Units.inchesToMMs(-3.5), Units.inchesToMMs(0));
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
    }

    public void init(Runnable sleeper) {
        initOdo(sleeper);
    }

    public void recalibrateOdo() {
        odo.recalibrateIMU();
    }

    public Pose2d getCurrentPose() {
        return toPose2d(odo.getPosition());
    }

    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }

    public void resetOdo() {
        odo.resetPosAndIMU();
    }

    public void driveFieldCentric(double strafeSpeed, double forwardSpeed,
                                  double turnSpeed) {
        drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, 0);
    }

    public void updatePose() {
        odo.update();
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

    public double getHeadingFromInternalIMUAsRadians() {
        return internalIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    private static Pose2d toPose2d(GoBildaPinpointDriver.Pose2D pose2D) {
        return new Pose2d(pose2D.getX(DistanceUnit.METER), pose2D.getY(DistanceUnit.METER),
                new Rotation2d(pose2D.getHeading(AngleUnit.RADIANS)));
    }
}
