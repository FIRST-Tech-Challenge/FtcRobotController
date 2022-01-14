package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.lib.drive.ArcadeDrive;
import org.firstinspires.ftc.teamcode.lib.drive.HorizontalDrive;
import org.firstinspires.ftc.teamcode.lib.drive.Odometry;
import org.firstinspires.ftc.teamcode.lib.drive.TankDrive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;


import static org.commandftc.RobotUniversal.hardwareMap;
import static org.commandftc.RobotUniversal.opMode;

import org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants;
import org.firstinspires.ftc.teamcode.lib.tragectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.lib.tragectory.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.lib.tragectory.TrajectorySequenceRunner;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class DriveTrainSubsystem extends MecanumDrive implements TankDrive, ArcadeDrive, HorizontalDrive {
    private final DcMotor m_FrontLeftMotor;
    private final DcMotor m_RearLeftMotor;
    private final DcMotor m_FrontRightMotor;
    private final DcMotor m_RearRightMotor;

    private final BNO055IMU imu;

    public static PIDCoefficients FORWARD_PID = new PIDCoefficients(2, 0, 0);
    public static PIDCoefficients STRAFE_PID = new PIDCoefficients(7, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(5, 0, 0);

    private final TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(6, Math.toRadians(165), 0.28);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(0.4);

    private final boolean trajectoryControled;

    public boolean trajectories = true;

    public DriveTrainSubsystem() {
        super(DriveTrainConstants.kV, DriveTrainConstants.kA, DriveTrainConstants.kStatic, 0.259, 1);
        setLocalizer(new Odometry(
                new DoubleSupplier[]{
                        this::getLeftEncodersAvg,
                        this::getRightEncodersAvg
                },
                this::getHeading
        ));
        m_FrontLeftMotor  =  hardwareMap.dcMotor.get("FrontLeftDriveMotor");
        m_RearLeftMotor   =  hardwareMap.dcMotor.get("RearLeftDriveMotor");
        m_FrontRightMotor =  hardwareMap.dcMotor.get("FrontRightDriveMotor");
        m_RearRightMotor  =  hardwareMap.dcMotor.get("RearRightDriveMotor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Default drive type
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        m_RearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        m_FrontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        m_RearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        TrajectoryFollower follower = new HolonomicPIDVAFollower(FORWARD_PID, STRAFE_PID, HEADING_PID,
                new Pose2d(0.1, 0.1, Math.toRadians(0.5)), 0.5);

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);

//        trajectoryControled = !CommandBasedTeleOp.class.isAssignableFrom(opMode.getClass());
        trajectoryControled = false;

        // Because we aren't extending SubsystemBase
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        updatePoseEstimate();
//        if (trajectoryControled && trajectories) {
//            DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
//            if (signal != null) setDriveSignal(signal);
//        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        m_FrontLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        m_RearLeftMotor.setZeroPowerBehavior(zeroPowerBehavior);
        m_FrontRightMotor.setZeroPowerBehavior(zeroPowerBehavior);
        m_RearRightMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return m_FrontLeftMotor.getZeroPowerBehavior();
    }

    public void setDriveMode(DcMotor.RunMode mode) {
        m_FrontLeftMotor.setMode(mode);
        m_RearLeftMotor.setMode(mode);
        m_FrontRightMotor.setMode(mode);
        m_RearRightMotor.setMode(mode);
    }

    public DcMotor.RunMode getDriveMode() {
        return m_FrontLeftMotor.getMode();
    }

    @Override
    public void stop() {
        m_FrontLeftMotor.setPower(0);
        m_RearLeftMotor.setPower(0);
        m_FrontRightMotor.setPower(0);
        m_RearRightMotor.setPower(0);
    }

    public void driveForward(double power) {
        m_FrontLeftMotor.setPower(power);
        m_RearLeftMotor.setPower(power);
        m_FrontRightMotor.setPower(power);
        m_RearRightMotor.setPower(power);
    }

    @Override
    public void tankDrive(double left, double right) {
        m_FrontLeftMotor.setPower(left);
        m_RearLeftMotor.setPower(left);
        m_FrontRightMotor.setPower(right);
        m_RearRightMotor.setPower(right);
    }

    @Override
    public void arcadeDrive(double x, double y,double spin) {
        m_FrontLeftMotor.setPower(x + y + spin);
        m_RearLeftMotor.setPower(-x + y + spin);
        m_FrontRightMotor.setPower(-x + y - spin);
        m_RearRightMotor.setPower(x + y - spin);
    }

    public void setPowers(double frontLeft, double rearLeft, double frontRight, double rearRight) {
        m_FrontLeftMotor.setPower(frontLeft);
        m_RearLeftMotor.setPower(rearLeft);
        m_FrontRightMotor.setPower(frontRight);
        m_RearRightMotor.setPower(rearRight);
    }

    @Override
    public void driveLeft(double power) {
        m_FrontLeftMotor.setPower(-power);
        m_RearLeftMotor.setPower(power);
        m_FrontRightMotor.setPower(power);
        m_RearRightMotor.setPower(-power);
    }

    @Override
    public void driveRight(double power) {
        m_FrontLeftMotor.setPower(power);
        m_RearLeftMotor.setPower(-power);
        m_FrontRightMotor.setPower(-power);
        m_RearRightMotor.setPower(power);
    }

    public int getFrontLeftEncoder() {
        return m_FrontLeftMotor.getCurrentPosition();
    }

    public int getRearLeftEncoder() {
        return m_RearRightMotor.getCurrentPosition();
    }

    public int getFrontRightEncoder() {
        return m_FrontRightMotor.getCurrentPosition();
    }

    public int getRearRightEncoder() {
        return m_RearRightMotor.getCurrentPosition();
    }

    public int getLeftEncodersAvg() {
        return (getFrontLeftEncoder() + getRearLeftEncoder()) / 2;
    }

    public int getRightEncodersAvg() {
        return (getFrontRightEncoder() + getRearRightEncoder()) / 2;
    }

    public double getFrontLeftPosition() {
        return DriveTrainConstants.mm_to_ticks.apply(getFrontLeftEncoder());
    }

    public double getRearLeftPosition() {
        return DriveTrainConstants.mm_to_ticks.apply(m_RearRightMotor.getCurrentPosition());
    }

    public double getFrontRightPosition() {
        return DriveTrainConstants.mm_to_ticks.apply(m_FrontRightMotor.getCurrentPosition());
    }

    public double getRearRightPosition() {
        return DriveTrainConstants.mm_to_ticks.apply(m_RearRightMotor.getCurrentPosition());
    }

    public double getLeftPositionsAvg() {
        return (getFrontLeftPosition() + getRearLeftPosition()) / 2;
    }

    public double getRightPositionsAvg() {
        return (getFrontRightPosition() + getRearRightPosition()) / 2;
    }

    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    protected double getRawExternalHeading() {
        return getHeading();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(getFrontLeftPosition(), getRearLeftPosition(), getFrontRightPosition(), getRearRightPosition());
    }

    @Override
    public void setMotorPowers(double fl, double rl, double fr, double rr) {
        setPowers(fl, rl, fr, rr);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, TrajectoryVelocityConstraint VEL_CONSTRAINT) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                Math.toRadians(165), Math.toRadians(165)
        );
    }

    @NotNull
    @Contract("_, _, _ -> new")
    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    @NotNull
    @Contract("_ -> new")
    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void followTrajectoryAsync(@NotNull Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }
}