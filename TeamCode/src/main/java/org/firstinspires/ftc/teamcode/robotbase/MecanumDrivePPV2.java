package org.firstinspires.ftc.teamcode.robotbase;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

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
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.myroadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.myroadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.myroadrunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.myroadrunner.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.kV;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.COMMON_FEED_FORWARD;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.TRANSLATIONAL_PID;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.HEADING_PID;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.KP;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.KI;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.KD;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.minIntegralBound;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.maxIntegralBound;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.LATERAL_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.frontLeftInverted;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.frontRightInverted;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.rearLeftInverted;
import static org.firstinspires.ftc.teamcode.myroadrunner.drive.DriveConstants.rearRightInverted;
import static org.firstinspires.ftc.teamcode.robotbase.RobotEx.OpModeType.AUTO;
import static org.firstinspires.ftc.teamcode.robotbase.RobotEx.OpModeType.TELEOP;
/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class MecanumDrivePPV2 extends MecanumDrive implements Subsystem {
    /* ----------------------------------------- COMMON ----------------------------------------- */
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;
    private MotorExEx frontLeft, frontRight, rearRight, rearLeft;
    private IMU imu;
    private List<MotorExEx> motors;
    private VoltageSensor batteryVoltageSensor;

    /* ----------------------------------------- COMMON ----------------------------------------- */
    protected String m_name = this.getClass().getSimpleName();
    private com.arcrobotics.ftclib.drivebase.MecanumDrive drive;

    /* --------------------------------------- AUTONOMOUS --------------------------------------- */
    private TrajectorySequenceRunner trajectorySequenceRunner;
    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(
            MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH
    );
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);
    private TrajectoryFollower follower;
    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();

    public MecanumDrivePPV2(HardwareMap hardwareMap, RobotEx.OpModeType type) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        /* --------------------------------------- COMMON --------------------------------------- */
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ));
        imu.initialize(parameters);

        frontLeft = new MotorExEx(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);
        frontRight = new MotorExEx(hardwareMap, "frontRight", Motor.GoBILDA.RPM_312);
        rearRight = new MotorExEx(hardwareMap, "rearRight", Motor.GoBILDA.RPM_312);
        rearLeft = new MotorExEx(hardwareMap, "rearLeft", Motor.GoBILDA.RPM_312);

        motors = Arrays.asList(frontLeft, frontRight, rearLeft, rearRight);

        setMotorsInverted(frontLeftInverted, frontRightInverted, rearRightInverted, rearLeftInverted);

        if (RUN_USING_ENCODER) setMode(MotorExEx.RunMode.VelocityControl);

        setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(KP, KI, KD, kStatic, kV, kA);

            if (!COMMON_FEED_FORWARD) {
                frontLeft.setFeedforwardCoefficients(150, 1.1, 0);//2795
                frontRight.setFeedforwardCoefficients(120, 0.97, 0);//2795
                rearLeft.setFeedforwardCoefficients(120, 1, 0);//2795
                rearRight.setFeedforwardCoefficients(220, 1.07, 0);//2795
            }
        }

        if (type == AUTO){
            /* ------------------------------------- AUTONOMOUS ------------------------------------- */
            follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                    new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

            List<Integer> lastTrackingEncPositions = new ArrayList<>();
            List<Integer> lastTrackingEncVels = new ArrayList<>();

            // TODO: if desired, use setLocalizer() to change the localization method
            trajectorySequenceRunner = new TrajectorySequenceRunner(
                    follower, HEADING_PID
            );
        }
        if (type == TELEOP){
            /* --------------------------------------- TELEOP --------------------------------------- */
            CommandScheduler.getInstance().registerSubsystem(this);
            drive = new com.arcrobotics.ftclib.drivebase.MecanumDrive(
                    frontLeft, frontRight, rearLeft, rearRight
            );
        }
    }

    public String getName()
    {
        return m_name;
    }
    public void setName(String name)
    {
        m_name = name;
    }
    public String getSubsystem()
    {
        return getName();
    }
    public void setSubsystem(String subsystem)
    {
        setName(subsystem);
    }
    void drive(double strafeSpeed, double forwardSpeed, double turnSpeed, double heading,
               double maxSpeed)
    {
        drive.setMaxSpeed(0.3 * (1 + (7.0/3)*maxSpeed));
        drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, heading);
    }
    public void setMotorsInverted(
            boolean leftFrontInverted, boolean rightFrontInverted,
            boolean rightRearInverted, boolean leftRearInverted
    )
    {
        frontLeft.setInverted(leftFrontInverted);
        rearLeft.setInverted(leftRearInverted);
        frontRight.setInverted(rightFrontInverted);
        rearRight.setInverted(rightRearInverted);
    }
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose)
    {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed)
    {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading)
    {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }
    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose)
    {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }
    public void turnAsync(double angle)
    {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }
    public void turn(double angle)
    {
        turnAsync(angle);
        waitForIdle();
    }
    public void followTrajectoryAsync(Trajectory trajectory)
    {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }
        public void followTrajectory(Trajectory trajectory)
    {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }
    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence)
    {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }
    public void followTrajectorySequence(TrajectorySequence trajectorySequence)
    {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }
    public Pose2d getLastError()
    {
        return trajectorySequenceRunner.getLastPoseError();
    }
    public void update()
    {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }
    public void waitForIdle()
    {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }
    public boolean isBusy()
    {
        return trajectorySequenceRunner.isBusy();
    }
    public void setMode(Motor.RunMode mode)
    {
        for (MotorExEx motor : motors)
            motor.setRunMode(mode);
    }
    public void setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior zeroPowerBehavior)
    {
        for (MotorExEx motor : motors)
            motor.setZeroPowerBehavior(zeroPowerBehavior);
    }
    public void setPIDFCoefficients(double kP, double kI, double kD, double kF, double kV, double kA)
    {
        double batteryPercentage = 12 / batteryVoltageSensor.getVoltage();

        for (MotorExEx motor : motors) {
            motor.setIntegralBounds(minIntegralBound, maxIntegralBound);
            motor.setFeedforwardCoefficients(kF * batteryPercentage, kV * batteryPercentage, kA);
            motor.setVeloCoefficients(kP, kI, kD);
        }
    }
    public void setWeightedDrivePower(Pose2d drivePower)
    {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }
    @NonNull
    @Override
    public List<Double> getWheelPositions()
    {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        for (MotorExEx motor : motors) {
            int position = motor.getCurrentPosition();
            lastEncPositions.add(position);
            wheelPositions.add(encoderTicksToInches(position));
        }
        return wheelPositions;
    }
    @Override
    public List<Double> getWheelVelocities()
    {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (MotorExEx motor : motors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }
    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3)
    {
        frontLeft.set(v);
        rearLeft.set(v1);
        rearRight.set(v2);
        frontRight.set(v3);
    }
    @Override
    public double getRawExternalHeading()
    {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
    @Override
    public Double getExternalHeadingVelocity()
    {
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }
    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth)
    {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }
    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel)
    {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}