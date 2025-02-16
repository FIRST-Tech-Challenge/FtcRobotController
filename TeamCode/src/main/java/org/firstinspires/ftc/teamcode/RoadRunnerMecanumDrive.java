package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class RoadRunnerMecanumDrive extends MecanumDrive {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;
    
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);
    
    public static double LATERAL_MULTIPLIER = 1;
    
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;
    
    private TrajectoryFollower follower;
    
    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();
    
    public RoadRunnerMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);
        
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right");
        
        motors = Arrays.asList(frontLeft, backLeft, backRight, frontRight);
        
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        
        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }
    
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }
    
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }
    
    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
    }
    
    public void update() {
        updatePoseEstimate();
        DriveSignal signal = follower.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) {
            Pose2d vel = signal.getVel();
            setMotorPowers(
                vel.getX() - vel.getY() - vel.getHeading(),  // frontLeft
                vel.getX() + vel.getY() - vel.getHeading(),  // backLeft
                vel.getX() - vel.getY() + vel.getHeading(),  // backRight
                vel.getX() + vel.getY() + vel.getHeading()   // frontRight
            );
        }
    }
    
    @Override
    public void setDriveSignal(DriveSignal driveSignal) {
        setMotorPowers(driveSignal.getVel().getX(),
                driveSignal.getVel().getY(),
                driveSignal.getVel().getHeading(),
                driveSignal.getVel().getHeading());
    }
    
    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        frontLeft.setPower(v);
        backLeft.setPower(v1);
        backRight.setPower(v2);
        frontRight.setPower(v3);
    }
    
    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
    
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }
    
    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }
    
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    
    public boolean isBusy() {
        return follower.isFollowing();
    }
    
    public void turn(double angle) {
        double error = angle - getRawExternalHeading();
        Pose2d startPose = getPoseEstimate();
        startPose = new Pose2d(startPose.getX(), startPose.getY(), angle);
        
        Trajectory trajectory = trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(startPose.getX(), startPose.getY(), angle))
                .build();
        
        followTrajectory(trajectory);
    }
    
    // Constants
    public static double WHEEL_RADIUS = 2.0; // in
    public static double GEAR_RATIO = 1.0; // output (wheel) speed / input (encoder) speed
    public static double TRACK_WIDTH = 1.0; // in
    public static double WHEEL_BASE = 1.0; // in
    
    public static double TICKS_PER_REV = 1.0;
    
    public static double MAX_VEL = 30;
    public static double MAX_ACCEL = 30;
    public static double MAX_ANG_VEL = Math.toRadians(180);
    public static double MAX_ANG_ACCEL = Math.toRadians(180);
    
    public static double kV = 1.0 / rpmToVelocity(getMaxRpm());
    public static double kA = 0;
    public static double kStatic = 0;
    
    private static double getMaxRpm() {
        return 0; // Replace with actual max RPM
    }
    
    private static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }
    
    private static TrajectoryVelocityConstraint VEL_CONSTRAINT = new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(MAX_ANG_VEL),
            new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
    ));
    private static TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = new ProfileAccelerationConstraint(MAX_ACCEL);
} 