package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RoadRunnerMecanumDrive extends MecanumDrive {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx xEncoder, yEncoder;  // One encoder for each axis
    private BNO055IMU imu;
    
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0.5, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0.5, 0, 0);
    
    private TrajectoryFollower follower;
    
    // Tracking wheel constants
    private static final double TICKS_PER_REV = 8192;  // REV Through Bore Encoder
    private static final double WHEEL_RADIUS = 0.75;  // inches
    private static final double GEAR_RATIO = 1.0;     // output (wheel) speed / input (encoder) speed
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (2 * Math.PI * WHEEL_RADIUS * GEAR_RATIO);
    
    public RoadRunnerMecanumDrive(HardwareMap hardwareMap) {
        super(TRANSLATIONAL_PID, HEADING_PID, 2.0, 2.0);  // X and Y offsets for encoders
        
        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right");
        
        // Initialize encoders
        xEncoder = hardwareMap.get(DcMotorEx.class, "x_encoder");
        yEncoder = hardwareMap.get(DcMotorEx.class, "y_encoder");
        
        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        
        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        
        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize trajectory follower
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID);
    }
    
    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        frontLeft.setPower(v);
        frontRight.setPower(v1);
        backLeft.setPower(v2);
        backRight.setPower(v3);
    }
    
    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
    
    @Override
    protected double getHeading() {
        return getRawExternalHeading();
    }
    
    @Override
    public Double getHeadingVelocity() {
        return (double) imu.getAngularVelocity().zRotationRate;
    }
    
    @Override
    public Pose2d getPoseEstimate() {
        double x = xEncoder.getCurrentPosition() / TICKS_PER_INCH;
        double y = yEncoder.getCurrentPosition() / TICKS_PER_INCH;
        double heading = getRawExternalHeading();
        return new Pose2d(x, y, heading);
    }
    
    public void followTrajectory(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
    }
    
    public boolean isBusy() {
        return follower.isFollowing();
    }
    
    @Override
    public void update() {
        updatePoseEstimate();
        Pose2d currentPose = getPoseEstimate();
        
        if (follower != null && follower.isFollowing()) {
            follower.update(currentPose);
            setDriveSignal(follower.update(currentPose));
        }
    }
    
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, constraints);
    }
    
    public void turn(double angle) {
        double startHeading = getRawExternalHeading();
        double targetHeading = startHeading + angle;
        
        while (Math.abs(getRawExternalHeading() - targetHeading) > Math.toRadians(1)) {
            double error = targetHeading - getRawExternalHeading();
            double power = error * HEADING_PID.kP;
            
            // Cap the power
            power = Math.min(Math.max(power, -0.5), 0.5);
            
            setMotorPowers(-power, power, -power, power);
        }
        
        setMotorPowers(0, 0, 0, 0);
    }
} 