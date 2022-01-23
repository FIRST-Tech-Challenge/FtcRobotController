package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.TrikeDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import static org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants.*;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

@Config
public class TrikeDriveImpl extends TrikeDrive implements Subsystem {
    public static PIDCoefficients AXIAL_PID = new PIDCoefficients(0.01, 0, 0.01);
    public static PIDCoefficients CROSS_TRACK_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0.01, 0, 0);

    public static double VX_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectorySequenceRunner trajectorySequenceRunner;
    private TrajectoryFollower follower;

    private List<DcMotorEx> motors;
    private DcMotorEx leftMotor, rightMotor, swerveMotor, swivelMotor; // swerveMotor drives the module, swivelMotor rotates the module
    private DcMotorEx duckSpinner;
    private BNO055IMU imu;

    private VoltageSensor batteryVoltageSensor;

    private double heading,

    public TrikeDriveImpl(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH);
        follower = new TankPIDVAFollower(AXIAL_PID, CROSS_TRACK_PID, new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);
        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for(LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        leftMotor = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        rightMotor = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        swerveMotor = hardwareMap.get(DcMotorEx.class, "motorMiddle");
        swivelMotor = hardwareMap.get(DcMotorEx.class, "motorMiddleSwivel");
        duckSpinner = hardwareMap.get(DcMotorEx.class,"duckSpinner");
        motors = Arrays.asList(leftMotor, rightMotor, swerveMotor, swivelMotor, duckSpinner);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);

            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        swivelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new TankVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public double getExternalHeadingVelocity() {
        return (double) -imu.getAngularVelocity().xRotationRate;
    }


    @Override
    public double getChassisLength() {
        return ;
    }

    @Override
    public List<Double> getWheelPositions() {
        return null;
    }

    @Override
    public List<Double> getWheelVelocities() {
        return null;
    }

    @Override
    public List<Double> getModuleOrientations() {
        return null;
    }

    @Override
    public void setModuleOrientations(double swerve) {

    }

    @Override
    public void setMotorPowers(double left, double right, double swerve) {

    }

    @Override
    public void update() {

    }

    @Override
    public void stop() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        return null;
    }

    @Override
    public String getTelemetryName() {
        return null;
    }
}
