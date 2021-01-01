package org.firstinspires.ftc.teamcode.team10515.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.team10515.UltimateGoalRobot;
import org.firstinspires.ftc.teamcode.team10515.states.IState;

import static org.firstinspires.ftc.teamcode.team10515.Robot.getD1;
import static org.firstinspires.ftc.teamcode.team10515.Robot.getD2;
import static org.firstinspires.ftc.teamcode.team10515.Robot.getL1;
import static org.firstinspires.ftc.teamcode.team10515.Robot.getL2;
import static org.firstinspires.ftc.teamcode.team10515.Robot.getRobotLength;
import static org.firstinspires.ftc.teamcode.team10515.Robot.getWheelRadius;

public class RobotStateEstimator implements ISubsystem {
    private UltimateGoalRobot skystoneRobot;
    private BNO055IMU imu;
    private volatile Pose2d pose;
    private volatile Pose2d lastPose;
    private Pose2d velocityPose;

    public RobotStateEstimator(UltimateGoalRobot skystoneRobot, BNO055IMU imu, Pose2d initialPose) {
        setSkystoneRobot(skystoneRobot);
        setImu(imu);
        setPose(initialPose);
        setLastPose(initialPose);
        setVelocityPose(new Pose2d());
        initializeIMU();
    }

    @Override
    public IState getStateMachine() {
        return null;
    }

    @Override
    public Enum getState() {
        return null;
    }

    @Override
    public void start() {
        setPose(new Pose2d());
    }

    @Override
    public void update(double dt) {
        double backLeftVelocity = getSkystoneRobot().getDrive().getBackLeft().getVelocity() / getWheelRadius();
        double frontLeftVelocity = getSkystoneRobot().getDrive().getFrontLeft().getVelocity() / getWheelRadius();
        double backRightVelocity = getSkystoneRobot().getDrive().getBackRight().getVelocity() / getWheelRadius();
        double frontRightVelocity = getSkystoneRobot().getDrive().getFrontRight().getVelocity() / getWheelRadius();
        Rotation2d heading = getHeading();

        //Angle heading = new Angle(getImu().getAngularOrientation().firstAngle, AngularUnits.DEGREES);

        //1 - Front left
        //2 - Front right
        //3 - Back left
        //4 - Back right

        double cosPsi = heading.cos();
        double sinPsi = heading.sin();
        double denominator = (getD1() + getD2() + getL1() + getL2()) * getWheelRadius();

        //The convention here is that x is forward and y is left
        double velocityX = getWheelRadius() * ((-frontLeftVelocity + frontRightVelocity + backLeftVelocity - backRightVelocity) * sinPsi +
                (frontLeftVelocity + frontRightVelocity + backLeftVelocity + backRightVelocity) * cosPsi) / 4d;
        double velocityY = getWheelRadius() * ((-frontLeftVelocity + frontRightVelocity + backLeftVelocity - backRightVelocity) * cosPsi -
                (frontLeftVelocity + frontRightVelocity + backLeftVelocity + backRightVelocity) * sinPsi) / 4d;
        double angularVelocity = -(getWheelRadius() * (frontLeftVelocity - frontRightVelocity + backLeftVelocity - backRightVelocity)) / (8d * getRobotLength());
        /*double velocityX = frontLeftVelocity * (cosPsi + sinPsi) / getWheelRadius() +
                frontRightVelocity * (-(2 * getD1() + getL1() + getL2()) * cosPsi +
                        (-getL1() + getL2()) * sinPsi) / (denominator) +
                backLeftVelocity * ((getD1() - getD2()) * cosPsi + (getD1() + getD2() + 2 * getL1())
                        * sinPsi) / denominator;
        double velocityY = frontLeftVelocity * (cosPsi - sinPsi) / getWheelRadius() +
                frontRightVelocity * ((-getL1() + getL2()) * cosPsi + (2 * getD1() + getL1() +
                        getL2()) * sinPsi) / denominator + backLeftVelocity *
                ((getD1() + getD2() + 2 * getL1()) * cosPsi + (-getD1() + getD2()) * sinPsi) / denominator;*/
        setVelocityPose(new Pose2d(-velocityY, velocityX, getHeading().rotateBy(getPose().getRotation().inverse()).multiply(1 / dt, false)));
        setLastPose(getPose());
        setPose(new Pose2d(getPose().getTranslation().translateBy(new Translation2d(-velocityY * dt, velocityX * dt)), heading));
        //setPose(getPose().transformBy(new Pose2d(-velocityY * dt, velocityX * dt, new Rotation2d(heading.getRadians() - getPose().getRotation().getRadians(), true))));
    }

    @Override
    public void stop() {

    }

    /*public DoubleMatrix getDriveState() {
        return new DoubleMatrix(6, 1, new double[] {
                getPose().getTranslation().y(),
                getVelocityPose().getTranslation().y(),
                -getPose().getTranslation().x(),
                -getVelocityPose().getTranslation().x(),
                getPose().getRotation().getRadians(),
                getVelocityPose().getRotation().getRadians()
        });
    }*/

    public SimpleMatrix getDriveState() {
        return new SimpleMatrix(6, 1, false, new double[] {
                getPose().getTranslation().y(),
                getVelocityPose().getTranslation().y(),
                -getPose().getTranslation().x(),
                -getVelocityPose().getTranslation().x(),
                getPose().getRotation().getRadians(),
                getVelocityPose().getRotation().getRadians()
        });
    }

    public void initializeIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile  = "BNO055IMUCalibration.json";
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new AccelerationIntegrator();

        getImu().initialize(parameters);
    }

    public Rotation2d getHeading() {
        return new Rotation2d(Math.toRadians(getImu().getAngularOrientation().firstAngle), false);
    }

    public void resetPose() {
        setPose(Pose2d.identity());
        setVelocityPose(Pose2d.identity());
    }

    @Override
    public String toString() {
        return "Position: " + getPose();
    }

    public Pose2d getPose() {
        return pose;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public BNO055IMU getImu() {
        return imu;
    }

    public void setImu(BNO055IMU imu) {
        this.imu = imu;
    }

    public Pose2d getVelocityPose() {
        return velocityPose;
    }

    public void setVelocityPose(Pose2d velocityPose) {
        this.velocityPose = velocityPose;
    }

    public UltimateGoalRobot getSkystoneRobot() {
        return skystoneRobot;
    }

    public void setSkystoneRobot(UltimateGoalRobot skystoneRobot) {
        this.skystoneRobot = skystoneRobot;
    }

    public Pose2d getLastPose() {
        return lastPose;
    }

    public void setLastPose(Pose2d lastPose) {
        this.lastPose = lastPose;
    }

    @Override
    public String getName() {
        return null;
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }
}
