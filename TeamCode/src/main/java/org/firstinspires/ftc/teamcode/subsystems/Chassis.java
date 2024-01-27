package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.BTposeEstimator;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.BTCommand;
import org.firstinspires.ftc.teamcode.utils.RunCommand;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.BTposeEstimator;

import static org.firstinspires.ftc.teamcode.Constants.ChassisConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Chassis implements Subsystem {

    private HardwareMap map;
    private BTposeEstimator odometry;
    private Telemetry m_telemetry;
    private MotorEx motor_FL;
    private MotorEx motor_FR;
    private MotorEx motor_BL;
    private MotorEx motor_BR;
    private RevIMU gyro;
    private Motor leftEncoder;
    private Motor rightEncoder;
    private Motor horizontalEncoder;
    private Pose2d m_postitionFromTag;

    public Chassis(HardwareMap map, Telemetry telemetry, MotorEx.Encoder leftEncoder, MotorEx.Encoder rightEncoder) {
        register();
        this.map = map;
        this.m_telemetry = telemetry;
        motor_FL = new MotorEx(map, "motor_FL");//1
        motor_FR = new MotorEx(map, "motor_FR");//2
        motor_BL = new MotorEx(map, "motor_BL");//0
        motor_BR = new MotorEx(map, "motor_BR");//3

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro = new RevIMU(map, "imu");
        gyro.init(parameters);
        gyro.reset();
        horizontalEncoder = new MotorEx(map, "encoderCenter");
        horizontalEncoder.resetEncoder();
        leftEncoder.reset();
        rightEncoder.reset();
        odometry = new BTposeEstimator(
                () -> -metersFormTicks(leftEncoder.getPosition()),
                () -> metersFormTicks(rightEncoder.getPosition()),
                () -> metersFormTicks(horizontalEncoder.getCurrentPosition()),
                () -> gyro.getHeading(),
                TRACKWIDTH, WHEEL_OFFSET);
    }

    public void setMotors(double FL, double FR, double BL, double BR) {
        motor_FR.set(FR);
        motor_FL.set(FL);
        motor_BR.set(BR);
        motor_BL.set(BL);
    }

    //x is front facing, y is
    public BTCommand drive(DoubleSupplier frontVel, DoubleSupplier sidewayVel, DoubleSupplier retaliation) {
        return new RunCommand(() -> {
            m_telemetry.addData("front", frontVel);
            m_telemetry.update();
            drive(frontVel.getAsDouble(), sidewayVel.getAsDouble(), retaliation.getAsDouble());
        }, this);
    }

    public BTCommand fieldRelativeDrive(DoubleSupplier frontVel, DoubleSupplier sidewayVel, DoubleSupplier retaliation) {
        return new RunCommand(() -> {
            m_telemetry.addData("front", frontVel);
            m_telemetry.update();
            Translation2d vector = new Translation2d(sidewayVel.getAsDouble(), frontVel.getAsDouble());
            Translation2d rotated = vector.rotateBy(Rotation2d.fromDegrees(-gyro.getHeading()));
            drive(rotated.getY(), rotated.getX(), retaliation.getAsDouble());
        }, this);
    }

    public BTCommand stopMotor() {
        return new RunCommand(() -> setMotors(0, 0, 0, 0));
    }

    @Override
    public void periodic() {

        odometry.updatePose();
//        if (Constants.TimeToAprilTagCheck > time.seconds()) {
//            m_poseEstimator.setPoseToCameraPose(m_postitionFromTag);
//            time.reset()
//
//        }
        m_telemetry.addData("pose x: ", odometry.getPose().getX());
        m_telemetry.addData("pose y: ", odometry.getPose().getY());
        m_telemetry.addData("pose angle: ", odometry.getPose().getRotation().getDegrees());
        m_telemetry.addData("gyro angle: ", gyro.getHeading());
        m_telemetry.update();
    }

    @Override
    public void setDefaultCommand(Command defaultCommand) {
        Subsystem.super.setDefaultCommand(defaultCommand);
    }

    public double metersFormTicks(int ticks) {
        return (ticks / (double) tickPerRevolution) * (2 * odometryWheelRadius * Math.PI);
    }

    private void drive(double frontVel, double sidewayVel, double retaliation) {
        double r = Math.hypot(retaliation, sidewayVel);
        double robotAngle = Math.atan2(retaliation, sidewayVel) - Math.PI / 4;//shifts by 90 degrees so that 0 is to the right
        double rightX = frontVel;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        setMotors(v1, v2, v3, v4);
    }
}
