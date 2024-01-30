package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.BTposeEstimator;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.BTCommand;
import org.firstinspires.ftc.teamcode.utils.RunCommand;
import org.firstinspires.ftc.teamcode.utils.geometry.*;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.BTposeEstimator;

import static org.firstinspires.ftc.teamcode.Constants.ChassisConstants.*;
import static org.firstinspires.ftc.teamcode.Constants.ChassisConstants.PIDConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Chassis implements Subsystem {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private PIDFController m_pidcontroller;
    private double prevTime = 0;
    private Transform2d velocity, prevVelocity = new Transform2d(), acceleration, prevAcceleration = new Transform2d();
    private Pose2d prevPos;
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
    private double maxVelocityX = 0;
    private double maxVelocityY = 0;
    private double maxVelocityTheta = 0;
    private double maxAccelerationX = 0;
    private double maxAccelerationTheta = 0;
    private double maxAccelerationY = 0;

    ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    public Chassis(HardwareMap map, Telemetry telemetry, MotorEx.Encoder leftEncoder, MotorEx.Encoder rightEncoder) {
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
        prevPos = odometry.getPose();
        time.reset();
        time.startTime();
        prevTime = time.time();
        m_pidcontroller = new PIDFController(kp, ki, kd, kff);
        register();
        dashboardTelemetry.addData("drive: ", 0);
    }

    public void setMotors(double FL, double FR, double BL, double BR) {
        motor_FR.set(FR);
        motor_FL.set(FL);
        motor_BR.set(BR);
        motor_BL.set(BL);
    }

    ElapsedTime driveTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    boolean isFirstTime = true;
    //x is front facing, y is
    public BTCommand drive(DoubleSupplier frontVel, DoubleSupplier sidewayVel, DoubleSupplier retaliation) {
        return new RunCommand(() -> {
            if (isFirstTime == true) {
                driveTimer.reset();
                isFirstTime=false;

            }
            dashboardTelemetry.addData("drive: ",driveTimer.time());
            drive(frontVel.getAsDouble(), sidewayVel.getAsDouble(), retaliation.getAsDouble());
        }, this);

    }

    public BTCommand fieldRelativeDrive(DoubleSupplier frontVel, DoubleSupplier sidewayVel, DoubleSupplier retaliation) {
        return new RunCommand(() -> {
            m_telemetry.addData("front", frontVel);
            m_telemetry.update();
            Translation2d vector = new Translation2d(sidewayVel.getAsDouble(), frontVel.getAsDouble());
            Translation2d rotated = vector.rotateBy(Rotation2d.fromDegrees(-gyro.getHeading()));
            double wantedAngle = new Rotation2d(sidewayVel.getAsDouble(), frontVel.getAsDouble()).getDegrees();
            double correctPositionVec = m_pidcontroller.calculate(odometry.getPose().getRotation().getDegrees(), wantedAngle);

            drive(rotated.getY(), rotated.getX(), correctPositionVec + retaliation.getAsDouble());
        }, this);
    }

    public BTCommand stopMotor() {
        return new RunCommand(() -> setMotors(0, 0, 0, 0));
    }

    @Override
    public void periodic() {
        m_pidcontroller.setPIDF(kp, ki, kd, kff);
        odometry.updatePose();
        calcVA();
        dashboardTelemetry.addData("pose y: ", odometry.getPose().getY());
        dashboardTelemetry.addData("gyro angle: ", gyro.getHeading());
        dashboardTelemetry.addData("x:", odometry.getPose().getX());

        dashboardTelemetry.addData("Theta velocity : ", velocity.getRotation().getDegrees());
        dashboardTelemetry.addData("X velocity : ", velocity.getTranslation().getX());
        dashboardTelemetry.addData("Y velocity : ", velocity.getTranslation().getY());

        dashboardTelemetry.addData("X Acc : ", acceleration.getTranslation().getX());
        dashboardTelemetry.addData("Y Acc: ", acceleration.getTranslation().getY());
        dashboardTelemetry.addData("Theta Acc : ", acceleration.getRotation().getDegrees());

        dashboardTelemetry.update();
    }

    public void calcVA() {
        double dt = time.time() - prevTime;
        velocity = odometry.getPose().minus(prevPos).times(1 / dt);
        acceleration = velocity.minus(prevVelocity).times(1 / dt);


        prevPos = odometry.getPose();
        prevVelocity = velocity;
        prevAcceleration = acceleration;
        prevTime = time.time();

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


