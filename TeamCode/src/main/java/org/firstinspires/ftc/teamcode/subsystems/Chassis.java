package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.teamcode.Constants.*;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.utils.PoseEstimator;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Chassis implements Subsystem {

    private HardwareMap map;
    ElapsedTime time = new ElapsedTime();
    private PoseEstimator m_poseEstimator;
    private Telemetry m_telemetry;
    private MotorEx motor_FL;
    private MotorEx motor_FR;
    private MotorEx motor_BL;
    private MotorEx motor_BR;
    private Motor leftEncoder;
    private Motor rightEncoder;
    private Motor horizontalEncoder;
    private Pose2d m_postitionFromTag;
    HolonomicOdometry odometry;
    public Chassis(HardwareMap map, Telemetry telemetry, Supplier<Integer> poseL, Supplier<Integer> poseR){
        this.map=map;
        this.m_telemetry=telemetry;
        motor_FL = new MotorEx(map, "motor_FL");
        motor_FR = new MotorEx(map, "motor_FR");
        motor_BL = new MotorEx(map, "motor_BL");
        motor_BR = new MotorEx(map, "motor_BR");
        leftEncoder = new MotorEx(map, "encoderLeft");
        rightEncoder = new MotorEx(map, "encoderRight");
        horizontalEncoder = new MotorEx(map, "encoderCenter");
        horizontalEncoder.resetEncoder();
        leftEncoder.resetEncoder();
        rightEncoder.resetEncoder();


         odometry = new HolonomicOdometry(
                () -> metersFormTicks(leftEncoder.getCurrentPosition()) ,
                () -> metersFormTicks(rightEncoder.getCurrentPosition()),
                () -> metersFormTicks(horizontalEncoder.getCurrentPosition()),
                Constants.TRACKWIDTH, Constants.WHEEL_OFFSET);
        time.startTime();

    }

    public void setMotors (double FL, double FR, double BL, double BR){
        motor_FR.set(FR);
        motor_FL.set(FL);
        motor_BR.set(BR);
        motor_BL.set(BL);
    }


    public Command drive(DoubleSupplier x, DoubleSupplier theta, DoubleSupplier y) {
        return new RunCommand(()->{
            double r = Math.hypot(y.getAsDouble(), theta.getAsDouble());
            double robotAngle = Math.atan2(y.getAsDouble(), theta.getAsDouble()) - Math.PI / 4;//shifts by 90 degrees so that 0 is to the right
            double rightX = x.getAsDouble();
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;
            setMotors(v1, v2, v3, v4);
        },this);
    }


    @Override
    public void periodic() {
        odometry.updatePose();
//        if (Constants.TimeToAprilTagCheck > time.seconds()) {
//            m_poseEstimator.setPoseToCameraPose(m_postitionFromTag);
//            time.reset()
//
//        }
        m_telemetry.addData("gtth x: ", 363645);
        m_telemetry.addData("pose x: ", odometry.getPose().getX());
        m_telemetry.addData("pose y: ", odometry.getPose().getY());
        m_telemetry.update();
    }

    @Override
    public void setDefaultCommand(Command defaultCommand) {
        Subsystem.super.setDefaultCommand(defaultCommand);
    }
    public double metersFormTicks(int ticks){
        return (ticks/(double) tickPerRevolution)*(2*odometryWheelRadius*Math.PI);
    }
}
