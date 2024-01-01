package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.PoseEstimator;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.function.DoubleSupplier;

public class Chassis implements Subsystem {

    private HardwareMap map;
    ElapsedTime time = new ElapsedTime();
    private PoseEstimator m_poseEstimator;
    private Telemetry m_telemetry;
    private MotorEx motor_FL;
    private MotorEx motor_FR;
    private MotorEx motor_BL;
    private MotorEx motor_BR;
    private MotorEx encoderLeft;
    private MotorEx encoderRight;
    private MotorEx encoderCenter;
    private Pose2d m_postitionFromTag;
    HolonomicOdometry odometry;
    public Chassis(HardwareMap map, Telemetry telemetry, Pose2d positionFromTag, PoseEstimator poseEstimator, ElapsedTime time){
        this.map=map;
        this.m_telemetry=telemetry;
        this.m_postitionFromTag = positionFromTag;
        this.m_poseEstimator = poseEstimator;
        motor_FL = new MotorEx(map, "motor_FL");
        motor_FR = new MotorEx(map, "motor_FR");
        motor_BL = new MotorEx(map, "motor_BL");
        motor_BR = new MotorEx(map, "motor_BR");
        encoderLeft = new MotorEx(map, "left odometer");
        encoderRight = new MotorEx(map, "right odometer");
        encoderCenter = new MotorEx(map, "right odometer");
        HolonomicOdometry odometry = new HolonomicOdometry(
                () -> encoderLeft.getCurrentPosition() * Constants.TICKS_TO_CM,
                () -> encoderRight.getCurrentPosition() * Constants.TICKS_TO_CM,
                () -> encoderCenter.getCurrentPosition() * Constants.TICKS_TO_CM,
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
        if (Constants.TimeToAprilTagCheck > time.seconds()) {
            m_poseEstimator.setPoseToCameraPose(m_postitionFromTag);
            time.reset();
        }
    }
    @Override
    public void setDefaultCommand(Command defaultCommand) {
        Subsystem.super.setDefaultCommand(defaultCommand);
    }
}
