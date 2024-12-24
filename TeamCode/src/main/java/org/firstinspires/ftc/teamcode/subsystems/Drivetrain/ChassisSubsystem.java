package org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import static org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisConstants.RotationPID.PIDConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisConstants.RotationPID.*;
import static org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisConstants.FeedForwardConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.PID.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.utils.PID.TrapezoidProfile;

public class ChassisSubsystem implements Subsystem{
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public ProfiledPIDController m_pidY;
    public ProfiledPIDController m_pidX;
    public ProfiledPIDController m_rotationpid;
    public MotorEx motor_FL;
    public MotorEx motor_FR;
    public MotorEx motor_BL;
    public MotorEx motor_BR;
    public RevIMU gyro;
    private HardwareMap map;
    private VoltageSensor voltageSensor;

    @Config
    public static class SpeedsAndAcc {
        public static double maxVelocityX = 0;
        public static double maxVelocityY = 0;
        public static double maxVelocityTheta = 0;
        public static double maxAccelerationX = 0;
        public static double maxAccelerationTheta = 0;
        public static double maxAccelerationY = 0;
    }


    public ChassisSubsystem(HardwareMap map){
        motor_FL = new MotorEx(map, "motor_FL");//tbd
        motor_FR = new MotorEx(map, "motor_FR");//tbd
        motor_BL = new MotorEx(map, "motor_BL");//tbd
        motor_BR = new MotorEx(map, "motor_BR");//tbd
        gyro = new RevIMU(map,"imu");
        voltageSensor = map.voltageSensor.iterator().next();

        m_pidX = new ProfiledPIDController(Xkp,Xki,Xkd,new TrapezoidProfile.Constraints(SpeedsAndAcc.maxVelocityX,SpeedsAndAcc.maxAccelerationX));
        m_pidY = new ProfiledPIDController(Ykp,Yki,Ykd, new TrapezoidProfile.Constraints(SpeedsAndAcc.maxVelocityY,SpeedsAndAcc.maxAccelerationY));
        m_rotationpid = new ProfiledPIDController( rkp,rki,rkd,new TrapezoidProfile.Constraints(180,180));

        m_rotationpid.enableContinuousInput(-180,180);
        m_rotationpid.setTolerance(tolerance);

        motor_FR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_BR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_BL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_FL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        updateTelemetry();

    }

    public void setMotors(double FL, double FR, double BL, double BR) {
        double compensation = 12.0 / voltageSensor.getVoltage();
        motor_FR.set(compensation * applyFeedFoward(ks, kv, FR));
        motor_FL.set(compensation * applyFeedFoward(ks, kv, FL));
        motor_BR.set(compensation * applyFeedFoward(ks, kv, BR));
        motor_BL.set(compensation * applyFeedFoward(ks, kv, BL));
        dashboardTelemetry.addData("compensation", compensation);
    }
    public double applyFeedFoward(double ks, double kv, double velocity){

        double s=velocity<0.01? 0: ks*Math.signum(velocity);// this is from kookybotz
        return s + kv * velocity;
    }

    @Override
    public void periodic(){
        updateTelemetry();
        updateValues();
    }

    private void updateTelemetry() {

    }

    private void updateValues() {

    }


}