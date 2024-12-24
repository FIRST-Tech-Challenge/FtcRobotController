package org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import static org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisConstants.RotationPID.PIDConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisConstants.RotationPID.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.PID.PIDController;
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

    @Config
    public static class SpeedsAndAcc {
        public static double maxVelocityX = 0;
        public static double maxVelocityY = 0;
        public static double maxVelocityTheta = 0;
        public static double maxAccelerationX = 0;
        public static double maxAccelerationTheta = 0;
        public static double maxAccelerationY = 0;
    }

    public static class ChassisMotorsFeedfoward{
        @Config
        public static class chassisFL{
            public static double ks=0.085;//0.085 is a value from kookybotz
            public static double kv=1;
        }@Config
        public static class chassisFR{
            public static double ks=0.085;//0.085 is a value from kookybotz
            public static double kv=1;
        }@Config
        public static class chassisBL{
            public static double ks=0.085;//0.085 is a value from kookybotz
            public static double kv=1;
        }@Config
        public static class chassisBR{
            public static double ks=0.085;//0.085 is a value from kookybotz
            public static double kv=1;
        }
    }

    public ChassisSubsystem(HardwareMap map){
        motor_FL = new MotorEx(map, "motor_FL");//tbd
        motor_FR = new MotorEx(map, "motor_FR");//tbd
        motor_BL = new MotorEx(map, "motor_BL");//tbd
        motor_BR = new MotorEx(map, "motor_BR");//tbd
        gyro = new RevIMU(map,"imu");

        m_pidX = new ProfiledPIDController(Xkp,Xki,Xkd,new TrapezoidProfile.Constraints(SpeedsAndAcc.maxVelocityX,SpeedsAndAcc.maxAccelerationX));
        m_pidY = new ProfiledPIDController(Ykp,Yki,Ykd, new TrapezoidProfile.Constraints(SpeedsAndAcc.maxVelocityY,SpeedsAndAcc.maxAccelerationY));
        m_rotationpid = new ProfiledPIDController( rkp,rki,rkd,new TrapezoidProfile.Constraints(180,180));
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