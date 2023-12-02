package org.firstinspires.ftc.teamcode.subsystems;

import android.hardware.HardwareBuffer;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ClimberSubsystem extends SubsystemBase {
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    Servo m_servo;
    DcMotorEx m_motor;
    double m_positionSetpoint = 0.0;
    public static int ticPosition = 9200;
    int m_targetPosition = 0;

    public static double max_pos = 1.0  ;
    public static double min_pos = 0.0;

    public static double downPose = 0.45;
    public static double upPose = 0.8;

    public static double k_p = 8.0;
    public static double p = 12.0;
    public static double i = 0.0;
    public static double d = 1.0;
    public static double f = 15.0;

    public static int zero = -3;

    public ClimberSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
        m_motor = m_hardwareMap.get(DcMotorEx.class, "climber");
        m_servo = m_hardwareMap.get(Servo.class, "CS");
        m_targetPosition = 0;
        MotorConfigurationType motorConfigurationType = m_motor.getMotorType();
        motorConfigurationType.setAchieveableMaxRPMFraction(0.99);
        motorConfigurationType.setMaxRPM(435);

        m_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_motor.setDirection(DcMotorSimple.Direction.FORWARD);

        m_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_motor.setTargetPosition(0);
        m_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_motor.setPower(0.8);

        m_positionSetpoint = downPose;

    }

    public void extendAndClimb(){
        m_positionSetpoint = upPose;
        m_targetPosition = ticPosition;
    }

    public void retractMotor(){

       m_targetPosition = zero;
    }

    public void retractServo(){
        m_positionSetpoint = downPose;
    }
    public void resetToZero(){
        m_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(!m_motor.isOverCurrent()) {
            m_motor.setPower(-0.25);
        }
        m_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_motor.setTargetPosition(0);
        m_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public Command climb(){
        return new InstantCommand(()-> this.extendAndClimb());
    }

    public Command retract(){
        return new InstantCommand(()-> this.retractMotor());
    }

    public Command retractS(){
        return new InstantCommand(()-> this.retractServo());
    }

    public Command reset(){
        return new InstantCommand(()-> this.resetToZero());
    }


    @Override
    public void periodic(){
        m_servo.setPosition(m_positionSetpoint);
        m_servo.scaleRange(min_pos, max_pos);
        m_motor.setTargetPosition(m_targetPosition);
        m_motor.setVelocityPIDFCoefficients(p, i, d, f);
        m_motor.setPositionPIDFCoefficients(k_p);

    }


}
