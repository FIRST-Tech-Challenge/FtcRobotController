package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends SubsystemBase {
    Telemetry m_telemetry;
    Double kS, kCos, kV, kA;
    PIDController m_pid;
    MotorEx m_armMotor;


    public Arm(MotorEx armMotor, Telemetry telemetry) {

        m_armMotor = armMotor;
        m_telemetry = telemetry;
        kS = 0.0;
        kCos = 0.0;
        kV = 0.0;
        kA = 0.0;

        ArmFeedforward feedforward = new ArmFeedforward(kS, kCos, kV, kA);
        telemetry.addData("Arm Values", "kS: %d, kCos: %d, kV: %d, kA: %d", kS, kCos, kV, kA);

        m_telemetry.addLine("Arm Initialized");

    }


    @Override
    public void periodic() {

    }

    public void driveToSetPoint(Integer setpoint) {
        m_pid.setSetPoint(setpoint);
        m_telemetry.addData("Arm Setpoint", m_pid.getSetPoint());
    }

    public void drive(Double speed) {
        m_armMotor.set(speed);
    }
}
