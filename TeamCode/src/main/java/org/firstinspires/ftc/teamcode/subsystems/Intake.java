package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends SubsystemBase {
    Telemetry m_telemetry;
    CRServo m_intakeLeft, m_intakeRight;
    Double m_leftmultiplier, m_rightmultiplier;



    public Intake(CRServo intakeLeft, CRServo intakeRight, Telemetry telemetry) {

        m_telemetry = telemetry;
        m_intakeLeft = intakeLeft;
        m_intakeRight = intakeRight;
        m_leftmultiplier = 1.0;
        m_rightmultiplier = 1.0;

        m_telemetry.addLine("Intake Initialized");

    }


    @Override
    public void periodic() {
        m_telemetry.update();
    }

    public void runIntake(Double speed) {

        m_intakeLeft.set(speed * m_leftmultiplier);
        m_intakeRight.set(-speed * m_leftmultiplier);
        m_telemetry.addData("Intake speeds", "Left Intake: %.2f, Right Intake: %.2f",
                m_intakeLeft.get(), m_intakeRight.get());
    }

    public void stopIntake() {
        m_intakeLeft.set(0.0);
        m_intakeRight.set(0.0);
    }
}
