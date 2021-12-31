package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.commandftc.RobotUniversal.hardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotor m_intakeMotor;

    public IntakeSubsystem() {
        m_intakeMotor = hardwareMap.dcMotor.get("IntakeMotor");

        m_intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void intake(double power) {
        m_intakeMotor.setPower(power);
    }

    public double getIntakePower() {
        return m_intakeMotor.getPower();
    }

    public void stop() {
        m_intakeMotor.setPower(0);
    }
}
