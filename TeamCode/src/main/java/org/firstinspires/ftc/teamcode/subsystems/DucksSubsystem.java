package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.commandftc.RobotUniversal.hardwareMap;

public class DucksSubsystem extends SubsystemBase {
    private final DcMotor m_motor;

    public DucksSubsystem() {
        m_motor = hardwareMap.dcMotor.get("DucksMotor");
        m_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void spin(double power) {
        m_motor.setPower(power);
    }

    public double getSpin() {
        return m_motor.getPower();
    }

    public int getCurrentPosition() {
        return m_motor.getCurrentPosition();
    }
}
