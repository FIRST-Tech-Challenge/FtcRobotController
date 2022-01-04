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
        m_motor.setPower(0);
        m_motor.setTargetPosition(0);
        m_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPower(double power) {
        m_motor.setPower(power);
    }

    public boolean isBusy() {
        return m_motor.isBusy();
    }

    public double getPower() {
        return m_motor.getPower();
    }

    public void setTargetPosition(int position) {
        m_motor.setTargetPosition(position);
    }

    public int getCurrentPosition() {
        return m_motor.getCurrentPosition();
    }
}
