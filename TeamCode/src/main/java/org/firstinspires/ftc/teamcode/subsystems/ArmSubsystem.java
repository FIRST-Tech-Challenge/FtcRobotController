package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.commandftc.RobotUniversal.hardwareMap;

public class ArmSubsystem extends SubsystemBase {
	private final DcMotor m_motor;
	
	public ArmSubsystem() {
		m_motor = hardwareMap.dcMotor.get("ArmMotor");
		// TODO: check direction (spin clockwise on positive)
		m_motor.setDirection(DcMotorSimple.Direction.FORWARD);
		m_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		m_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	public void setPower(double power) {
		m_motor.setPower(power);
	}

	public double getPower() {
		return m_motor.getPower();
	}
}
