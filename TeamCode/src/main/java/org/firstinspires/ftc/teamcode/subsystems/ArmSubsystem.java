package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.commandftc.RobotUniversal;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.commandftc.RobotUniversal.hardwareMap;

import org.firstinspires.ftc.teamcode.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
	private final DcMotor m_motor;
	private final Servo m_leftServo;
	private final Servo m_rightServo;

	public ArmSubsystem() {
		m_motor = hardwareMap.dcMotor.get("ArmMotor");
		m_leftServo = hardwareMap.servo.get("LeftIntakeServo");
		m_rightServo = hardwareMap.servo.get("RightIntakeServo");
		// TODO: check direction (spin clockwise on positive)
		m_motor.setDirection(DcMotorSimple.Direction.FORWARD);
		m_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		m_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		m_rightServo.setDirection(Servo.Direction.REVERSE);
	}
	public int getDirection() {
		if(m_motor.getPower() > 0) return 1; // 1 == positive
		else if(m_motor.getPower() < 0) return -1; // -1 == negative
		else return  0;// 0 == neutral
	}

	public void setPower(double power) {
		m_motor.setPower(power);
	}

	public double getPower() {
		return m_motor.getPower();
	}

	public double getAngle() {
		return (double)m_motor.getCurrentPosition() / ArmConstants.motorGear / ArmConstants.gear;
	}

	public void setVerticalPosition(double position) {
		m_leftServo.setPosition(position);
		m_rightServo.setPosition(position);
	}

	public double getVerticalPosition() {
		return m_leftServo.getPosition();
	}

	public double SyncError() {
		return Math.abs(m_leftServo.getPosition() - m_rightServo.getPosition());
	}

	public int getTargetPosition() {
		return m_motor.getTargetPosition();
	}

	public int getCurrentPosition() {
		return m_motor.getCurrentPosition();
	}
}
