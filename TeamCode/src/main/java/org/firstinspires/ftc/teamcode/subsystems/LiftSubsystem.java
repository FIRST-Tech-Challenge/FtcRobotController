package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants.LiftConstants;

import static org.commandftc.RobotUniversal.hardwareMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {
    private final DcMotor m_liftMotor;

        public LiftSubsystem() {
        m_liftMotor = hardwareMap.dcMotor.get("LiftMotor");
        m_liftMotor.setPower(0);
        m_liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_liftMotor.setTargetPosition(0);
        m_liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public int getCurrentPosition() {
        return m_liftMotor.getCurrentPosition();
    }
    public int getTargetPosition() {
        return m_liftMotor.getTargetPosition();
    }

    public double getHeight() {
        return LiftConstants.distance_per_revolution * m_liftMotor.getCurrentPosition();
    }

    public void setPower(double power) {
        m_liftMotor.setPower(power);
    }

    public double getPower() {
        return m_liftMotor.getPower();
    }

    public void stop() {
        m_liftMotor.setPower(0);
    }

    public boolean isBusy() {
        return m_liftMotor.isBusy();
    }
}
