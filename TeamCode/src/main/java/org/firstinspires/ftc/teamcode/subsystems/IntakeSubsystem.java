package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

/** M1 - the intake roller that pulls balls into the robot. */
public class IntakeSubsystem extends SubsystemBase {

    private final DcMotorEx motor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, RobotConstants.INTAKE_NAME);
        motor.setDirection(RobotConstants.INTAKE_DIRECTION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Pulls balls in. */
    public void intake() {
        motor.setPower(RobotConstants.INTAKE_POWER);
    }

    /** Spits balls back out - the same speed, reversed. */
    public void outtake() {
        motor.setPower(-RobotConstants.INTAKE_POWER);
    }

    public void stop() {
        motor.setPower(0);
    }

    public double getPower() {
        return motor.getPower();
    }
}
