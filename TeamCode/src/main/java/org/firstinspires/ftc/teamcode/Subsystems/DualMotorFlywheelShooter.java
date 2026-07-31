package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Config.FlywheelShooterConfig;
import org.firstinspires.ftc.teamcode.Config.HardwareMapConfig;

public class DualMotorFlywheelShooter extends FlywheelShooter {

    private final DcMotorEx leftShooterMotor;
    private final DcMotorEx rightShooterMotor;

    public DualMotorFlywheelShooter(HardwareMap hardwareMap) {
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, HardwareMapConfig.left_shooter_motor_id);
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, HardwareMapConfig.right_shooter_motor_id);
        leftShooterMotor.setDirection(FlywheelShooterConfig.LEFT_FLYWHEEL_MOTOR_DIRECTION);
        rightShooterMotor.setDirection(FlywheelShooterConfig.RIGHT_FLYWHEEL_MOTOR_DIRECTION);
    }

    @Override
    void applyPower(double power) {
        leftShooterMotor.setPower(power);
        rightShooterMotor.setPower(power);
    }

    @Override
    double getVelocity() {
        return leftShooterMotor.getVelocity() + rightShooterMotor.getVelocity() / 2;
    }
}
