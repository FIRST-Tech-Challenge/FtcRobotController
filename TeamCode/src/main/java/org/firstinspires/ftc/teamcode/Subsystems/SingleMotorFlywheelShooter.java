package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Config.FlywheelShooterConfig;
import org.firstinspires.ftc.teamcode.Config.HardwareMapConfig;

public class SingleMotorFlywheelShooter extends FlywheelShooter {

    private final DcMotorEx shooterMotor;

    public SingleMotorFlywheelShooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, HardwareMapConfig.shooter_motor_id);
        shooterMotor.setDirection(FlywheelShooterConfig.FLYWHEEL_MOTOR_DIRECTION);
    }

    @Override
    void applyPower(double power) {
        shooterMotor.setPower(power);
    }

    @Override
    double getVelocity() {
        return shooterMotor.getVelocity();
    }
}
