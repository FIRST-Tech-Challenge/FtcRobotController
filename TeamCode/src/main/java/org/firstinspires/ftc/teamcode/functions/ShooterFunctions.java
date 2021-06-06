package org.firstinspires.ftc.teamcode.functions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.UpliftRobot;

public class ShooterFunctions {

    UpliftRobot robot;
    DcMotorEx shooter1;
    DcMotorEx shooter2;

    public ShooterFunctions(UpliftRobot robot){
        this.robot = robot;
        this.shooter1 = robot.shooter1;
        this.shooter2 = robot.shooter2;
    }

    public void setShooterPower(double power) {
        shooter1.setPower(power);
        shooter2.setPower(power);
    }

    public void setShooterPIDF(double kP, double kI, double kD, double kF) {
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));
    }

    public void setShooterVelocity(double targetVelocity) {
        shooter1.setVelocity(targetVelocity);
        shooter2.setVelocity(targetVelocity);
    }
}
