package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.UpliftRobot;
import org.firstinspires.ftc.teamcode.toolkit.core.Subsystem;
import org.firstinspires.ftc.teamcode.toolkit.misc.Utils;

public class ShooterSubsystem extends Subsystem {

    private UpliftRobot robot;
    public DcMotorEx shooter1;
    public DcMotorEx shooter2;

    public ShooterSubsystem(UpliftRobot robot){
        super(robot);
        this.robot = robot;
        this.shooter1 = robot.shooter1;
        this.shooter2 = robot.shooter2;
    }
    @Override
    public void enable() {

    }

    @Override
    public void disable() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void safeDisable() {
        setShooterPower(0);
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

    public void shooterFunctionTest() {
        setShooterPower(1);
        if(!robot.safeSleep(5000)) {
            safeDisable();
            return;
        }
        setShooterPower(0.5);
        if(!robot.safeSleep(5000)) {
            safeDisable();
            return;
        }
        setShooterPower(0);
    }

}
