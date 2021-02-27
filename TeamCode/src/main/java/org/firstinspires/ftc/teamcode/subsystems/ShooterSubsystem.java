package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
