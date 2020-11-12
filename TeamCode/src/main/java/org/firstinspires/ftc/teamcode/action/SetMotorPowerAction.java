package org.firstinspires.ftc.teamcode.action;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.playmaker.Action;
import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;

public class SetMotorPowerAction implements Action {

    String motorName;
    double power;
    public SetMotorPowerAction(String motorName, double power) {
        this.motorName = motorName;
        this.power = power;
    }


    @Override
    public void init(RobotHardware hardware) {

    }

    @Override
    public boolean doAction(RobotHardware hardware) {
        DcMotor motor = hardware.hardwareMap.get(DcMotor.class, motorName);
        if (motor != null) {
            motor.setPower(power);
        }
        return true;
    }

    @Override
    public Double progress() {
        return null;
    }

    @Override
    public String progressString() {
        return null;
    }
}
