package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class YellowJacket19_2 extends Motor {

    private DcMotor motor;
    private double resetValue;

    public static final double TICKS_PER_REV = 537.6;

    public YellowJacket19_2(HardwareMap hMap, String motorName) {
        motor = hMap.get(DcMotor.class, motorName);
    }

    // @Override
    public void setPower(double speed) {
        motor.setPower(speed);
    }

    // @Override
    public double getPower() {
        return motor.getPower();
    }

    // @Override
    public void setDirection(boolean isInverted) {
        if(isInverted) {
            motor.setDirection(DcMotor.Direction.FORWARD);
        } else {
            motor.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    // @Override
    public boolean getDirection() {
        return motor.getDirection() ==DcMotor.Direction.REVERSE;
    }

    // @Override
    public void disable() {
        motor.close();
    }

    // @Override
    public String getType() {
        return this.getClass().toString();
    }

    // @Override
    public void setMotor(double output) {
        set(output);
    }

    // @Override
    public void stopMotor() {
        set(0);
    }

    public double getEncoder() {
        return motor.getCurrentPosition() - resetValue;
    }

    public void resetEncoder() {
        resetValue = motor.getCurrentPosition();
    }

    public double getRevolutions() {
        return getEncoder() / TICKS_PER_REV;
    }
}
