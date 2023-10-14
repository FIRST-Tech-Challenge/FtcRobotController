package org.firstinspires.ftc.teamcode.lib.drivers;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class RevCRServo {
    private static final double MAX_POWER = 1d;
    private static final double MIN_DELTA_POWER = 0.005d;
    private CRServo crServo;
    private double lastPower;

    public RevCRServo(CRServo crServo) {
        setCrServo(crServo);
        setLastPower(0d);
        getCrServo().setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) {
        power = Range.clip(power, -getMaxPower(), getMaxPower());
        if(Math.abs(power - getLastPower()) > getMinDeltaPower() || power == 0 && getLastPower() != 0) {
            getCrServo().setPower(power);
            setLastPower(power);
        }
    }

    public CRServo getCrServo() {
        return crServo;
    }

    public void setCrServo(CRServo crServo) {
        this.crServo = crServo;
    }

    public double getLastPower() {
        return lastPower;
    }

    public void setLastPower(double lastPower) {
        this.lastPower = lastPower;
    }

    public static double getMaxPower() {
        return MAX_POWER;
    }

    public static double getMinDeltaPower() {
        return MIN_DELTA_POWER;
    }
}
