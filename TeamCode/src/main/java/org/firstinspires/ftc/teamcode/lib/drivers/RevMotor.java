package org.firstinspires.ftc.teamcode.lib.drivers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.openftc.revextensions2.ExpansionHubMotor;

public class RevMotor {
    private static final double MAX_POWER = 1d;
    private static final double MIN_DELTA_POWER = 0.005d;
    private              ExpansionHubMotor motor;
    private              boolean           onMasterHub;
    private              int               currentEncoderTicks;
    private              double            lastPower;
    private              double            encoderTicksPerRevolution;
    private              double            encoderTicksPerInch;
    private              double            velocity;
    private              double            externalGearRatio = 1d;

    public RevMotor(final ExpansionHubMotor motor, final boolean onMasterHub,
                            final boolean resetEncoder, final boolean brakeMode, final boolean reverse) {
        setMotor(motor);
        setOnMasterHub(onMasterHub);
        setCurrentEncoderTicks(0);
        setLastPower(0d);
        setZeroPowerBehavior(brakeMode ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        setVelocity(0d);
        if(resetEncoder) {
            resetEncoder();
        }

        if(reverse) {
            getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public RevMotor(final ExpansionHubMotor motor, final boolean onMasterHub,
                            final boolean resetEncoder, final boolean brakeMode, final boolean reverse,
                            final double encoderTicksPerRevolution) {
        this(motor, onMasterHub, resetEncoder, brakeMode, reverse);
        setEncoderTicksPerRevolution(encoderTicksPerRevolution * getExternalGearRatio());
    }

    public RevMotor(final ExpansionHubMotor motor, final boolean onMasterHub,
                            final boolean resetEncoder, final boolean brakeMode, final boolean reverse,
                            final double encoderTicksPerRevolution, final double rotationDiameter) {
        this(motor, onMasterHub, resetEncoder, brakeMode, reverse, encoderTicksPerRevolution);
        setEncoderTicksPerInch(getEncoderTicksPerRevolution() / (rotationDiameter * Math.PI));
    }

    public RevMotor(final ExpansionHubMotor motor, final boolean onMasterHub,
                    final boolean resetEncoder, final boolean brakeMode, final boolean reverse,
                    final double encoderTicksPerRevolution, final double rotationDiameter,
                    final double externalGearRatio) {
        this(motor, onMasterHub, resetEncoder, brakeMode, reverse, encoderTicksPerRevolution, rotationDiameter);
        setExternalGearRatio(externalGearRatio);
    }

    public void resetEncoder() {
        if(getMotor() != null) {
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setPower(double power) {
        power = Range.clip(power, -getMaxPower(), getMaxPower());
        if(Math.abs(power - getLastPower()) > getMinDeltaPower() || power == 0 && getLastPower() != 0) {
            getMotor().setPower(power);
            setLastPower(power);
        }
    }

    public double getPosition() {
        return getEncoderTicksPerInch() != 0d ? getCurrentEncoderTicks() / getEncoderTicksPerInch() :
                getEncoderTicksPerRevolution() != 0d ? getCurrentEncoderTicks() / getEncoderTicksPerRevolution() :
                        getCurrentEncoderTicks();
    }

    public double getVelocity() {
        return velocity;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public double getEncoderTicksPerRevolution() {
        return encoderTicksPerRevolution;
    }

    public void setEncoderTicksPerRevolution(double encoderTicksPerRevolution) {
        this.encoderTicksPerRevolution = encoderTicksPerRevolution;
    }

    public double getEncoderTicksPerInch() {
        return encoderTicksPerInch;
    }

    public void setEncoderTicksPerInch(double encoderTicksPerInch) {
        this.encoderTicksPerInch = encoderTicksPerInch;
    }

    public static double getMaxPower() {
        return MAX_POWER;
    }

    public void setEncoderReading(int position) {
        setCurrentEncoderTicks(position);
    }

    public void setEncoderReading(int position, double dt) {
        if(getEncoderTicksPerInch() != 0d) {
            setVelocity((position - getCurrentEncoderTicks()) / (dt * getEncoderTicksPerInch()));
        } else if(getEncoderTicksPerRevolution() != 0d) {
            setVelocity((position - getCurrentEncoderTicks()) / dt);
        }

        setEncoderReading(position);
    }

    public void setMode(DcMotor.RunMode runMode) {
        getMotor().setMode(runMode);
    }

    public DcMotor.RunMode getMode() {
        return getMotor().getMode();
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        getMotor().setDirection(direction);
    }

    public DcMotorSimple.Direction getDirection() {
        return getMotor().getDirection();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        getMotor().setZeroPowerBehavior(zeroPowerBehavior);
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return getMotor().getZeroPowerBehavior();
    }

    public void setTargetPosition(int position) {
        getMotor().setTargetPosition(position);
    }

    public int getTargetPosition() {
        return getMotor().getTargetPosition();
    }

    public ExpansionHubMotor getMotor() {
        return motor;
    }

    public void setMotor(ExpansionHubMotor motor) {
        this.motor = motor;
    }

    public boolean isOnMasterHub() {
        return onMasterHub;
    }

    public void setOnMasterHub(boolean onMasterHub) {
        this.onMasterHub = onMasterHub;
    }

    public int getCurrentEncoderTicks() {
        return currentEncoderTicks;
    }

    public void setCurrentEncoderTicks(int currentEncoderTicks) {
        this.currentEncoderTicks = currentEncoderTicks;
    }

    public double getLastPower() {
        return lastPower;
    }

    public void setLastPower(double lastPower) {
        this.lastPower = lastPower;
    }

    public static double getMinDeltaPower() {
        return MIN_DELTA_POWER;
    }

    public double getExternalGearRatio() {
        return externalGearRatio;
    }

    public void setExternalGearRatio(double externalGearRatio) {
        this.externalGearRatio = externalGearRatio;
    }
}
