package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class CachingDcMotorEx implements DcMotorEx, Robot.Listener {

    private DcMotorEx delegate;
    private double cachedPower = 0;
    private double cachedVelocity = 0;
    private boolean needsPowerUpdate = false;
    private boolean needsVelocityUpdate = false;
    private AngleUnit angleUnit = AngleUnit.RADIANS;
    public CachingDcMotorEx(DcMotorEx delegate) {
        this.delegate = delegate;
    }

    @Override
    public void setMotorEnable() {
        delegate.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        delegate.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return delegate.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate) {
        this.setVelocity(angularRate, AngleUnit.RADIANS);
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        if (angularRate != cachedVelocity) {
            needsVelocityUpdate = true;
            cachedVelocity = angularRate;
            this.angleUnit = unit;
        }
    }

    @Override
    public double getVelocity() {
        return delegate.getVelocity();
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return delegate.getVelocity(unit);
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        delegate.setPIDCoefficients(mode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        delegate.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        delegate.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        delegate.setPositionPIDFCoefficients(p);
    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return delegate.getPIDCoefficients(mode);
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return delegate.getPIDFCoefficients(mode);
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        delegate.setTargetPositionTolerance(tolerance);
    }

    @Override
    public int getTargetPositionTolerance() {
        return delegate.getTargetPositionTolerance();
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return delegate.getCurrent(unit);
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return delegate.getCurrentAlert(unit);
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        delegate.setCurrentAlert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
        return delegate.isOverCurrent();
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return delegate.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        delegate.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return delegate.getController();
    }

    @Override
    public int getPortNumber() {
        return delegate.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        delegate.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return delegate.getZeroPowerBehavior();
    }

    @Override
    public void setPowerFloat() {
        delegate.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return delegate.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        delegate.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return delegate.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return delegate.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return delegate.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        delegate.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return delegate.getMode();
    }

    @Override
    public void setDirection(Direction direction) {
        delegate.setDirection(Direction.REVERSE);
    }

    @Override
    public Direction getDirection() {
        return delegate.getDirection();
    }

    @Override
    public void setPower(double power) {
        if (power != cachedPower) {
            cachedPower = power;
            needsPowerUpdate = true;
        }
    }

    @Override
    public double getPower() {
        return cachedPower;
    }

    @Override
    public Manufacturer getManufacturer() {
        return delegate.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return delegate.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return delegate.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return delegate.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        delegate.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        delegate.close();
    }

    @Override
    public void update() {
        if (needsPowerUpdate) delegate.setPower(cachedPower);
        if (needsVelocityUpdate) delegate.setVelocity(cachedVelocity, angleUnit);
    }
}
