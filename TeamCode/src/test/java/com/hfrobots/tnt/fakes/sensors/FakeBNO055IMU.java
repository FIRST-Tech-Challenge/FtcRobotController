package com.hfrobots.tnt.fakes.sensors;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class FakeBNO055IMU implements BNO055IMU, HardwareDevice {
    private Parameters parameters;

    @Override
    public void write(Register register, byte[] data) {

    }

    @Override
    public boolean initialize(Parameters parameters) {
        this.parameters = parameters;

        return true;
    }

    @Override
    public Parameters getParameters() {
        return parameters;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }

    @Override
    public Orientation getAngularOrientation() {
        return null;
    }

    @Override
    public Orientation getAngularOrientation(AxesReference reference, AxesOrder order, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit) {
        return null;
    }

    @Override
    public Acceleration getOverallAcceleration() {
        return null;
    }

    @Override
    public AngularVelocity getAngularVelocity() {
        return null;
    }

    @Override
    public Acceleration getLinearAcceleration() {
        return null;
    }

    @Override
    public Acceleration getGravity() {
        return null;
    }

    @Override
    public Temperature getTemperature() {
        return null;
    }

    @Override
    public MagneticFlux getMagneticFieldStrength() {
        return null;
    }

    @Override
    public Quaternion getQuaternionOrientation() {
        return null;
    }

    @Override
    public Position getPosition() {
        return null;
    }

    @Override
    public Velocity getVelocity() {
        return null;
    }

    @Override
    public Acceleration getAcceleration() {
        return null;
    }

    @Override
    public void startAccelerationIntegration(Position initialPosition, Velocity initialVelocity, int msPollInterval) {

    }

    @Override
    public void stopAccelerationIntegration() {

    }

    @Override
    public SystemStatus getSystemStatus() {
        return null;
    }

    @Override
    public SystemError getSystemError() {
        return null;
    }

    @Override
    public CalibrationStatus getCalibrationStatus() {
        return null;
    }

    @Override
    public boolean isSystemCalibrated() {
        return false;
    }

    @Override
    public boolean isGyroCalibrated() {
        return false;
    }

    @Override
    public boolean isAccelerometerCalibrated() {
        return false;
    }

    @Override
    public boolean isMagnetometerCalibrated() {
        return false;
    }

    @Override
    public CalibrationData readCalibrationData() {
        return null;
    }

    @Override
    public void writeCalibrationData(CalibrationData data) {

    }

    @Override
    public byte read8(Register register) {
        return 0;
    }

    @Override
    public byte[] read(Register register, int cb) {
        return new byte[0];
    }

    @Override
    public void write8(Register register, int bVal) {

    }
}
