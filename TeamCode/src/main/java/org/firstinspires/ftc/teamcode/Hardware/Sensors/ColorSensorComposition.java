package org.firstinspires.ftc.teamcode.Hardware.Sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

public class ColorSensorComposition implements ColorSensor {
    private ColorSensor colorSensor;
    public ColorSensorComposition(ColorSensor Colorsensor) {
        this.colorSensor = Colorsensor;
    }

    public ColorSensorComposition() {
    }

    @Override
    public int red() {
        return colorSensor.red();
    }

    @Override
    public int green() {
        return colorSensor.green();
    }

    @Override
    public int blue() {
        return colorSensor.blue();
    }

    @Override
    public int alpha() {
        return colorSensor.alpha();
    }

    @Override
    public int argb() {
        return colorSensor.argb();
    }

    @Override
    public void enableLed(boolean enable) {
        colorSensor.enableLed(enable);
    }

    @Override
    public void setI2cAddress(I2cAddr newAddress) {
        colorSensor.setI2cAddress(newAddress);
    }

    @Override
    public I2cAddr getI2cAddress() {
        return colorSensor.getI2cAddress();
    }

    @Override
    public Manufacturer getManufacturer() {
        return colorSensor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return colorSensor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return colorSensor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return colorSensor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        colorSensor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        colorSensor.close();
    }
}