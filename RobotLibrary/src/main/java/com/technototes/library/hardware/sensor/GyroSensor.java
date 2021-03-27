package com.technototes.library.hardware.sensor;

public class GyroSensor extends Sensor<com.qualcomm.robotcore.hardware.GyroSensor> {

    public GyroSensor(com.qualcomm.robotcore.hardware.GyroSensor device) {
        super(device);
        device.calibrate();
    }
    public GyroSensor(String deviceName){
        super(deviceName);
        getDevice().calibrate();
    }

    @Override
    public double getSensorValue() {
        return getDevice().getHeading();
    }
}
