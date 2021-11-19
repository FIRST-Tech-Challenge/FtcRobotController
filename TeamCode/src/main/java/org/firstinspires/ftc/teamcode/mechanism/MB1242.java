package org.firstinspires.ftc.teamcode.mechanism;

import android.icu.util.TimeUnit;

import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@I2cDeviceType
@DeviceProperties(name = "Ultrasonic Sensor", description = "Ultrasonic Distance Sensor from MaxBotix", xmlTag = "MB1242", compatibleControlSystems = ControlSystem.REV_HUB)
public class MB1242 extends I2cDeviceSynchDevice<I2cDeviceSynch> implements DistanceSensor {
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    @Override
    protected boolean doInitialize() {
        timer.reset();
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "MB1242 MaxBotix Ultrasonic Distance Sensor";
    }

    public MB1242(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
        this.deviceClient.setI2cAddress(I2cAddr.create8bit(0xe1));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    public void ping() {
        deviceClient.write(0xE0, TypeConversion.intToByteArray(0x51));
    }

    /**
     * Allow 100ms between pinging.
     */
    public short readRange(){
        return TypeConversion.byteArrayToShort(deviceClient.read(0xE1, 2));
    }

    public double getDistance(DistanceUnit unit){
        return unit.fromCm(readRange());
    }
}
