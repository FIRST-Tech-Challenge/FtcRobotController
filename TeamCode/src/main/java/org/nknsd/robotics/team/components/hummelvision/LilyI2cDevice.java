package org.nknsd.robotics.team.components.hummelvision;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;


@I2cDeviceType
@DeviceProperties(
        name = "Lily Vision",
        xmlTag = "LilyVision",
        description = "Hummel Brand Vision Sensor"
)
public class LilyI2cDevice extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    public static final int DEFAULT_ADDRESS = 0x13;
    public LilyI2cDevice(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);

        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                0, 12, I2cDeviceSynch.ReadMode.REPEAT);

        deviceClient.setReadWindow(readWindow);

        deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_ADDRESS));
        super.registerArmingStateCallback(false);
        deviceClient.engage();
    }
    @Override
    protected boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Lily's I2c Vision Sensor";
    }

    public int[] getData(){
        byte[] data = deviceClient.read(0,12);
        int[] output = new int[12];
        for (int i = 0 ; i < 12 ; i++)
            output[i] = data[i] & 0xff; // convert the weird signed java bytes to numbers
        return output;
    }

}
