package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@I2cDeviceType
@DeviceProperties(name = "IR Proximity Sensor", xmlTag = "VCNL4000")
public class VCNL4000 extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    @Override
    public Manufacturer getManufacturer()
    {

        return Manufacturer.Adafruit;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public String getDeviceName()
    {

        return "Adafruit MCP9808 Temperature Sensor";
    }

    public VCNL4000(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned)
    {
        super(deviceClient, deviceClientIsOwned);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }
    public enum Register
    {
        COMMAND(0x01),
        PRODUCT_ID_REVISION(0x02),
        T_L(0x03),
        T_LIMIT_CRITICAL(0x04),
        TEMPERATURE(0x05),
        MANUFACTURER_ID(0x06),
        DEVICE_ID_REVISION(0x07),
        RESOLUTION(0x08);

        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }
    }
}