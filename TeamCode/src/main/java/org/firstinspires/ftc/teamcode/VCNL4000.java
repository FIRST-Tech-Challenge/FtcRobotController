package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cDeviceType
@DeviceProperties(name = "VCNL4000 Proximity Sensor", xmlTag = "VCNL4000")
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

        return "Adafruit VCNL4000 Proximity Sensor";
    }

    public enum Register
    {
        PRODUCT_ID(0x81),
        NO_FUNCTION(0x82),
        P_LED_CURRENT_SET(0x83),
        AMBIENT_LIGHT_PARAMETER(0x84),
        AMBIENT_LIGHT_RESULT_HIGH(0x85),
        AMBIENT_LIGHT_RESULT_LOW(0x86),
        PROXIMITY_MEASUREMENT_HIGH(0x87),
        PROXIMITY_MEASUREMENT_LOW(0x88),
        PROXIMITY_MEASUREMENT_SIGNAL_FREQUENCY(0x89);

        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }
    }
    protected void setOptimalReadWindow()
    {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.PRODUCT_ID.bVal,
                Register.PROXIMITY_MEASUREMENT_SIGNAL_FREQUENCY.bVal - Register.PRODUCT_ID.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    public VCNL4000(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned)
    {
        super(deviceClient, deviceClientIsOwned);

        this.setOptimalReadWindow();

        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged
        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
    }
    protected void writeLong(final Register reg, short value)
    {
        deviceClient.write(reg.bVal, TypeConversion.longToByteArray(value));
    }

    protected long readLong()
    {
        return TypeConversion.byteArrayToLong(deviceClient.read(Register.PRODUCT_ID.bVal, 8));
    }

    public long getProductID() {
        return readLong();
    }
}