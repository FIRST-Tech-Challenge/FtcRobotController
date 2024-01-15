package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@SuppressWarnings({"WeakerAccess", "unused"}) // Ignore access and unused warnings

@I2cDeviceType
@DeviceProperties(name = "MB1242", description = "Ultrasonic distance sensor made by maxbotix", xmlTag = "MB1242")

public class MB1242Ultrasonic extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    
    private final static byte RANGE_COMMAND = 81;
    private final static byte CHANGE_ADDRESS_1 = (byte) 170;
    private final static byte CHANGE_ADDRESS_2 = (byte) 165;
    public int range;
    
    public void range() {
        writeByte((byte)81);
    }
    
    public int centimeters() {
        return readShort(Register.READ_DISTANCE);
    }
    
    public double inches() {
        return (double) centimeters() / 2.54;
    }
    
    public short changeAddress(short address) {
        
        if (address % 2 != 0) address --;
        
        if (address == 0 | address == 80 | address == 164 | address == 170) {
            return 0;
        }
        
        writeByteArray(Register.FIRST, new byte[]{CHANGE_ADDRESS_1, CHANGE_ADDRESS_2, (byte)address});
        
        currentAddress = I2cAddr.create7bit(address);
        deviceClient.setI2cAddress(currentAddress);
        
        return address;
    }
    
    protected void writeByte(byte value)
    {
        deviceClient.waitForWriteCompletions(I2cWaitControl.WRITTEN);
        deviceClient.write(TypeConversion.shortToByteArray(value));
    }
    
    protected void writeByteArray(final Register reg, byte[] value)
    {
        deviceClient.write(reg.bVal, value);
    }
    
    protected short readShort(Register reg)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }
    
    ///////////////////////////// Config ///////////////////////////////
    
    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(112);
    public I2cAddr currentAddress;
    
    public MB1242Ultrasonic(I2cDeviceSynch deviceClient) {
        
        super(deviceClient, true);
    
        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }
    
    protected void setOptimalReadWindow()
    {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.ONLY_ONCE);
        this.deviceClient.setReadWindow(readWindow);
    }
    
    
    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }
    
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }
    
    @Override
    public String getDeviceName() {
        return "Max Botix Ultrasonic Distance sensor MB1242";
    }
    
    public enum Register {
        FIRST(0),
        READ_DISTANCE(0),
        LAST(1),
        RANGE_COMMAND(81);
        
        public int bVal;
        
        Register(int bVal) {
            
            this.bVal = bVal;
        }
    }
}

