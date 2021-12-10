package org.firstinspires.ftc.teamcode.competition.utils.eyesquaredsea;

import androidx.annotation.RequiresPermission;

import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cSensor(name = "Ultrasonic Distance Sensor", description = "Ultrasonic Distance Sensor", xmlTag = "TOFSENSOR")
public class EyeSquaredSeaDistanceSensor extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    protected void writeData(final Commands cmd, short value) {
        deviceClient.write(cmd.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readData(final Commands cmd, short value) {
        return TypeConversion.byteArrayToShort(deviceClient.read(cmd.bVal, cmd.returnedValues));
    }

    public EyeSquaredSeaDistanceSensor(I2cDeviceSynch i2cDeviceSynch) {
        super(i2cDeviceSynch, true);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
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
        return "Distance Sensor (Probably Ultrasonic)";
    }

    public enum Commands
    {
        INITIATE_WRITE(224, 0, ReadWrite.WRITE),
        WRITE_RANGE_COMMAND(81, 0, ReadWrite.WRITE),
        READ_LAST(225, 2, ReadWrite.READ);

        public int bVal;
        public int returnedValues;
        public ReadWrite readWrite;

        Commands(int bVal, int returnedValues, ReadWrite readWrite) {
            this.bVal = bVal;
            this.returnedValues = returnedValues;
            this.readWrite = readWrite;
        }
    }

    public enum ReadWrite { READ, WRITE }
}
