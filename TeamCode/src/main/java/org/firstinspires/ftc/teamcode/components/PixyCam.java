package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cSensor(name = "PixyCam", description = "PixyCam", xmlTag = "PixyCam")
public class PixyCam extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    /**
     * Block describes the signature, location, and size of a detected block.
     */
    public class Block
    {
        /**
         * A number from 1 through 7 corresponding to the color trained into the PixyCam,
         * or a sequence of octal digits corresponding to the multiple colors of a color code.
         */
        public final int signature;

        /**
         * The x, y location of the center of the detected block.
         * x is in the range (0, 255)
         * 0 is the left side of the field of view and 255 is the right.
         * y is in the range (0, 199)
         * 0 is the top of the field of view and 199 is the bottom.
         */
        public final int x, y;

        /**
         * The size of the detected block.
         * width or height of zero indicates no block detected.
         * maximum width is 255.
         * maximum height is 199.
         */
        public final int width, height;

        public Block(int signature, byte x, byte y, byte width, byte height)
        {
            this.signature = signature;
            this.x = TypeConversion.unsignedByteToInt(x);
            this.y = TypeConversion.unsignedByteToInt(y);
            this.width = TypeConversion.unsignedByteToInt(width);
            this.height = TypeConversion.unsignedByteToInt(height);
        }

        @Override
        public String toString()
        {
            return String.format("x: %d, y: %d, w: %d, h: %d", this.x, this.y, this.width, this.height);
        }
    }

    /**
     * The ReadWindow used to do a PixyCam LEGO protocol GeneralQuery
     */
    private I2cDeviceSynch.ReadWindow legoProtocolGeneralQueryReadWindow;
    /**
     * The ReadWindows used to do the PixyCam LEGO protocol SignatureQuery.
     */
    private I2cDeviceSynch.ReadWindow[] legoProtocolSignatureQueryReadWindows;

    public PixyCam(I2cDeviceSynch deviceSynch)
    {
        super(deviceSynch, true);

        this.legoProtocolGeneralQueryReadWindow = new I2cDeviceSynch.ReadWindow(0x50, 6, I2cDeviceSynch.ReadMode.REPEAT);
        this.legoProtocolSignatureQueryReadWindows = new I2cDeviceSynch.ReadWindow[7];
        for (int i = 1; i <= 7; i++)
            this.legoProtocolSignatureQueryReadWindows[i - 1] = NewLegoProtocolSignatureQueryReadWindow(i);

        super.registerArmingStateCallback(false);
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(1));
        this.deviceClient.engage();
    }

    private I2cDeviceSynch.ReadWindow NewLegoProtocolSignatureQueryReadWindow(int signature)
    {
        return new I2cDeviceSynch.ReadWindow(0x50 + signature, 5, I2cDeviceSynch.ReadMode.REPEAT);
    }

    private byte[] ReadEntireWindow(I2cDeviceSynch.ReadWindow readWindow)
    {
        this.deviceClient.setReadWindow(readWindow);
        return this.deviceClient.read(readWindow.getRegisterFirst(), readWindow.getRegisterCount());
    }

    /***
     *
     * @return a Block object containing details about the location of the largest detected block
     */
    public Block GetBiggestBlock()
    {
        byte[] buffer = ReadEntireWindow(this.legoProtocolGeneralQueryReadWindow);

        int signature = buffer[1] << 8 | buffer[0];

        return new Block(signature, buffer[2], buffer[3], buffer[4], buffer[5]);
    }

    /**
     * @param signature is a scale between 1 and 7 corresponding to the signature trained into the PixyCam.
     * @return a Block object containing details about the location of the largest detected block for the specified signature.
     */
    public Block GetBiggestBlock(int signature)
    {
        if (signature < 1 || signature > 7)
            throw new IllegalArgumentException("signature must be between 1 and 7");

        byte[] buffer = ReadEntireWindow(this.legoProtocolSignatureQueryReadWindows[signature - 1]);

        return new Block(signature, buffer[1], buffer[2], buffer[3], buffer[4]);
    }


    @Override
    protected boolean doInitialize()
    {
        return true;
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName()
    {
        return "PixyCam";
    }
}