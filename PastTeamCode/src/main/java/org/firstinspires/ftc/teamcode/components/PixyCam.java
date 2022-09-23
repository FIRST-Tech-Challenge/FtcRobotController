package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import java.util.*;

@I2cDeviceType()
@DeviceProperties(name = "PixyCam", description = "PixyCam", xmlTag = "PixyCam")
public class PixyCam extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    private static final int BLOCK_SIZE = 14;
    private int blockCount;

    public PixyCam(I2cDeviceSynch deviceClient){
        super(deviceClient, true);
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(0x54));
        this.setReadWindow();
        this.deviceClient.engage();
    }

    private void setReadWindow()
    {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                0,
                1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
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
        return "PixyCam";
    }

    public void setBlockCount(int count){
        blockCount = count;
    }

    public ArrayList<Block> getBlocks() {

        ArrayList<Block> blocks = new ArrayList<>(blockCount);

        // We don't know where in the register the frame will start, so we'll need to
        // gather enough bytes to make sure we can read <blockCount> concurrent blocks.
        // In order to do this, we'll read <(blockCount + 1)*BLOCK_SIZE> bytes, plus two
        // to account for the sync words.
        byte[] bytes = new byte[(blockCount+1)*BLOCK_SIZE+2];
        int length = bytes.length;
        // read data from pixy
        bytes = this.getDeviceClient().read(0, length);

        // search for sync
        for (int i = 0; i < length; i++){

            // We're too late in the register. Return what we have (if anything),
            // or we'll get an ArrayIndexOutOfBoundsException.
            if (i + 14 >= length) {
                return blocks;
            }

            boolean sync1 = checkSync(bytes,i);
            boolean sync2 = checkSync(bytes,i+2);
            boolean startOfFrame = sync1 && sync2;

            if (startOfFrame){

                // Advance two to skip the extra sync word
                i += 2;
                for (int j = 0; j < blockCount; j++) {
                    byte[] tempBytes = new byte[BLOCK_SIZE];
                    System.arraycopy(bytes, i, tempBytes, 0, BLOCK_SIZE);
                    i += BLOCK_SIZE;
                    Block block = getBlock(tempBytes);
                    if (block != null) {
                        blocks.add(block);
                    }
                }
                return blocks;

            }
        }

        // We didn't find anything; just return an empty ArrayList;
        return blocks;

    }

    private boolean checkSync(byte[] bytes, int byteOffset){

        int b1 = bytes[byteOffset];
        if (b1 < 0) {
            b1 += 256;
        }
        int b2 = bytes[byteOffset + 1];
        if (b2 < 0) {
            b2 += 256;
        }

        return b1 == 0x55 && b2 == 0xaa;

    }

    private Block getBlock(byte[] bytes){

        Block block = new Block();

        int checksum = convertBytesToInt(bytes[3], bytes[2]);
        // if the checksum is 0 or the checksum is a sync byte, then there
        // are no more blocks.
        if (checksum == 0 || checksum == 0xaa55)
        {
            // return an empty block
            return block;
        }
        block.signature = convertBytesToInt(bytes[5], bytes[4]);
        block.xCenter = convertBytesToInt(bytes[7], bytes[6]);
        block.yCenter = convertBytesToInt(bytes[9], bytes[8]);
        block.width = convertBytesToInt(bytes[11], bytes[10]);
        block.height = convertBytesToInt(bytes[13], bytes[12]);

        return block;
    }

    public class Block{
        public int signature;
        public int xCenter;
        public int yCenter;
        public int width;
        public int height;
    }

    private int convertBytesToInt(int msb, int lsb){
        if (msb < 0) {
            msb += 256;
        }

        int value = msb * 256;

        if (lsb < 0) {
            value += 256;
        }
        value += lsb;
        return value;
    }
}