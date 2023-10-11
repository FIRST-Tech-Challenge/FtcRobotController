package org.firstinspires.ftc.team417_PowerPlay;

import com.qualcomm.robotcore.hardware.DigitalChannel;

public class I2CToSPIConversion {

    private byte address;

    private DigitalChannel resetPin;

    public I2CToSPIConversion(DigitalChannel resetPin, boolean a0, boolean a1, boolean a2) {
        this.resetPin = resetPin;
        this.address = (byte) (0B0101000 | ((byte) (a2 ? 1 : 0) << (byte) 2) | ((byte) (a1 ? 1 : 0) << (byte) 1) | ((byte) (a0 ? 1 : 0)));
    }

    void reset() {
        resetPin.setMode(DigitalChannel.Mode.OUTPUT);
        resetPin.setState(true);
        try {
            Thread.sleep(1);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        resetPin.setState(false);
        try {
            Thread.sleep(1);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        resetPin.setState(true);
    }


}

