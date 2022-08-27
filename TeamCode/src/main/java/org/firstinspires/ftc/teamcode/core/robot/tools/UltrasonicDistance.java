package org.firstinspires.ftc.teamcode.core.robot.tools;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.teamcode.core.thread.old.EventThread;
import androidx.annotation.NonNull;

public class UltrasonicDistance {
    private final Thread updateThread;
    private double curMM = 0;
    private final I2cDeviceSynch RANGE1Reader;
    public UltrasonicDistance(@NonNull HardwareMap hardwareMap, EventThread eventThread) {
        RANGE1Reader = new I2cDeviceSynchImpl(
                hardwareMap.get(I2cDevice.class, "range"),
                new I2cAddr(0x14),
                true
        );
        RANGE1Reader.engage();
        updateThread = new Thread(() -> {
            while (!eventThread.isInterrupted()) {
                this.update();
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });
    }

    /**
     * does not need to be run by user, can be to force
     */
    public void update() {
        byte[] range1Cache = RANGE1Reader.read(0x04, 2);
        curMM = (range1Cache[0] & 0xFF) * 10;
    }

    public void init() {
        updateThread.setPriority(5);
        updateThread.start();
    }

    public double mm() {
        return curMM;
    }
}
