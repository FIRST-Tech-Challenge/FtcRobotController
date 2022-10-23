package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cConfigureChannelCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HubPerformance {

    public HubPerformance() {};

    public static void enable(HardwareMap hardwareMap) throws InterruptedException{
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

            for (int i2cBus = 0; i2cBus < 4;i2cBus++) {
                LynxI2cConfigureChannelCommand cmd = new LynxI2cConfigureChannelCommand(module, i2cBus, LynxI2cConfigureChannelCommand.SpeedCode.FAST_400K);
                try {
                    cmd.send();
                } catch (LynxNackException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}
