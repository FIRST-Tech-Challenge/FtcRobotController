package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxFirmwareVersionManager;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

public class LynxOptimizedI2cFactory {

    public static I2cDeviceSynch createLynxI2cDeviceSynch(LynxModule module, int bus) {
        return new BetterI2cDeviceSynchImplOnSimple(
                LynxFirmwareVersionManager.createLynxI2cDeviceSynch(AppUtil.getDefContext(), module, bus), true);
    }

}
