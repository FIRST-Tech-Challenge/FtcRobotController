package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSwitch
{
    DigitalChannel colorSwitch;

    public ColorSwitch(HardwareMap hardwareMap)
    {
        colorSwitch = hardwareMap.get(DigitalChannel.class, "switch");
        colorSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    /**
     * Blue team = 1
     * Red team = -1
     */
    public int getTeam()
    {
        if (colorSwitch.getState())
        {
            return 1;
        }
        else
        {
            return -1;
        }
    }
}
