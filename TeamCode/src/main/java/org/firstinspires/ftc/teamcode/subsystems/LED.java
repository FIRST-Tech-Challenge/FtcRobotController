package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LED {
    private RevBlinkinLedDriver led;

    public LED(HardwareMap hw, String name)
    {
      led = hw.get(RevBlinkinLedDriver.class, name);
    }

    public void setRed()
    {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    public void setBlue()
    {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    public void setYellow()
    {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    }

    public void setDefault()
    {
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
    }
}
