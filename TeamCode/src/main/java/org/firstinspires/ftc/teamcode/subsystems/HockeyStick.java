package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HockeyStick
{
    //FIGURE OUT LIMIT
    private final double LIMIT = 0;
    private DcMotorEx hockeyStick;
    public HockeyStick(HardwareMap hardwareMap, String motorName)
    {
        hockeyStick = hardwareMap.get(DcMotorEx.class, motorName);
    }

    public void power(double power)
    {
        hockeyStick.setPower(power);
    }

    public double getPostion()
    {
        return hockeyStick.getCurrentPosition();
    }
}
