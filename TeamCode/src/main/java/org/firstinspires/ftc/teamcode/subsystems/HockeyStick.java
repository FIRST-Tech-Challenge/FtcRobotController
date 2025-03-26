package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HockeyStick
{
    //FIGURE OUT LIMIT
    private final double LIMIT = 0;

    private double parallel;
    private double hover;
    private DcMotorEx hockeyStick;
    public HockeyStick(HardwareMap hardwareMap, String motorName)
    {
        hockeyStick = hardwareMap.get(DcMotorEx.class, motorName);
    }

    public void resetEncoders(){
        hockeyStick.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hockeyStick.setTargetPosition(0);
        hockeyStick.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
