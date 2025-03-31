package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HockeyStick
{
    //FIGURE OUT LIMIT
    private final int TOP = -750;
    private final int BOTTOM = -1000;

    private double parallel;
    private double hover;
    private DcMotorEx hockeyStick;
    public HockeyStick(HardwareMap hardwareMap, String motorName)
    {
        hockeyStick = hardwareMap.get(DcMotorEx.class, motorName);

        hockeyStick.setTargetPosition(hockeyStick.getCurrentPosition());
        hockeyStick.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hockeyStick.setPower(0);


    }


    public void resetEncoders(){
        hockeyStick.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hockeyStick.setTargetPosition(0);
        hockeyStick.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public int getPosition()
    {
        return hockeyStick.getCurrentPosition();
    }

    public void adjustPos(double val)
    {
        int pos = getPosition() + (int) (val * 100);
        hockeyStick.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hockeyStick.setTargetPosition(pos);
        hockeyStick.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setDown()
    {
        hockeyStick.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hockeyStick.setTargetPosition(BOTTOM);
        hockeyStick.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setUP()
    {
        hockeyStick.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hockeyStick.setTargetPosition(TOP);
        hockeyStick.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void toggle()
    {
        int TOGGLE = (getPosition() == TOP) ?  BOTTOM: TOP;
        hockeyStick.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hockeyStick.setTargetPosition(TOGGLE);
        hockeyStick.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
