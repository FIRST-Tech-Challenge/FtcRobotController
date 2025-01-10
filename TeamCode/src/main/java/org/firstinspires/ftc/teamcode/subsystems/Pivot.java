package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

public class Pivot {
    private DcMotorEx pivot;
    private Encoder encoder;
    private double count;

    public Pivot(HardwareMap hw, String name, String encoderName)
    {
        pivot = hw.get(DcMotorEx.class, name);
        encoder = new OverflowEncoder(new RawEncoder(hw.get(DcMotorEx.class, encoderName)));
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pivot.setTargetPosition(0);

        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pivot.setPower(1);

        count = 0;
    }
    public Pivot(HardwareMap hw, String name)
    {
        pivot = hw.get(DcMotorEx.class, name);

    }
    public void setPower(double pos)
    {
        pivot.setPower(pos);
    }

    public void setPosition(int pos)
    {
        pivot.setTargetPosition(pos);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot.setPower(1);
    }

    public void setZero()
    {
        setPosition(100);


    }

    public void setMid()
    {
        setPosition(200);
    }

    public void setFull() throws InterruptedException {
        setPosition(400);
        Thread.sleep(1000);
        setPosition(525);

    }

    public void toggle() throws InterruptedException {
        if(count == 0)
        {
            setMid();
            count++;
        }else if(count ==1)
        {
            setFull();
            count++;
        }else if(count == 2){
            setMid();
            count++;
        }else {
            setZero();
            count = 0;
        }
    }

    public void goBack()
    {
            setZero();
            count = 0;

    }
    public double getPosition()
    {
        return encoder.getPositionAndVelocity().position;
    }


}
