package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Base64;

public class DoubleHorizontalExtendo {
    private DcMotor extendo;
    private Encoder encoder;

    public DoubleHorizontalExtendo(HardwareMap hw, String extendoName, String encoderName)
    {
        this.extendo = hw.get(DcMotor.class, extendoName);
        this.encoder = new OverflowEncoder(new RawEncoder(hw.get(DcMotorEx.class, encoderName)));
    }

    public void setPower(double power)
    {
        extendo.setPower(power);
    }

    public double getPower()
    {
        return extendo.getPower();
    }

    public double getPosition()
    {
        return encoder.getPositionAndVelocity().position;
    }


}
