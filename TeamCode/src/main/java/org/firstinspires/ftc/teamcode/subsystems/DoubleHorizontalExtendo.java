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

        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setTargetPosition(0);
        extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendo.setPower(1);
    }

    public DoubleHorizontalExtendo(HardwareMap hw, String extendoName)
    {
        this.extendo = hw.get(DcMotor.class, extendoName);
    }

    public void setPower(double power)
    {
        extendo.setPower(power);
    }

    public void setPosition(double targetPosition){

        extendo.setTargetPosition((int)targetPosition);
        extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendo.setPower(0.7);

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
