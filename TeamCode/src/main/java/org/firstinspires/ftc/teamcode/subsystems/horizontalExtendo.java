package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class horizontalExtendo {
    private Servo extendo;

    public horizontalExtendo(HardwareMap hw, String extendoName)
    {
        extendo = hw.get(Servo.class, extendoName);
    }

    public void setPosition(double pos)
    {
        extendo.setPosition(getPosition() + pos);
    }

    public double getPosition()
    {
        return extendo.getPosition();
    }


}
