package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Bucket {
    Servo servo;

    public Bucket(HardwareMap hw, String name)
    {
        servo = hw.get(Servo.class, name);
    }

    public void setPosition(double pos)
    {
        servo.setPosition(pos);
    }

    public double getPosition()
    {
        servo.getPosition();
    }
}
