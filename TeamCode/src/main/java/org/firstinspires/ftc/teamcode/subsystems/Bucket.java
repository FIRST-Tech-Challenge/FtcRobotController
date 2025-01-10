package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Bucket {
    Servo servo;
    boolean isUp;

    public Bucket(HardwareMap hw, String name)
    {
        servo = hw.get(Servo.class, name);
        isUp = false;
    }

    public void setPosition(double pos)
    {
        servo.setPosition(pos);
    }
    public void toggle() {
        if (isUp) {
           setBottom();
        } else {
            setUp();

        }
    }

    public void setUp()
    {
        servo.setPosition(0);
        isUp = true;
    }

    public void setBottom()
    {
        servo.setPosition(0.25);
        isUp = false;
    }

    public double getPosition()
    {
        return servo.getPosition();
    }
}
