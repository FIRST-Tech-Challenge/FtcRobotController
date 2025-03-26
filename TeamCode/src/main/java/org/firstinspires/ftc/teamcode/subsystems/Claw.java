package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo claw;
    HardwareMap hardwareMap;
    private final double OPEN = 0;
    private final double CLOSE = 0.7;
    /**
        @param: (String) name of the claw
     */
    public Claw(String clawName)
    {
        claw = hardwareMap.get(Servo.class, clawName);
    }

    public void open()
    {
        claw.setPosition(OPEN);
    }

    public void close()
    {
        claw.setPosition(CLOSE);
    }

    public double getPos()
    {
        return claw.getPosition();
    }

    public void toggle()
    {
        if(claw.getPosition() == OPEN)
        {
            close();
        }else {
            open();
        }
    }

}
