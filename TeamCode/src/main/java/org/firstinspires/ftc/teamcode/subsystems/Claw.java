package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo claw;
    private double open = 0.222;
    private double close = 0.568;
    private double ring = 0.418;
    private double position;
    /**
        @param: (String) name of the claw
     */
    public Claw(HardwareMap hardwareMap,String clawName)
    {
        claw = hardwareMap.get(Servo.class, clawName);
    }

    public void open()
    {
        position=open;
//        claw.setPosition(open);
    }

    public void close()
    {
        position=close;
//        claw.setPosition(close);
    }

    public void toggle()
    {
         position = (position == open) ? close: open;
    }

    public double getPos()
    {
        return claw.getPosition();
    }

    public void ringClose()
    {
        position=ring;
//        claw.setPosition(ring);
    }
    public void adjustPosition(double increment){
        position += increment*0.0001;
        claw.setPosition(position);
    }
}
