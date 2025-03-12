package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo claw;
    HardwareMap hardwareMap;
    private final double OPEN = 0;
    private final double CLOSE = 0.7;
    public Claw(String clawName)
    {
        claw = hardwareMap.get(Servo.class, clawName);
    }

    public void Open()
    {
        claw.setPosition(OPEN);
    }

    public void Close()
    {
        claw.setPosition(CLOSE);
    }
}
