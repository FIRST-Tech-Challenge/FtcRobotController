package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ParkFlag {

    private Servo parkFlag;

    private double stowPosition = 0;
    private double raisedPosition = 0.35;

    public ParkFlag(HardwareMap hw){
        this(hw, "parkFlag");
    }

    public ParkFlag(HardwareMap hw, String name){
        parkFlag = hw.get(Servo.class, name);
    }

    public void stowFlag(){
        parkFlag.setPosition(stowPosition);
    }

    public void raiseFlag(){
        parkFlag.setPosition(raisedPosition);
    }
}
