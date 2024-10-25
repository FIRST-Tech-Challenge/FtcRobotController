package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo claw;
    private String clawName;
    private final double open=0.8;
    private final double close=0.2;
    public Claw(HardwareMap hardwareMap){
        this.clawName="claw";
        this.claw=hardwareMap.get(Servo.class,clawName);
    }
    public Claw(HardwareMap hardwareMap, String clawName){
        this.clawName=clawName;
        this.claw=hardwareMap.get(Servo.class,clawName);
    }
    public void open(){
        claw.setPosition(open);
    }
    public void close(){
        claw.setPosition(close);
    }
    public void setPosition(double position){
        claw.setPosition(position);
    }
}
