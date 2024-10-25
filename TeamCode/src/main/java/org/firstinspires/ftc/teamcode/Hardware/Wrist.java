package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    private Servo wrist;
    private String wristName;
    public Wrist(HardwareMap hardwareMap){
        this.wristName="wrist";
        this.wrist=hardwareMap.get(Servo.class,wristName);
    }
    public Wrist(HardwareMap hardwareMap, String wristName){
        this.wristName=wristName;
        this.wrist=hardwareMap.get(Servo.class,wristName);
    }
    public void setPosition(double position){
        wrist.setPosition(position);
    }
}
