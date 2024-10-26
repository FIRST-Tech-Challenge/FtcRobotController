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
//    public Wrist(HardwareMap hardwareMap, String wristName){
//        this.wristName=wristName;
//        this.wrist=hardwareMap.get(Servo.class,wristName);
//    }

    public double lastPosition = 0;
    public void setPosition(double position){
        if(position!=lastPosition){
            wrist.setPosition(position);
        }
        lastPosition = position;
    }

    public void intake(){
        setPosition(.3);
    }
    public void deposit(){
        setPosition(.95);
    }

}
