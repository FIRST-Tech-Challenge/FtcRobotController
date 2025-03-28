package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {

    Servo wrist;
    private double position;
    private double SPEED_MULI = 0.001;
    private double MAX = 1;
    private double MIN = 0;

    public Wrist(HardwareMap hw, String wristName){
        wrist = hw.get(Servo.class,wristName);
    }

    public void adjustPosition(double increment){
        position += SPEED_MULI * increment;
        position = Math.max(Math.min(position, MAX), MIN);
        wrist.setPosition(position);
    }

    public double getPosition() {
        return position;
    }

    //Not the actual parallel position by the way!
    public void setParallel(){
        position = 0.5;
        wrist.setPosition(0.5);
    }
    public void setPosition(double pos){
        position = pos;
        wrist.setPosition(position);
    }
    public void initPos(){
        wrist.setPosition(0.5);
    }

    @SuppressLint("DefaultLocale")
    public String toString(){
        return String.format("Wrist Position: %.2f",position);
    }
}
