package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {

    Servo wrist;
    private double position;
    public Wrist(HardwareMap hw, String wristName){
        wrist = hw.get(Servo.class,wristName);
    }

    public void adjustPosition(double increment){
        position += 0.001 * increment;
        wrist.setPosition(position);
    }

    public double getPosition() {
        return position;
    }

    public void setParallel(){
        wrist.setPosition(0.5);
        position = 0.5;
    }
    public void initPos(){
        wrist.setPosition(0.5);
    }

    @SuppressLint("DefaultLocale")
    public String toString(){
        return String.format("Wrist Position: %.2f",position);
    }
}
