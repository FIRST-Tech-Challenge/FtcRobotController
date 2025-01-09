package org.firstinspires.ftc.teamcode.TheRealKingIsHERE.drive.Motors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WristAxonNate {
    public Servo axon;
    public HardwareMap hardwareMap;
    public double position_;

    public void init(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        axon = hardwareMap.get(Servo.class, "wristServo");
        setPosition(0.42);
    }

    public void setPosition(double position){
        axon.setPosition(position);
        this.position_ = position;
    }

    public void moveLeft(double distance){
        distance = Math.abs(distance);
        if((position_ + distance) >= 1){
            this.position_ = 1;
            setPosition(1);
        }
        else {
            axon.setPosition(position_ + distance);
            this.position_ = position_ + distance;
        }
    }

    public void moveRight(double distance){
        distance = Math.abs(distance);
        if((position_ - distance) <= 0){
            this.position_ = 0;
            setPosition(0);
        }
        else {
            axon.setPosition(position_ - distance);
            this.position_ = position_ - distance;
        }
    }
}
