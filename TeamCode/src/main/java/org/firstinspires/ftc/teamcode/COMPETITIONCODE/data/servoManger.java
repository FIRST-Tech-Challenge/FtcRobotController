package org.firstinspires.ftc.teamcode.COMPETITIONCODE.data;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servoManger {
    private Servo servo;

    public void init(HardwareMap hwMap,String ServoName) {
        servo = hwMap.get(Servo.class, ServoName);
    }
    public void setServoPosition(double position){
        servo.setPosition(position);
    }
    public double getServoPosition(){return servo.getPosition();}
}