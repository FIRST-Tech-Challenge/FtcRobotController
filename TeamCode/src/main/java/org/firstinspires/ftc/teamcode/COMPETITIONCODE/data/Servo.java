package org.firstinspires.ftc.teamcode.COMPETITIONCODE.data;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Servo {
    private Servo servo;

    public void init(HardwareMap hwMap,String ServoName) {
        servo = hwMap.get(Servo.class, "servo");
    }
    public void setServoPosition(double position){
        servo.setServoPosition(position);
    }
}