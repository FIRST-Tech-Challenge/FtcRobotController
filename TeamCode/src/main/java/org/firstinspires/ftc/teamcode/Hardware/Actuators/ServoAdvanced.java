package org.firstinspires.ftc.teamcode.Hardware.Actuators;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoAdvanced {
    private double lastPos = 0;
    private Servo servo;
    private double servoTolerance = 0.001;

    public ServoAdvanced(Servo servo) {
        this.servo = servo;
    }


    public void setPosition(double newPos){
        if(Math.abs(newPos - lastPos) > servoTolerance){
            servo.setPosition(newPos);
            lastPos = newPos;
        }
    }



    public double getPosition(){
        return lastPos;
    }
    public double getTolerance(){
        return servoTolerance;
    }
    public void setTolerance() {
        servoTolerance = 0.001;
    }
}

