package org.firstinspires.ftc.teamcode.Hardware.Actuators;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoAdvanced {
    private double lastPos = 0;
    Servo servo = null;
    private double servoTolerance = 0.001;

    public ServoAdvanced(Servo servo) {
        this.servo = servo;
    }


    public void setPos(double newPos){
        if(Math.abs(newPos - lastPos) > servoTolerance){
            servo.setPosition(newPos);
            lastPos = newPos;
        }
    }



    public double getPos(){
        return lastPos;
    }
    public double getTolerance(){
        return servoTolerance;
    }
    public void setTolerance() {
        servoTolerance = 0.001;
    }
}

