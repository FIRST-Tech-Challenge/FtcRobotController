package org.firstinspires.ftc.teamcode.auto.servo;

import com.qualcomm.robotcore.hardware.Servo;


// basic servo with no set positions at all
public class Servob {
    public final String servoName;
    protected Servo servo;

    public Servob(String servoName){
        this.servoName = servoName;

    }


    public void setServo(Servo servo){
        this.servo = servo;
    }

    public Servo getServo(){
        return servo;
    }

    public double getPos(){
        return servo.getPosition();
    }


}
