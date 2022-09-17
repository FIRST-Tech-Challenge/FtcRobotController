package org.firstinspires.ftc.forteaching;

import com.qualcomm.robotcore.hardware.Servo;

public class BasicServoCode {
    private final Servo servo; // Since there's only one Servo so just name it as is

    public BasicServoCode(Servo servo) {
        this.servo = servo;
    }

    public void setPosition(double position){
        // Range from 0.0 to 1.0
        // The only to keep this method because the Servo constant is private
        this.servo.setPosition(position);
    }

    public void toLeft(){
        this.setPosition(0.0);
    }

    public void toRight(){
        this.setPosition(1.0);
    }

    public void toMiddle(){
        this.setPosition(0.5);
    }

    public double get(){
        /// Not really useful since technically we didn't extending the Subsystem class
        return this.servo.getPosition();
    }
}
