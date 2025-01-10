package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

public class Claw {
    private final Servo servo;
    private boolean isOpen;
    public Claw(HardwareMap hw){
        servo = hw.get(Servo.class, "claw");
        isOpen = false;
    }
//    public void open(){
//        servo.
//    }
    public void close(){
        servo.setPosition(1);
        isOpen = false;
    }
    public void release(){

        servo.setPosition(0);
        isOpen = true;
    }

    public void disableClaw()
    {
        servo.setPosition(1);
    }

    public void enableClaw()
    {
        servo.setPosition(1);
        isOpen = true;
    }
    public double getPosition()
    {
        return servo.getPosition();
    }
    public void toggle() {
        if (isOpen) {
            close();
        } else {
            release();

        }
    }

    public void setPostion(double pos)
    {
        servo.setPosition(pos);
    }

    public boolean getIsOpen()
    {
        return isOpen;
    }
//    public void close()
//    {
//        servo.setPower(-0.7);
//        isOpen = false;
//    }
//
//    public void release()
//    {
//        servo.setPower(0.7);
//        isOpen = true;
//    }

}
