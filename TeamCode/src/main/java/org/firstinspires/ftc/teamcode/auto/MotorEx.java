package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.DcMotorEx;

// a class for handling A DcMotorEx and it's simplified functions for it
public class MotorEx {
    public String motorname;
    public DcMotorEx motor;

    public MotorEx(String name){
        this.motorname = name;
    }
    //setup for when init hit in auto
    public void setMotor(DcMotorEx motor) {
        this.motor = motor;
    }

    public void move(int ticks) {
        int position = motor.getCurrentPosition() + ticks;
        motor.setTargetPosition(position);
    }
    public void stopMotor() {
        motor.setTargetPosition(motor.getCurrentPosition());
    }
    public DcMotorEx getMotor() {
        return motor;
    }
}
