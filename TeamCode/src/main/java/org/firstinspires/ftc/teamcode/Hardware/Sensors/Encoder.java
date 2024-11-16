package org.firstinspires.ftc.teamcode.Hardware.Sensors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Encoder {
    private DcMotorEx encoder;
    private int storedPos = 0;
    public Encoder(DcMotorEx encoder) {
        this.encoder = encoder;
        this.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void encoder(DcMotorEx encoder){
        this.encoder = encoder;
    }
    public void updateEncoder(){
        storedPos = encoder.getCurrentPosition();
    }


    public void resetEncoder(){
        storedPos = encoder.getCurrentPosition();
    }
    public int getCurrentPosition(){
        updateEncoder();
        return getCurrentPosition() - storedPos;
    }
    public double getVelocity(){
        return encoder.getVelocity();
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        encoder.setDirection(direction);
    }

}
