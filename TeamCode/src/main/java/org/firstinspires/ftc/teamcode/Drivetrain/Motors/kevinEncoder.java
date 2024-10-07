package org.firstinspires.ftc.teamcode.Drivetrain.Motors;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class kevinEncoder extends DcMotorExComposition{
    private DcMotorEx encoder;
    private int pos = 0;
    public kevinEncoder(DcMotorEx encoder) {
        this.encoder = encoder;
    }


    public void Encoder(DcMotorEx encoder){
        this.encoder = encoder;
        return;
    }
    public void updateEncoder(){
        pos = encoder.getCurrentPosition();
    }


    public void resetEncoder(){
        pos = 0;
    }
    public int getCurrentPosition(){
        return pos;
    }
    public double getVelocity(){
        return encoder.getVelocity();
    }



}
