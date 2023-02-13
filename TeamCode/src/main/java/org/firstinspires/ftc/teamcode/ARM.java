package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ARM {
    public DcMotor Motor5;
    public DcMotor Motor6;
    public DcMotor Motor7;
    public void stop(){
        Motor5.setPower(0);
        Motor6.setPower(0);
    }
    public void armMove(float power){
        Motor5.setPower(power*0.7);
        Motor6.setPower(power*0.7);
    }
    public void armExtend(float power){
        Motor7.setPower(power*0.7);
    }

}