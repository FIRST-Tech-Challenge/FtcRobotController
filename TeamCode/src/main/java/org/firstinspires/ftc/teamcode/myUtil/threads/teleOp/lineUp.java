package org.firstinspires.ftc.teamcode.myUtil.threads.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.myUtil.MecanumHardAuto;

public class lineUp extends Thread{

    MecanumHardAuto r;
    OpMode opMode;
    public lineUp(OpMode opMode, MecanumHardAuto r){
        this.opMode = opMode;
        this.r = r;

    }

    public void run(){
        r.moveInches(0.6,-12);
        r.sensorRotate(0.6,-Math.PI/2);
        r.sensorRotate(0.6,-Math.PI/2);
        r.moveInches(0.6,36);
    }

}
