package org.firstinspires.ftc.teamcode.Chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Chassis {
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    float slw;
    boolean btToggle= false;
    boolean smInput=false;

    public Chassis(HardwareMap hardwareMap){
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

    }

    public void slowMode(boolean bt){
        if(bt && !smInput){
            btToggle = !btToggle;
            smInput=true;
        }
        smInput=bt;

        if(btToggle){slw=0.5f;}else{slw=1.0f;}
    }

    public float[] mecanumDr(float lx, float ly, float rx, float ry){
        Boolean lsDeadzone;
        Boolean rsDeadzone;
        float[] power = new float[4];
        /*
        Deadzones | true = stick is near neutral position
        Protects from drift
        */
        lsDeadzone = (Math.abs(lx) < 0.1) &&
                (Math.abs(ly) < 0.1);
        rsDeadzone = (Math.abs(rx) < 0.1) &&
                (Math.abs(ry) < 0.1);

        //reset
        power[0]=0;
        power[1]=0;
        power[2]=0;
        power[3]=0;

        //translation
        if(!lsDeadzone){
            power[0]+=ly;
            power[1]+=ly;
            power[2]+=ly;
            power[3]+=ly;

            power[0]-=lx;
            power[1]+=lx;
            power[2]+=lx;
            power[3]-=lx;
        }

        //rotation
        if(!rsDeadzone){
            power[0]-=rx;
            power[1]+=rx;
            power[2]-=rx;
            power[3]+=rx;
        }

        //modifications
        power[0]*=slw;
        power[1]*=slw;
        power[2]*=slw;
        power[3]*=slw;

        return power;
    }
}