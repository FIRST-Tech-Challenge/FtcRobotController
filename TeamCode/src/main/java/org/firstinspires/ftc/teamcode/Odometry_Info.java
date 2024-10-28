package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Odometry_Info {

    DcMotorEx odoRight = null;
    DcMotorEx odoLeft = null;
    DcMotorEx odoBack = null;

    /*static variables for odometry sensor stats*/
    static double odoTPM = 2000;
    static double C = 2*Math.PI*16*(1/odoTPM);
    static double L = 172;
    static double B = 146;

    /* Variables to notate the current positions of the robot*/
    double curX = 0.0;
    double curY = 0.0;
    double cur0 = 0.0;

    /*Target Position Variables*/
    double tarX = 0.0;
    double tarY = 0.0;
    double tar0 = 0.0;

    /* Distance Variables for the odo in MM. 1 = right, 2 = left, 3 = back */
    double Cn1 = 0.0;
    double Cn2 = 0.0;
    double Cn3 = 0.0;

    Odometry_Info(HardwareMap hwMap){
        odoRight = hwMap.get(DcMotorEx.class, "Odometry_Pod_Right");
        odoLeft = hwMap.get(DcMotorEx.class, "Odometry_Pod_Left");
        odoBack = hwMap.get(DcMotorEx.class, "Odometry_Pod_Back");
    }

    /* this function should be placed in the loop section*/
    public void updateCurPos(){

        Cn1 = odoRight.getCurrentPosition();
        Cn2 = odoLeft.getCurrentPosition();
        Cn3 = odoBack.getCurrentPosition();

        curX += C*(Cn1+Cn2/L);
        cur0 += C*(Cn2-Cn1/L);
        curY += C*(Cn3-B*(Cn2-Cn1/L));
    }

    public void setTargetPos(double x, double y, double O){
        tarX = x;
        tarY = y;
        tar0 = O;
    }

    public boolean curPosIsTarPos() {
        if (curX != tarX && curY != tarY && cur0 != tar0) {
            return true;
        }
        else{
            return false;
        }
    }

}
