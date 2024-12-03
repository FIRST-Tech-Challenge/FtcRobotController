package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Odometry_Info {

    DcMotorEx odoRight = null;
    DcMotorEx odoLeft = null;
    DcMotorEx odoBack = null;

    /*static variables for odometry sensor stats*/
    static double odoTPM = 2000.0;
    static double C = 2*Math.PI*16;
    static double L = 190.504;
    static double B = 42.382;

    /* Variables to notate the current positions of the robot*/
    double Xc = 0.0;
    double Xp = 0.0;
    double Theta0 = 0.0;

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

        odoRight.setDirection(DcMotorEx.Direction.REVERSE);
        odoLeft.setDirection(DcMotorEx.Direction.REVERSE);
    }

    /* this function should be placed in the loop section*/
    public void updateCurPos(){

        Cn1 = C*(odoRight.getCurrentPosition()/odoTPM);
        Cn2 = C*(odoLeft.getCurrentPosition()/odoTPM);
        Cn3 = C*(odoBack.getCurrentPosition()/odoTPM);

        Xc = ((Cn1+Cn2)/2);
        Theta0 = cur0;
        cur0 = ((Cn1-Cn2)/L);
        Xp = (Cn3 - (B*cur0));

        curX = (Xc*Math.cos(Theta0) - Xp*Math.sin(Theta0));
        curY = (Xc*Math.sin(Theta0) + Xp*Math.cos(Theta0));
    }

    /* Tool Functions */

    public void setTargetPos(double x, double y, double O){
        tarX = x;
        tarY = y;
        tar0 = O;
    }

    public boolean curPosIsTarPos() {
        if (curX != tarX && curY != tarY && cur0 != tar0) {
            return false;
        }
        else{
            return true;
        }
    }

    public void resetEncoders(){
        odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        odoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
