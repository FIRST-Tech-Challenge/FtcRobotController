package org.firstinspires.ftc.teamcode.ebotsenums;

import org.firstinspires.ftc.teamcode.ebotsenums.CsysDirection;
import org.firstinspires.ftc.teamcode.ebotsenums.MotionControlCoefficient;

public enum Speed {
    SLOW (0.2, 0.3, 0.2, 0.00, 0.0, 0.015, 0.0, 0.0),
    MEDIUM (0.60,0.4,  0.06, 0.00, 0.0, 0.015, 0.0, 0.0),  //iGain was 0.05
    FAST (1.0, 1.0, 0.05, 0.00, 0.0,0.010, 0.0,0.0),  // was 0.05 at start
    TELEOP (1.0, 1.0, 0.04, 0.0, 0.0,0.015, 0.0,0.0);  // was 0.05 at start

    /**  ENUM VARIABLES     **************/
    private double maxSpeed;
    private double turnSpeed;
    private double k_p;  //for translate proportional
    private double k_i;  //for translate integrator
    private double k_d;  //for translate derivative
    private double s_p;  //for spin proportional
    private double s_i;  //for spin integrator
    private double s_d;  //for spin derivative

    private final double measuredTranslateSpeed; // in / s
    private final double measuredAngularSpeedDeg;;  //  degrees / s

    /**  CONSTRUCTOR    **************/
    Speed(double speed, double turnSpeed, double pGain, double iGain, double dGain, double spinPGain, double spinIGain, double spinDGain){
        this.maxSpeed = speed;
        this.turnSpeed = turnSpeed;
        this.k_p = pGain;
        this.k_i = iGain;
        this.k_d = dGain;
        this.s_p = spinPGain;
        this.s_i = spinIGain;
        this.s_d = spinDGain;
        this.measuredTranslateSpeed = 45.26 * maxSpeed;
        this.measuredAngularSpeedDeg = 233.6 * turnSpeed;

    }
    /**  ENUM GETTERS AND SETTERS  ***********/
    public double getMaxSpeed(){return this.maxSpeed;}
    public double getTurnSpeed(){return this.turnSpeed;}
    public double getK_p(){return this.k_p;}
    public double getK_i(){return this.k_i;}
    public double getS_p(){return this.s_p;}
    public double getS_i(){return this.s_i;}

    public double getMeasuredTranslateSpeed() {
        return measuredTranslateSpeed;
    }

    public double getMeasuredAngularSpeedDeg() {
        return measuredAngularSpeedDeg;
    }
    public double getMeasuredAngularSpeedRad() {
        return Math.toRadians(measuredAngularSpeedDeg);
    }


    public void setK_i(double inputK_i){
        this.k_i = inputK_i;
    }
    public void setK_p(double inputK_p){
        this.k_p = inputK_p;
    }
    public void setS_p(double inputS_p){
        this.s_p = inputS_p;
    }
    public void setS_i(double inputS_i){
        this.s_i = inputS_i;
    }

    public double getCoefficient(MotionControlCoefficient coef, CsysDirection dir){
        double coefficient = 0;
        if(dir == CsysDirection.X | dir == CsysDirection.Y){
            coefficient = (coef == MotionControlCoefficient.P) ? this.k_p : this.k_i;
        } else if(dir == CsysDirection.Heading){
            coefficient = (coef == MotionControlCoefficient.P) ? this.s_p : this.s_i;
        }
        return  coefficient;
    }


    public boolean isIntegratorOn(CsysDirection csysDirection){
        boolean isIntegratorOn = false;
        if(csysDirection == CsysDirection.Heading){
            isIntegratorOn = this.s_i > 0;
        } else if (csysDirection == CsysDirection.X || csysDirection == CsysDirection.Y){
            isIntegratorOn = this.k_i > 0;
        }
        return isIntegratorOn;
    }


    @Override
    public String toString(){
        return "maxSpeed: " + maxSpeed + " , turnSpeed: " + turnSpeed
                + ", Translate Coefficients k_p / k_i / k_d: " + this.k_p + " / " + this.k_i + " / " + this.k_d
                + ", Spin Coefficients s_p / s_i / s_d: " + this.s_p + " / " + this.s_i + " / " + this.s_d;
    }
}

