package org.darbots.darbotsftclib.libcore.integratedfunctions.pid_control;

public class PIDCoefficients {
    public double Kp;
    public double Ki;
    public double Kd;
    public PIDCoefficients(){
        this.Kp = 0;
        this.Ki = 0;
        this.Kd = 0;
    }
    public PIDCoefficients(double Kp, double Ki, double Kd){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }
    public PIDCoefficients(PIDCoefficients oldPIDCoefficients){
        this.Kp = oldPIDCoefficients.Kp;
        this.Ki = oldPIDCoefficients.Ki;
        this.Kd = oldPIDCoefficients.Kd;
    }
    public PIDCoefficients(com.qualcomm.robotcore.hardware.PIDCoefficients FTCPidCoefficients){
        this.Kp = FTCPidCoefficients.p;
        this.Ki = FTCPidCoefficients.i;
        this.Kd = FTCPidCoefficients.d;
    }
    public com.qualcomm.robotcore.hardware.PIDCoefficients toFTCPIDCoefficients(){
        return new com.qualcomm.robotcore.hardware.PIDCoefficients(this.Kp,this.Ki,this.Kd);
    }
}
