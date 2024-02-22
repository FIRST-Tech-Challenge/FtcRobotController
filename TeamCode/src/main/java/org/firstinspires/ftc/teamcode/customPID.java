package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
public class customPID {
    private double P;
    private double I;
    private double D;
    private final double KP;
    private final double KI;
    private final double KD;
    private boolean clamp;
    private final double maxValue;
    private final double minValue;
    private final double maxPosition;
    private double errorTotal;
    private double lastError;
    private double lastIntegral;
    private static final TelemetryPacket packet = new TelemetryPacket();
    private static final FtcDashboard dashboard = FtcDashboard.getInstance();

    public customPID(double KP, double KI, double KD, double maxValue, double minValue, double maxPosition){
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.minValue = minValue;
        this.maxValue = maxValue;
        this.maxPosition = maxPosition;
        this.errorTotal = 0;
        this.lastError = 0;
        this.lastIntegral = 0;
    }
    //lolijmionihbu uh
    private void proportionality(double error){
        this.P = this.KP * error;
    }
    private void integrator(double error){
        toClamp(error);
        if(!clamp){
            this.I = KI * (errorTotal + error);
            this.lastIntegral = KI * (errorTotal + error);
        }else{
            this.I = 0;
            this.lastIntegral = 0;
        }
    }
    private void derivative(double error){
        this.D = KD * (lastError - error);
    }
    private void toClamp(double error){
        if((Math.abs((lastError - error))<maxPosition * 0.004)&&(Math.abs(this.lastIntegral-this.I)>lastError*0.06)){
            clamp = true;
        }else{
            clamp = false;
        }
        clamp = (Math.abs((lastError - error)) < maxPosition * 0.004) && (Math.abs(this.lastIntegral - this.I) > lastError * 0.06);
    }
    public double outputPID(double error){
        proportionality(error);
        integrator(error);
        derivative(error);
        packet.put("P", this.P);
        packet.put("I", this.I);
        packet.put("D", this.D);
        double output =  this.P + this.I + this.D;
        this.lastError = error;
        this.errorTotal = errorTotal + error;
        if(output>(0.8*maxValue)){
            return 0.8 * maxValue;
        }else if(output<0.9 * minValue) {
            return -0.8 * minValue;
        }else{
            return output;
        }
    }
}