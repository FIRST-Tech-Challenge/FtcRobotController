package org.firstinspires.ftc.teamcode.Mechanisms.Utils.Planners;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MotionProfile {
    double desiredD;
    double deltaTAccel;
    double deltaTDecel;
    double dAccel;
    double dDecel;
    double dCruise;
    double deltaTCruise;
    double deltaT;
    public double vMax;
    double aAccel;
    double aDecel;
    int reverse;
    public MotionProfile(double desiredD, double vMax, double aAccel, double aDecel, boolean reverse){
        double deltaTAccel = vMax/aAccel;
        double deltaTDecel = vMax/aDecel;
        double dAccel = aAccel*Math.pow(deltaTAccel,2)/2;
        double dDecel = aDecel*Math.pow(deltaTDecel,2)/2;
        if (desiredD<dAccel+dDecel) {
            deltaTDecel = Math.sqrt((2*aAccel*desiredD)/(Math.pow(aDecel,2)+aAccel*aDecel));
            deltaTAccel = (aDecel/aAccel)*deltaTDecel;
            dAccel = aAccel*Math.pow(deltaTAccel,2)/2;
            dDecel = aDecel*Math.pow(deltaTDecel,2)/2;
            vMax = aAccel * deltaTAccel;
        }
        double dCruise = desiredD-dAccel-dDecel;
        double deltaTCruise = dCruise/vMax;
        double deltaT = deltaTAccel+deltaTCruise+deltaTDecel;
        this.desiredD = desiredD;
        this.vMax = vMax;
        this.dAccel = dAccel;
        this.dDecel = dDecel;
        this.aAccel = aAccel;
        this.aDecel = aDecel;
        this.dCruise = dCruise;
        this.deltaTCruise = deltaTCruise;
        this.deltaT = deltaT;
        this.deltaTAccel = deltaTAccel;
        this.deltaTDecel = deltaTDecel;
        if (reverse){
            this.reverse=-1;
        } else {
            this.reverse=1;
        }

    }
    public double getPos(double t){
        double tCruise = t-deltaTAccel;
        double tDecel = t-deltaTAccel-deltaTCruise;
        if (t > deltaT){
            return desiredD*reverse;
        } else if (t <= deltaTAccel) {
            return aAccel*Math.pow(t,2)/2*reverse;
        } else if (deltaTAccel<t&& t<=deltaTAccel+deltaTCruise){
            return dAccel+vMax*tCruise*reverse;
        } else if (deltaTAccel+deltaTCruise<t&&t<=deltaT){
            return (dAccel+dCruise+vMax*tDecel-aDecel*Math.pow(tDecel,2)/2)*reverse;
        }
        return -1;
    }
    public double getVelocity(double t){
        if (t>deltaT){
            return 0;
        } else if (t<deltaTAccel){
            return aAccel*t*reverse;
        } else if (deltaTAccel<t&&t<=deltaTAccel+deltaTCruise){
            return vMax*reverse;
        } else if (deltaTAccel+deltaTCruise<t&&t<=deltaT){
            double tDecel = t-deltaTAccel-deltaTCruise;
            return (vMax-aDecel*tDecel)*reverse;
        }
        return 10000;
    }
    public double getAcceleration(double t){
        if (t>deltaT){
            return 0;
        } else if (t<=deltaTAccel){
            return aAccel*reverse;
        } else if (deltaTAccel<t&&t<=deltaTAccel+deltaTCruise){
            return 0;
        } else if (deltaTAccel+deltaTCruise<t&&t<=deltaT){
            return -aDecel*reverse;
        }
        return -1;
    }
    public double getTime(){
        return deltaT;
    }
    public int isReverse(){
        return reverse;
    }
    }
