package org.firstinspires.ftc.teamcode.Usefuls.Motor;

import org.firstinspires.ftc.teamcode.Usefuls.Math.M;
import org.firstinspires.ftc.teamcode.Usefuls.Timer;

public class AnglePID {
    //todo: specialize for angles (Done but not tested)
    private AnglePID.ErrorFunction errorFunction;
    private AnglePID.ResponseFunction responseFunction;
    private AnglePID.Coefficients coefficients;
    private Timer timer;
    private double e = 0.0;
    private double et = 0.0;
    private double deDt = 0.0;

    public static class Coefficients {
        public double kp = 0.0, ki = 0.0, kd = 0.0, deDtGain = 0.0, etMax = 1e+99;

        public Coefficients(double kp, double ki, double kd) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
        }

        public Coefficients(double kp, double ki, double kd, double deDtGain, double etMax) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
            this.deDtGain = deDtGain;
            this.etMax = etMax;
        }
    }

    public static interface ErrorFunction { public double execute(); }
    public static interface ResponseFunction { public void execute(double factor); }

    public AnglePID(AnglePID.Coefficients coefficients, AnglePID.ErrorFunction errorFunction, AnglePID.ResponseFunction responseFunction) {
        this.errorFunction = errorFunction;
        this.responseFunction = responseFunction;
        this.coefficients = coefficients;
        this.timer = new Timer();
    }

    public void update() {
        this.timer.update();

        double e = this.errorFunction.execute();
        double dt = this.timer.getDt();

        double deDt = M.lerp((e - this.e) / dt, this.deDt, this.coefficients.deDtGain);

        double det = e * dt;
        if(Math.signum(det) != Math.signum(this.et)) {this.et = 0.0;}
        double et = M.clamp(this.et + det, -this.coefficients.etMax, this.coefficients.etMax);

        double factor = 0.0;
        factor += this.coefficients.kp * e;
        factor += this.coefficients.ki * et; //Absement/Absition (Integral of d/t)
        factor += this.coefficients.kd * deDt; //Velocity (Derivative of d/t)

        this.e = e;
        this.et = et;
        this.deDt = deDt;

        this.responseFunction.execute(factor);
    }
}

//package org.firstinspires.ftc.teamcode.usefuls.Motor;
//
//import org.firstinspires.ftc.teamcode.usefuls.Math.M;
//import org.firstinspires.ftc.teamcode.usefuls.Timer;
//
//public class AnglePID {
//    //todo: specialize for angles (Done but not tested)
//    private AnglePID.ErrorFunction errorFunction;
//    private AnglePID.ResponseFunction responseFunction;
//    private AnglePID.Coefficients coefficients;
//    private Timer timer;
//    private double e = 0.0;
//    private double et = 0.0;
//    private double deDt = 0.0;
//
//    private int count = 0;
//    private double currentAngle;
//    private double lastAngle;
//
//    public static class Coefficients {
//        public double kp = 0.0, ki = 0.0, kd = 0.0, deDtGain = 0.0, etMax = 1e+99;
//
//        public Coefficients(double kp, double ki, double kd) {
//            this.kp = kp;
//            this.ki = ki;
//            this.kd = kd;
//        }
//
//        public Coefficients(double kp, double ki, double kd, double deDtGain, double etMax) {
//            this.kp = kp;
//            this.ki = ki;
//            this.kd = kd;
//            this.deDtGain = deDtGain;
//            this.etMax = etMax;
//        }
//    }
//
//    public static interface ErrorFunction { public double execute(); }
//    public static interface ResponseFunction { public void execute(double factor); }
//
//    public AnglePID(AnglePID.Coefficients coefficients, AnglePID.ErrorFunction errorFunction, AnglePID.ResponseFunction responseFunction) {
//        this.errorFunction = errorFunction;
//        this.responseFunction = responseFunction;
//        this.coefficients = coefficients;
//        this.timer = new Timer();
//    }
//
//    public void update() {
//        this.timer.update();
//
//        double e = this.updateAngle();
//        double dt = this.timer.getDt();
//
//        double deDt = M.lerp((e - this.e) / dt, this.deDt, this.coefficients.deDtGain);
//
//        double det = e * dt;
//        if(Math.signum(det) != Math.signum(this.et)) {this.et = 0.0;}
//        double et = M.clamp(this.et + det, -this.coefficients.etMax, this.coefficients.etMax);
//
//        double factor = 0.0;
//        factor += this.coefficients.kp * e;
//        factor += this.coefficients.ki * et; //Absement/Absition (Integral of d/t)
//        factor += this.coefficients.kd * deDt; //Velocity (Derivative of d/t)
//
//        this.e = e;
//        this.et = et;
//        this.deDt = deDt;
//
//        this.responseFunction.execute(factor);
//    }
//    private double updateAngle(){
//        this.currentAngle = this.errorFunction.execute();
//        if(Math.abs(this.currentAngle - this.lastAngle) > 180){
//            this.count += Math.signum(this.lastAngle - this.currentAngle);
//        }
//        this.lastAngle = this.currentAngle;
//        return this.count * 360 + this.currentAngle;
//    }
//}
//This does not work for some reason