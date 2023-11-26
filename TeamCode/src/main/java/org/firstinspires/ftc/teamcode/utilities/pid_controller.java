package org.firstinspires.ftc.teamcode.utilities;
import com.sun.tools.javac.tree.JCTree;

public class pid_controller {

    private double Kp;
    private double Ki;
    private double Kd;

    private double Tau;

    private double limMin;
    private double limMax;

    public double sampleTime;

    //Controller Memory
    private double integrator;
    private double previousError;
    private double differentiator;
    private double prevMeasurement;

    public double out;

    void init_pid(double P, double I, double D){
        Kp=P;
        Ki=I;
        Kd=D;
        Tau = 0;

//        limMin = min_pos;
//        limMax = max_pose;


        integrator = 0;
        differentiator = 0;
        previousError = 0;
        prevMeasurement = 0;

        out = 0;
    }

    double update(double setPoint,double measurment, double minCmd, double maxCmd, double sampleTime){
        limMin = minCmd;
        limMax = maxCmd;
        double error= setPoint - measurment;

        //Prop gain
        double proportional = Kp*error;

        //Integral term
        integrator = integrator + 0.5*Ki*sampleTime*(error+previousError);

        double limMinInt, limMaxInt;
        if (limMax > proportional){
            limMaxInt = limMax - proportional;
        }else{
            limMaxInt = 0;
        }

        if (limMin < proportional){
            limMinInt = limMin - proportional;
        }else{
            limMinInt = 0;
        }

        //Clamp integrator
        if (integrator > limMaxInt){
            integrator =  limMaxInt;
        }else if (integrator < limMinInt){
            integrator = limMinInt;
        }

        differentiator = (2.0 * Kd*(measurment - prevMeasurement) + (2.0 * Tau - sampleTime)*differentiator) / (2.0*Tau + sampleTime);

        out = proportional + integrator + differentiator;

        // compare output and limit
        if (out > limMax){
            out = limMax;
        }else if (out < limMin){
            out = limMin;
        }

        //store error and measurement
        previousError =error;
        prevMeasurement = measurment;

        return out;
    }

}