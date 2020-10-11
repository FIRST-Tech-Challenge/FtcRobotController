package org.darbots.darbotsftclib.libcore.integratedfunctions.pid_control;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDIntegrator {
    public static class PIDErrorInfo{
        public double Error;
        public double Integral;
        public double Derivative;
        public PIDErrorInfo(){
            this.Error = 0;
            this.Integral = 0;
            this.Derivative = 0;
        }
        public PIDErrorInfo(double Error, double Integral, double Derivative){
            this.Error = Error;
            this.Integral = Integral;
            this.Derivative = Derivative;
        }
        public PIDErrorInfo(PIDErrorInfo Info){
            this.Error = Info.Error;
            this.Integral = Info.Integral;
            this.Derivative = Info.Derivative;
        }
    }
    double m_IntegratedError;
    double m_DerivedError;
    double m_LastError;
    ElapsedTime m_ErrorTime;
    public PIDIntegrator(){
        this.m_IntegratedError = 0;
        this.m_DerivedError = 0;
        this.m_LastError = 0;
        this.m_ErrorTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }
    public PIDIntegrator(PIDIntegrator oldCalculator){
        this.m_IntegratedError = oldCalculator.m_IntegratedError;
        this.m_DerivedError = 0;
        this.m_LastError = oldCalculator.m_LastError;
        this.m_ErrorTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }
    public void feedError(double error){
        double deltaTime = this.m_ErrorTime.seconds();
        double deltaError = this.m_LastError - error;
        this.m_IntegratedError += error * deltaTime;
        this.m_DerivedError = deltaError / deltaTime;
        this.m_LastError = error;
    }
    public void clearPreviousErrors(){
        this.m_LastError = 0;
        this.m_IntegratedError = 0;
        this.m_DerivedError = 0;
    }
    public PIDErrorInfo getPIDErrorInfo(){
        return new PIDErrorInfo(this.m_LastError,this.m_IntegratedError,this.m_DerivedError);
    }
}
