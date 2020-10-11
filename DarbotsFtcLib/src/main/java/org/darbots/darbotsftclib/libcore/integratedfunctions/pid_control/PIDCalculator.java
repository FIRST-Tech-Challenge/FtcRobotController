package org.darbots.darbotsftclib.libcore.integratedfunctions.pid_control;

public class PIDCalculator {
    public PIDCoefficients pidCoefficients;
    private PIDIntegrator m_PIDIntegrator;
    public PIDCalculator(PIDCoefficients pidCoefficients){
        this.pidCoefficients = pidCoefficients;
        this.m_PIDIntegrator = new PIDIntegrator();
    }
    public PIDCalculator(PIDCalculator oldPIDCalculator){
        this.pidCoefficients = oldPIDCalculator.pidCoefficients;
        this.m_PIDIntegrator = new PIDIntegrator(oldPIDCalculator.m_PIDIntegrator);
    }
    public void clearPreviousErrors(){
        this.m_PIDIntegrator.clearPreviousErrors();
    }
    public PIDIntegrator getPIDCalculator(){
        return this.m_PIDIntegrator;
    }
    public double getPIDPower(){
        PIDIntegrator.PIDErrorInfo errorInfo = this.m_PIDIntegrator.getPIDErrorInfo();
        return this.pidCoefficients.Kp * errorInfo.Error + this.pidCoefficients.Ki * errorInfo.Integral + this.pidCoefficients.Kd * errorInfo.Derivative;
    }
    public double getPIPower(){
        PIDIntegrator.PIDErrorInfo errorInfo = this.m_PIDIntegrator.getPIDErrorInfo();
        return this.pidCoefficients.Kp * errorInfo.Error + this.pidCoefficients.Ki * errorInfo.Integral;
    }
    public double getPDPower(){
        PIDIntegrator.PIDErrorInfo errorInfo = this.m_PIDIntegrator.getPIDErrorInfo();
        return this.pidCoefficients.Kp * errorInfo.Error + this.pidCoefficients.Kd * errorInfo.Derivative;
    }
    public double getPPower(){
        PIDIntegrator.PIDErrorInfo errorInfo = this.m_PIDIntegrator.getPIDErrorInfo();
        return this.pidCoefficients.Kp * errorInfo.Error;
    }
    public double getIPower(){
        PIDIntegrator.PIDErrorInfo errorInfo = this.m_PIDIntegrator.getPIDErrorInfo();
        return this.pidCoefficients.Ki * errorInfo.Integral;
    }
    public double getDPower(){
        PIDIntegrator.PIDErrorInfo errorInfo = this.m_PIDIntegrator.getPIDErrorInfo();
        return this.pidCoefficients.Kd * errorInfo.Derivative;
    }
    public void feedError(double error){
        this.m_PIDIntegrator.feedError(error);
    }
}
