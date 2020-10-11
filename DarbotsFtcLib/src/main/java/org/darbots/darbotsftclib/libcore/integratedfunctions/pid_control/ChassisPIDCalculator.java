package org.darbots.darbotsftclib.libcore.integratedfunctions.pid_control;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPose2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;

public class ChassisPIDCalculator {
    public PIDCoefficients xPIDCoefficients;
    public PIDCoefficients yPIDCoefficients;
    public PIDCoefficients rotZPIDCoefficients;
    private ChassisPIDIntegrator m_PIDCalculator;
    public ChassisPIDCalculator(PIDCoefficients xPIDCoefficients, PIDCoefficients yPIDCoefficients, PIDCoefficients rotZPIDCoefficients){
        this.xPIDCoefficients = xPIDCoefficients;
        this.yPIDCoefficients = yPIDCoefficients;
        this.rotZPIDCoefficients = rotZPIDCoefficients;
        this.m_PIDCalculator = new ChassisPIDIntegrator();
    }
    public ChassisPIDCalculator(ChassisPIDCalculator oldPIDController){
        this.xPIDCoefficients = oldPIDController.xPIDCoefficients;
        this.yPIDCoefficients = oldPIDController.yPIDCoefficients;
        this.rotZPIDCoefficients = oldPIDController.rotZPIDCoefficients;
        this.m_PIDCalculator = new ChassisPIDIntegrator(oldPIDController.m_PIDCalculator);
    }
    public void clearPreviousErrors(){
        this.m_PIDCalculator.clearPreviousErrors();
    }
    public ChassisPIDIntegrator getPIDCalculator(){
        return this.m_PIDCalculator;
    }
    public RobotVector2D getPIDPower(){
        ChassisPIDIntegrator.ChassisPIDErrorInfo errorInfo = this.m_PIDCalculator.getPIDErrorInfo();
        double XProportionalPower = this.xPIDCoefficients.Kp * errorInfo.Error.X;
        double YProportionalPower = this.yPIDCoefficients.Kp * errorInfo.Error.Y;
        double RotZProportionalPower = this.rotZPIDCoefficients.Kp * errorInfo.Error.getRotationZ();
        double XPower = XProportionalPower + this.xPIDCoefficients.Ki * errorInfo.Integral.X + this.xPIDCoefficients.Kd * errorInfo.Derivative.X;
        double YPower = YProportionalPower + this.yPIDCoefficients.Ki * errorInfo.Integral.Y + this.yPIDCoefficients.Kd * errorInfo.Derivative.Y;
        double RotZPower = RotZProportionalPower + this.rotZPIDCoefficients.Ki * errorInfo.Integral.getRotationZ() + this.rotZPIDCoefficients.Kd * errorInfo.Derivative.getRotationZ();
        return new RobotVector2D(XPower,YPower,RotZPower);
    }
    public RobotVector2D getPIPower(){
        ChassisPIDIntegrator.ChassisPIDErrorInfo errorInfo = this.m_PIDCalculator.getPIDErrorInfo();
        double XProportionalPower = this.xPIDCoefficients.Kp * errorInfo.Error.X;
        double YProportionalPower = this.yPIDCoefficients.Kp * errorInfo.Error.Y;
        double RotZProportionalPower = this.rotZPIDCoefficients.Kp * errorInfo.Error.getRotationZ();
        double XPower = XProportionalPower + this.xPIDCoefficients.Ki * errorInfo.Integral.X;
        double YPower = YProportionalPower + this.yPIDCoefficients.Ki * errorInfo.Integral.Y;
        double RotZPower = RotZProportionalPower + this.rotZPIDCoefficients.Ki * errorInfo.Integral.getRotationZ();
        return new RobotVector2D(XPower,YPower,RotZPower);
    }
    public RobotVector2D getPDPower(){
        ChassisPIDIntegrator.ChassisPIDErrorInfo errorInfo = this.m_PIDCalculator.getPIDErrorInfo();
        double XProportionalPower = this.xPIDCoefficients.Kp * errorInfo.Error.X;
        double YProportionalPower = this.yPIDCoefficients.Kp * errorInfo.Error.Y;
        double RotZProportionalPower = this.rotZPIDCoefficients.Kp * errorInfo.Error.getRotationZ();
        double XPower = XProportionalPower + this.xPIDCoefficients.Kd * errorInfo.Derivative.X;
        double YPower = YProportionalPower + this.yPIDCoefficients.Kd * errorInfo.Derivative.Y;
        double RotZPower = RotZProportionalPower + this.rotZPIDCoefficients.Kd * errorInfo.Derivative.getRotationZ();
        return new RobotVector2D(XPower,YPower,RotZPower);
    }
    public RobotVector2D getPPower(){
        ChassisPIDIntegrator.ChassisPIDErrorInfo errorInfo = this.m_PIDCalculator.getPIDErrorInfo();
        double XProportionalPower = this.xPIDCoefficients.Kp * errorInfo.Error.X;
        double YProportionalPower = this.yPIDCoefficients.Kp * errorInfo.Error.Y;
        double RotZProportionalPower = this.rotZPIDCoefficients.Kp * errorInfo.Error.getRotationZ();
        double XPower = XProportionalPower;
        double YPower = YProportionalPower;
        double RotZPower = RotZProportionalPower;
        return new RobotVector2D(XPower,YPower,RotZPower);
    }
    public RobotVector2D getIPower(){
        ChassisPIDIntegrator.ChassisPIDErrorInfo errorInfo = this.m_PIDCalculator.getPIDErrorInfo();
        double XPower = this.xPIDCoefficients.Ki * errorInfo.Integral.X;
        double YPower = this.yPIDCoefficients.Ki * errorInfo.Integral.Y;
        double RotZPower = this.rotZPIDCoefficients.Ki * errorInfo.Integral.getRotationZ();
        return new RobotVector2D(XPower,YPower,RotZPower);
    }
    public RobotVector2D getDPower(){
        ChassisPIDIntegrator.ChassisPIDErrorInfo errorInfo = this.m_PIDCalculator.getPIDErrorInfo();
        double XPower = this.xPIDCoefficients.Kd * errorInfo.Derivative.X;
        double YPower = this.yPIDCoefficients.Kd * errorInfo.Derivative.Y;
        double RotZPower = this.rotZPIDCoefficients.Kd * errorInfo.Derivative.getRotationZ();
        return new RobotVector2D(XPower,YPower,RotZPower);
    }
    public void feedError(RobotPose2D error){
        this.m_PIDCalculator.feedError(error);
    }
    public void feedError(double X, double Y, double RotZ){
        this.m_PIDCalculator.feedError(X,Y,RotZ);
    }
}
