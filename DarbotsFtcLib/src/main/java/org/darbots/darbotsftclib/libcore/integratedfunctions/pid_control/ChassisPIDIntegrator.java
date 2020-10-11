package org.darbots.darbotsftclib.libcore.integratedfunctions.pid_control;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotVector2D;

public class ChassisPIDIntegrator {
    public static class ChassisPIDErrorInfo{
        public RobotVector2D Error;
        public RobotVector2D Integral;
        public RobotVector2D Derivative;
        public ChassisPIDErrorInfo(){
            this.Error = new RobotVector2D(0,0,0);
            this.Integral = new RobotVector2D(0,0,0);
            this.Derivative = new RobotVector2D(0,0,0);
        }
        public ChassisPIDErrorInfo(RobotVector2D Error, RobotVector2D Integral, RobotVector2D Derivative){
            this.Error = new RobotVector2D(Error);
            this.Integral = new RobotVector2D(Integral);
            this.Derivative = new RobotVector2D(Derivative);
        }
        public ChassisPIDErrorInfo(ChassisPIDErrorInfo Info){
            this.Error = new RobotVector2D(Info.Error);
            this.Integral = new RobotVector2D(Info.Integral);
            this.Derivative = new RobotVector2D(Info.Derivative);
        }
    }
    RobotVector2D m_IntegratedError;
    RobotVector2D m_DerivedError;
    RobotVector2D m_LastError;
    ElapsedTime m_ErrorTime;
    public ChassisPIDIntegrator(){
        this.m_IntegratedError = new RobotVector2D(0,0,0);
        this.m_DerivedError = new RobotVector2D(0,0,0);
        this.m_LastError = new RobotVector2D(0,0,0);
        this.m_ErrorTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }
    public ChassisPIDIntegrator(ChassisPIDIntegrator oldCalculator){
        this.m_IntegratedError = new RobotVector2D(oldCalculator.m_IntegratedError);
        this.m_DerivedError = new RobotVector2D(0,0,0);
        this.m_LastError = new RobotVector2D(oldCalculator.m_LastError);
        this.m_ErrorTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }
    public void feedError(RobotVector2D error){
        double deltaTime = this.m_ErrorTime.seconds();
        double deltaErrorX = this.m_LastError.X - error.X, deltaErrorY = this.m_LastError.Y - error.Y, deltaErrorRotZ = this.m_LastError.getRotationZ() - error.getRotationZ();

        this.m_IntegratedError.offsetValues(error.X * deltaTime, error.Y * deltaTime, error.getRotationZ() * deltaTime);

        this.m_DerivedError.setValues(deltaErrorX / deltaTime,deltaErrorY / deltaTime,deltaErrorRotZ / deltaTime);

        this.m_LastError.setValues(error);
    }
    public void feedError(double X, double Y, double RotZ){
        double deltaTime = this.m_ErrorTime.seconds();
        double deltaErrorX = this.m_LastError.X - X, deltaErrorY = this.m_LastError.Y - Y, deltaErrorRotZ = this.m_LastError.getRotationZ() - RotZ;

        this.m_IntegratedError.offsetValues(X * deltaTime, X * deltaTime, RotZ * deltaTime);

        this.m_DerivedError.setValues(deltaErrorX / deltaTime,deltaErrorY / deltaTime,deltaErrorRotZ / deltaTime);

        this.m_LastError.setValues(X,Y,RotZ);
    }
    public void clearPreviousErrors(){
        this.m_LastError.setValues(0,0,0);
        this.m_IntegratedError.setValues(0,0,0);
        this.m_DerivedError.setValues(0,0,0);
    }
    public ChassisPIDErrorInfo getPIDErrorInfo(){
        return new ChassisPIDErrorInfo(this.m_LastError,this.m_IntegratedError,this.m_DerivedError);
    }
}
