package com.qualcomm.robotcore.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public interface DcMotorEx extends DcMotor {
    void setMotorEnable();
    void setMotorDisable();
    boolean isMotorEnabled();
    void setVelocity(double angularRate);
    void setVelocity(double angularRate, AngleUnit unit);
    double getVelocity();
    double getVelocity(AngleUnit unit);
    // ### void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException;
    void setVelocityPIDFCoefficients(double p, double i, double d, double f);
    void setPositionPIDFCoefficients(double p);
    // ### PIDFCoefficients getPIDFCoefficients(RunMode mode);
    void setTargetPositionTolerance(int tolerance);
    int getTargetPositionTolerance();
    double getCurrent(CurrentUnit unit);
    double getCurrentAlert(CurrentUnit unit);
    void setCurrentAlert(double current, CurrentUnit unit);
    boolean isOverCurrent();

}
