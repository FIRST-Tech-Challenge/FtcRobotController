package org.firstinspires.ftc.teamcode.PIDs;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MotorPID {
    double Kp, Ki, Kd; // PID values to tune
    double errorSum = 0; // The sum of errors for the integral term
    double lastError = 0; // The error calculated on the previous run of the PID
    double error = 0;
    boolean isTargetValReached = false;

    // We do not declare an ElapsedTime object because that will be passed through from another class
    // like in the "getDerivativeTerm" or "getIntegralTerm" method

    public MotorPID(double kp, double ki, double kd){
    // Setting the PID terms in this class to the ones passed through from another class
    this.Kp = kp;
    this.Ki = ki;
    this.Kd = kd;
    }

    public double getProportionalTerm(double error){
        return Kp * error;
        // "error" represents the difference between the target position and the encoder value
        /*
              Ex.
              double encoderPosition = armMotor.getPosition();
              double error = reference - encoderPosition;
        */
    }

    public double getIntegralTerm(double error, ElapsedTime timer){
        errorSum += error*timer.seconds();
        return Ki * errorSum;
    }

    public double getDerivativeTerm(double error, ElapsedTime timer){
        double derivative = (error - lastError) / timer.seconds();
        return Kd * derivative;
    }


    public double getTunedPosition(ElapsedTime timer){
        return getProportionalTerm(error) + getIntegralTerm(error, timer) + getDerivativeTerm(error, timer);
    }

}
