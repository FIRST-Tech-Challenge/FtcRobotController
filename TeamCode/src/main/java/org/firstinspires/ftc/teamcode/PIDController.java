package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double Kp, Ki, Kd, setPoint;
    private double integralSum = 0;
    private double lastError = 0;

    public PIDController(double Kp, double Ki, double Kd){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public double getLastError(){
        return lastError;
    }

    public void setSetPoint(double setPoint){
        this.setPoint = setPoint;
    }

    public double calculatePower(DcMotorEx motor, double timeElapsed) {
        double encoderPosition = motor.getCurrentPosition();
        double error = setPoint - encoderPosition;

        //rate of change of error
        double derivative = (error-lastError) / timeElapsed;

        //sum of errors over time
        integralSum += (error * timeElapsed);

        //motor power
        double power =  (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        lastError = error;
        return power;
    }

}
