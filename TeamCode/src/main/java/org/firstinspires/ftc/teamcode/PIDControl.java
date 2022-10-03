package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDControl(double reference, double state) {

    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    private double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    double error = reference - state;
    integralSum += error * timer.seconds();
    double derivative = (error - lastError ) / timer.seconds();


    lastError = error;
    timer.reset();

    double output = (error * Kp) + ( derivative * Kd) + (integralSum * Ki);
    return output;


}
