package org.firstinspires.ftc.teamcode;

public class PID
{
    double kP = 0.24, kI = 0, kD = 0, error, target;

    void setTarget(double setpoint)
    {
        target = setpoint;
    }

    double runControlLoop(double position)
    {
        error = target - position;
        return kP * error;
    }
}
