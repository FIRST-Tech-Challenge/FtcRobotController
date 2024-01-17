package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Drivebases.Swerve.DiffySwerve.Aarush;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SwervePID {
    double integralSum = 0;
    private double lastError = 0;
    private double kP, kI, kD;

    // kP = Aggressiveness
    // kD = Dampener (like a spring)
    // kI = how long does the patience last (0 = will wait patiently forever!)

    // kP
    // The proportional term or Kp is a value that is directly proportional to the error of the system.
    // To get the Proportional output, we take the Kp * error and add it to our system's input.
    // Changing Kp determines how fast or how slow the system moves towards 0 error.
    // You can think of Kp like a rubber band, where the thicker it is,
    // the harder it is to pull away from its equilibrium point.

    // kD
    // The derivative term or Kd is a value directly proportional to the rate of change of the error of the system.
    // To calculate this, we can find the slope of the error from the last update to the current update of the loop.
    // Derivative ensures that our system isn't responding too quickly by penalizing excessive rates of change.
    // You can think of Kd as increasing the friction of your skates on an ice skating rink.
    // The lower the friction, the harder to control, and you need a little bit of dampening to allow you
    // to keep yourself stable.

    // kI
    // The Integral term or Ki is directly proportional to the sum of all the errors over time.
    // This allows the system to overcome nonlinear effects such as static friction that would otherwise
    // be difficult to tune out. This works by adding up the remaining error until the output grows large enough
    // to overcome the constant disturbance.

    ElapsedTime timer = new ElapsedTime();

    public SwervePID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void overridePID (double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double runPID(double state, double target) {
        double error = target - state;

        double derivative = (error - lastError) / timer.seconds();
        integralSum += error * timer.seconds();

        lastError = error;
        timer.reset();

        double output = (error * kP) + (derivative * kD) + (integralSum * kI);
        return output;
    }
}