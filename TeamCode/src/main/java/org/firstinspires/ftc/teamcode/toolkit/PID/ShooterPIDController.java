package org.firstinspires.ftc.teamcode.toolkit.PID;

import org.firstinspires.ftc.teamcode.UpliftRobot;

public class ShooterPIDController {
    double kP, kI, kD, p, i, d, targetVelocity, tolerance, lastP, maxP, lastTime, output;
    UpliftRobot robot;

    public ShooterPIDController(double kP, double kI, double kD, double tolerance, double targetVel, UpliftRobot robot) {
        this.robot = robot;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.tolerance = tolerance;
        this.targetVelocity = targetVel;
        this.p = 0;
        this.i = 0;
        this.d = 0;
        this.lastP = 0;
        this.lastTime = 0;
        this.maxP = 0;
        this.output = 0;

    }

    public double getOutput() {

        p = targetVelocity - robot.shooter1SmoothVel;
        i += p * (System.currentTimeMillis() - lastTime);
        d = ((p-lastP)/(System.currentTimeMillis() - lastTime));
        lastP = p;
        lastTime = System.currentTimeMillis();
        if(Math.signum(p) != Math.signum(i)) {
            i = 0;
        }
        if(Math.abs(p) < Math.abs(tolerance)){
            output = 0;
        } else{
            output = p * kP + i * kI + d * kD; 
        }

        return output;
    }

}
