package org.firstinspires.ftc.teamcode.resourses;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double pConstant, dConstant;
    private double power, target;
    private double lastTime, lastError;

    private ElapsedTime runtime = new ElapsedTime();

    public PIDController(double pConstant, double dConstant, ElapsedTime runtime) {
        this.pConstant = pConstant;
        this.dConstant = dConstant;
        this.runtime = runtime;
    }

    public void update(double currentPosition) {
        double error = currentPosition - target;

        double derivative = (error - lastError) / (runtime.time() - lastTime);

        power = -error * pConstant + (-dConstant * derivative);

        if (power > 1 || power < -1) power /= Math.abs(power);
        
        lastTime = runtime.time();
        lastError = error;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public double getPower(){
        return power;
    }
}
