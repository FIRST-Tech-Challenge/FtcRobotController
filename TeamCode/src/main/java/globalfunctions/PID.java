package globalfunctions;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    public double sKp = 0;
    public double sKd = 0;
    public double sKi = 0;

    public double Kp = 0;
    public double Kd = 0;
    public double Ki = 0;

    public double error = 0;
    public double lastError = 0;
    public double derivative = 0;
    public double integral = 0;

    public double lastTime = -0.01;
    public double deltaTime = 0;

    public double deltaError = 0;

    public double restPow = 0;

    public ElapsedTime timer = new ElapsedTime();

    public double sacc = 0;
    public double acc = 0;

    public double maxI = 1000;


    public void setCoefficients(double k, double d, double i){
        sKp = k;
        sKd = d;
        sKi = i;
        scaleCoeffs(1);
    }
    public void setRestPow(double pow){
        restPow = pow;
    }
    public void setAcc(double in){
        sacc = in;
        acc = in;
    }

    public double getPower(){
        return Math.signum(error)*(Kp*abs(error) - Kd*abs(derivative) + Ki*abs(integral) + restPow);
    }

    public double abs(double in){
        return Math.abs(in);
    }

    public void update(double error){
        deltaTime = timer.seconds()-lastTime;
        lastTime += deltaTime;

        deltaError = error - lastError;
        this.error += deltaError;
        lastError = error;

        derivative = deltaError/deltaTime;

        if(Ki*abs(integral) < maxI || Math.signum(integral*error) == -1) {
            integral += error * deltaTime;
        }

    }

    public void reset(){
        error = 0;
        lastError = 0;
        derivative = 0;
        integral = 0;
        lastTime = -0.1;
        deltaTime = 0;
        deltaError = 0;
    }

    public boolean done(){
        return abs(error) < acc;
    }

    public void setMaxI(double maxI){
        this.maxI = maxI;
    }

    public void scaleCoeffs(double scale) {
        if (scale < 100) {
            Kp = sKp * scale;
            Kd = sKd * scale;
            Ki = sKi * scale;
        }
    }

    public void scaleAccs(double scale){
        acc = sacc * scale;
    }




}
