package globalfunctions;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    //starting coeffs
    public double sKp = 0;
    public double sKd = 0;
    public double sKi = 0;
    //Current coeffs
    public double Kp = 0;
    public double Kd = 0;
    public double Ki = 0;
    //Current values
    public double error = 0;
    public double lastError = 0;
    public double derivative = 0;
    public double integral = 0;
    //Last time and delta time
    public double lastTime = -0.01;
    public double deltaTime = 0;

    //Scale to scale ks
    public double scale = 1;
    //Scale to scale accs
    public double AccScale = 1;


    //Change in error
    public double deltaError = 0;
    //Rest power to have
    public double restPow = 0;
    //Timer
    public ElapsedTime timer = new ElapsedTime();
    //starting accuracy
    public double sacc = 0;
    //Current accuracy
    public double acc = 0;
    //MaxI value
    public double maxI = 1000;
    //I range value
    public double rangeI = 0;
    //MaxD value
    public double maxD = 1000;

    public boolean disableI = false;

    //Sets coeffs
    public void setCoefficients(double k, double d, double i){
        sKp = k;
        sKd = d;
        sKi = i;
        scaleCoeffs(scale);
    }
    //Sets rest pow
    public void setRestPow(double pow){
        restPow = pow;
    }
    //Sets range of I
    public void setRangeI(double ri){rangeI = ri; }
    //Sets accuracies
    public void setAcc(double in){
        sacc = in;
        scaleAccs(AccScale);
    }
    //Gets the power for the motor
    public double getPower(){
        return Math.signum(error) * (Kp * abs(error) - Kd * abs(derivative) + Ki * abs(integral) + restPow);
//        return 0;
    }
    //Absolute value
    public double  abs(double in){
        return Math.abs(in);
    }
    //Updates the values
    public void update(double error){
        deltaTime = timer.seconds()-lastTime;
        lastTime += deltaTime;

        deltaError = error - lastError;
        this.error += deltaError;
        lastError = error;

        derivative = deltaError/deltaTime;

        if(!disableI) {
            if (abs(error) < rangeI) {
                //|| Math.signum(integral * error) == -1
                if (Ki * abs(integral + error * deltaTime) < maxI) {
                    integral += error * deltaTime;
                } else {
                    integral = 0;
                }
            }
        }else{
            integral = 0;
        }

        if(Kd*abs(derivative) > maxD) {
            derivative = maxD * Math.signum(derivative);
        }

    }
    //Resets eberything
    public void reset(){
        error = 0;
        lastError = 0;
        derivative = 0;
        integral = 0;
        lastTime = -0.01;
        deltaTime = 0;
        deltaError = 0;
        timer.reset();
    }
    //Error is within accuracy
    public boolean isDone(){
        return abs(error) < acc;
    }
    //Sets max I
    public void setMaxI(double maxI){
        this.maxI = maxI;
    }
    //Sets max D
    public void setMaxD(double maxD){
        this.maxD = maxD;
    }
    //Scales coeefs by scale
    public void scaleCoeffs(double scale) {
        if (scale < 100) {
            this.scale = scale;
            Kp = sKp * scale;
            Kd = sKd * scale;
            Ki = sKi * scale;
        }
    }
    //Scales accuracies
    public void scaleAccs(double scale){
        this.AccScale = scale;
        acc = sacc * scale;
    }




}
