package globalfunctions;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
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

    public double acc = 0;


    public void setCoefficients(double k, double d, double i){
        Kp = k;
        Kd = d;
        Ki = i;
    }
    public void setRestPow(double pow){
        restPow = pow;
    }
    public void setAcc(double in){
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

        integral += error*deltaTime;

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




}
