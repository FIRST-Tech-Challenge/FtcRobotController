package globalfunctions;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
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
    //Change in error
    public double deltaError = 0;
    //Timer
    public ElapsedTime timer = new ElapsedTime();
    //Current accuracy
    public double acc = 0;

    //Sets coeffs
    public void setCoefficients(double k, double d, double i){
        Kp = k;
        Kd = d;
        Ki = i;
    }
    //Sets accuracies
    public void setAcc(double in){
        acc = in;
    }
    //Sets accuracies
    public void setScale(double in){
        scale = in;
    }
    //Gets the power for the motor
    public double getPower(){
        return Math.signum(error) * (Kp* scale * abs(error) - Kd * abs(derivative) + Ki * abs(integral));
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

        integral += error * deltaTime;
    }
    //Resets everything
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




}
