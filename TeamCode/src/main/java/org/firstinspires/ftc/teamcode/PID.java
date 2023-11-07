package org.firstinspires.ftc.teamcode;

public class PID {
    private double error;
    private double totalError;
    private double prevError;

    private double PValue;
    private double IValue;
    private double DValue;

    // Dictates the inputs and outputs
    private double maxInput;
    private double minInput;
    private double maxOutput = 1500 ;// defaults to 100% and -100% motor power
    private double minOutput = -1500;

    private boolean continuous = false; // only for absolute encoders
    private double setPoint; // this will be set continuously
    private double output;
    private double result;

    public PID(double kp, double ki, double kd){
        PValue = kp;
        IValue = ki;
        DValue = kd;
    }

    public double updatePID(double value){
        error = setPoint - value;
        if (continuous) {
            if (Math.abs(error) > (maxInput - minInput) / 2) {
                if (error > 0) {
                    error = error - maxInput + minInput;
                }
                else {
                    error = error + maxInput - minInput;
                }
            }
        }

        if ((error * PValue < maxOutput) && (error * PValue > minOutput)) {
            totalError += error;
        } else {
            totalError = 0;
        }

        result = (PValue * error + IValue * totalError + DValue * (error - prevError));
        prevError = error;
        result = clamp(result);
        return result;
    }

    public void setPID(double p, double i , double d){
        PValue = p;
        IValue = i;
        DValue = d;
    }

    public double getResult(){
        return result;
    }
    public void setMaxOutput(double output){
        maxOutput = output;
    }
    public void setMinOutput(double output){
        minOutput = output;
    }
    public void setMinInput(double input){
        minInput = input;
    }
    public void setMaxInput(double input){
        maxInput = input;
    }
    public void setSetPoint(double target){
        setPoint = target;

    }
    public void setContinuous(boolean value){
        continuous = value;
    }


    public int getSetPoint(){
        return (int) setPoint;
    }
    public double clamp(double input){
        if(input > maxOutput){
            return maxOutput;
        }
        if(input < minOutput){
            return minOutput;
        }
        return input;
    }
    public double getError(){
        return prevError;
    }

}