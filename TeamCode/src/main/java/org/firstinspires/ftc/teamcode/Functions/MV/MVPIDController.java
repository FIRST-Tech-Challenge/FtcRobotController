package org.firstinspires.ftc.teamcode.Functions.MV;

public class MVPIDController {
    private double kP;
    private double kI;
    private double kD;
    private double tolerance;
    private double target = 0;
    private double start = 0;
    //private double currentError;
    private double previousError;
    private double integral;
    private double derivative;
    private double minInput;
    private double maxInput;
    private double minOutput;
    private double maxOutput;
    private boolean continuous;

    public void setkP( double kP){
        this.kP = kP;
    }

    public void setkI( double kI){
        this.kI = kI;
    }

    public void setkD( double kD){
        this.kD = kD;
    }

    public void pidController (double kP, double kI, double kD, double tolerance){
        setkP(kP);
        setkI(kI);
        setkD(kD);
        this.tolerance = tolerance;
    }


    /**
     * This method sets the PID Controller's continuous mode.
     * @param continuous : (boolean) value for minInput
     */
    public void setContinuous(boolean continuous){
        this.continuous = continuous;
    }

    /**
     * This method gets the PID Controller's continuous mode.
     */
    public boolean isContinuous(){
        return continuous;
    }

    /**
     * This method sets the minInput and maxInput value.
     * @param minInput : (double) value for minInput
     * @param maxInput : (double) value for maxInput
     */
    public void setInputRange(double minInput, double maxInput){
        this.minInput = minInput;
        this.maxInput = maxInput;
    }

    /**
     * This method returns the minInput and maxInput values
     * as an array.
     */
    public double[] getInputRange(){
        return new double[] {minInput, maxInput};
    }

    /**
     * This method adjust the input value
     * to be between minInput and maxInput.
     * @param input : (double) given value for a input
     */
    public double adjustInput(double input){
        // Adjust the input value
        input = Math.max(input, minInput);
        input = Math.min(input, maxInput);

        return input;
    }

    /**
     * This method sets the minOutput and maxOutput value.
     * @param minOutput : (double) value for minOutput
     * @param maxOutput : (double) value for maxOutput
     */
    public void setOutputRange(double minOutput, double maxOutput){
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }

    /**
     * This method returns the minOutput and maxOutput values
     * as an array.
     */
    public double[] getOutputRange(){
        return new double[] {minOutput, maxOutput};
    }

    /**
     * This method adjust the input value
     * to be between minInput and maxInput.
     * @param output : (double) given value for a output
     */
    public double adjustOutput(double output){
        // Adjust the output value
        output = Math.max(output, minInput);
        output = Math.min(output, maxInput);

        return output;
    }

    /**
     * This method wraps the input value around the
     * minInput and maxInput values if the controller is
     * in continuous mode.
     * @param input : (double) value for input
     */
    public double wrapToRange(double input){
        if (continuous) {
            if (Math.abs(input - target) > (maxInput-minInput) / 2) {
                if (input > target) {
                    input -= maxInput - minInput;
                } else {
                    input += maxInput - minInput;
                }
            }
        }

        return input;
    }

    public void setTarget(double target){
        this.target = target;
        this.integral = 0;
        this.derivative = 0;
    }

    public double getTarget(){
        return target;
    }

    public void setTolerance(double tolerance){
        this.tolerance = tolerance;
    }

    public double getTolerance(){
        return tolerance;
    }

    public boolean onTarget(){
        double  error = Math.abs(start - target);
        return error < tolerance;
    }

    public double calculate(double current){
        // Set new start position
        this.start = current;

        // Calculating the error
        double currentError = target - current;

        // Proportional term
        double p = kP * currentError;

        // Integral term
        integral = integral + currentError;
        double i = kI * integral;

        // Derivative term
        derivative = currentError - previousError;
        double d = kD * derivative;

        // Update the previous error
        previousError = currentError;

        // Calculate the output
        double output = p + i + d;

        // Checking if the error is within the tolerance
        if (Math.abs(currentError) < tolerance) {
            integral = 0;
            derivative = 0;
        }

        return output;
    }
}
