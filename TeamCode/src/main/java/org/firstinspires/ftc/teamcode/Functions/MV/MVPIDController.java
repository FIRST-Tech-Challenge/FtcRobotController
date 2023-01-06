package org.firstinspires.ftc.teamcode.Functions.MV;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * This class implements a PID controller. A PID (proportional-integral-derivative) controller is a
 * control loop feedback mechanism that uses a control system to continuously calculate an error
 * value between a desired setpoint and a measured process variable, and uses the error value to
 * adjust the process in a way that minimizes the error over time.
 * <p>
 * In a PID controller, the output power of the effector (such as a motor) is adjusted based on the
 * error value, which is calculated using three separate terms:
 * <ol>
 *    <li>The proportional term, which is based on the current error value and is used to make a
 *    correction that is proportional to the error.
 *
 *    <li>The integral term, which is based on the accumulated error over time and is used to correct
 *    for any systematic bias in the system.
 *
 *    <li>The derivative term, which is based on the rate of change of the error and is used to
 *    anticipate future errors and make corrections in advance.
 * </ol>
 * <p>
 * By adjusting the relative contributions of these three terms, a PID controller can achieve a wide
 * range of control behavior, including overshoot and oscillation, depending on the needs of the
 * system being controlled.
 */
public class MVPIDController {

    // instance variables for PID controller constants
    private double kP;                  // proportional gain
    private double kI;                  // integral gain
    private double kD;                  // derivative gain
    private double kF;                  // forward feed gain

    // instance variables for error values and elapsed time
    private double tolerance;           // tolerance
    private double target = 0;          // targeted value (setPoint)
    private double start = 0;           // starting position
    private double totalError = 0;      // total error
    private double previousError;       // previous error
    private double integral;            //integral
    private double derivative;          // derivative
    private ElapsedTime elapsedTime;    // elapsed time object
    private boolean firstRun;           // flag for first run
    private double currentTime = 0.0;   // current time
    private double deltaTime = 0.0;     // time delta

    // input and output limits
    private double minInput;            // minimum input
    private double maxInput;            // maximum input
    private double minOutput;           // minimum output
    private double maxOutput;           // maximum output

    // continuous mode flag
    private boolean continuous;         // continuous mode

    // telemetry object
    private Telemetry telemetry;

    /**
     * Constructor: Create an instance of the object.
     * <p></p>
     * This is the constructor for the MVPIDController class. It sets the proportional, integral,
     * derivative, and feed forward constants for the PID controller, as well as the tolerance
     * value. It also initializes an elapsed time object and sets a flag for the first run.
     *</p></p>
     * @param kP specifies the Proportional constant.
     * @param kI specifies the Integral constant.
     * @param kD specifies the Differential constant.
     * @param kF specifies the Feed forward constant.
     * @param tolerance specifies the PID tolerance.
     */
    public void pidController (double kP, double kI, double kD, double kF, double tolerance){
        try {
            //setkP(kP);
            this.kP = kP;
            //setkI(kI);
            this.kI = kI;
            //setkD(kD);
            this.kD = kD;
            //setkF(kF);
            this.kF = kF;

            if (tolerance < 0) {
                this.tolerance = Math.abs(tolerance);
                throw new IllegalArgumentException("Tolerance must be positive");
            }
            this.tolerance = tolerance;

            // initialize elapsed time object
            firstRun = true;
            elapsedTime =new ElapsedTime();  // initialize elapsed time

        } catch (NullPointerException e) {
            System.out.println("NullPointerException caught in pidController: " + e.getMessage());
        } catch (Exception e) {
            System.out.println("Exception caught in PIDController: " + e.getMessage());
        }
    }

    /**
     * This method sets the telemetry.
     * @param telemetry : (Telemetry) value for kP
     */
    // setter method for telemetry object
    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * This method sets the Proportional term of the PID Controller.
     * @param kP : (boolean) value for kP
     */
    public void setkP( double kP) {
        try {
            if (kP < 0) {
                throw new IllegalArgumentException("kP must be positive");
            }
            this.kP = kP;
        } catch (NullPointerException e) {
            telemetry.addData("Error:", "NullPointerException caught in setkP: " + e.getMessage());
            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("Error:", "Exception caught in setkP: " + e.getMessage());
            telemetry.update();
        }
    }

    /**
     * This method sets the Integral term of the PID Controller.
     * @param kI : (boolean) value for kI
     */
    public void setkI( double kI){
        try {
            if (kI < 0) {
                throw new IllegalArgumentException("kI must be positive");
            }
            this.kI = kI;
        } catch (NullPointerException e) {
            sendError("NullPointerException caught in setting kI: " + e.getMessage());
            //System.out.println("NullPointerException caught in setting kI: " + e.getMessage());
        } catch (Exception e) {
            sendError("Exception caught in setting kI: " + e.getMessage());
            //System.out.println("Exception caught in setting kI: " + e.getMessage());
        }
    }

    /**
     * This method sets the Derivative term of the PID Controller.
     * @param kD : (boolean) value for kD
     */
    public void setkD( double kD){
        try {
            if (kD < 0) {
                throw new IllegalArgumentException("kD must be positive");
            }
            this.kD = kD;
        } catch (NullPointerException e) {
            System.out.println("NullPointerException caught in setting kD: " + e.getMessage());
        } catch (Exception e) {
            System.out.println("Exception caught in setting kD: " + e.getMessage());
        }
    }

    /**
     * This method sets the Forward Feed term of the PID Controller.
     * @param kF : (boolean) value for kF
     */
    public void setkF( double kF){
        try {
            if (kF < 0) {
                throw new IllegalArgumentException("kF must be positive");
            }
            this.kF = kF;
        }
        catch (NullPointerException e) {
            System.out.println("NullPointerException caught in setting kF: " + e.getMessage());
        }
        catch (Exception e) {
            System.out.println("Exception caught in setting kF: " + e.getMessage());
        }
    }

    /**
     * This method gets the Proportional term of the PID Controller.
     */
    public double getkP(){
        return kP;
    }

    /**
     * This method gets the Integral term of the PID Controller.
     */
    public double getkI(){
        return kI;
    }

    /**
     * This method gets the Derivative term of the PID Controller.
     */
    public double getkD(){
        return kD;
    }

    /**
     * This method gets the Forward Feed term of the PID Controller.
     */
    public double getkF(){
        return kF;
    }

    /**
     * This method gets all the parameters of the PID Controller. It returns an array
     * of five elements: {kP, kI, kD, kF, tolerance}
     */
    public double[] getPIDParameters() {
        double[] parameters = new double[5];
        parameters[0] = kP;
        parameters[1] = kI;
        parameters[2] = kD;
        parameters[3] = kF;
        parameters[4] = tolerance;

        return parameters;
    }

    /**
     * This method sets the PID Controller's continuous mode.
     * @param continuous : (boolean) value for minInput
     */
    public void setContinuous(boolean continuous){
        // if (continuous != true || continuous != false) {
        //    throw new IllegalArgumentException("continuous must be a boolean value (true or false)");
        //}
        this.continuous = continuous;
    }

    /**
     * This method gets the PID Controller's continuous mode.
     */
    public boolean isContinuous(){
        return continuous;
    }

    /**
     * This method sets the range of valid input values (minimum and maximum input) for the PID
     * controller.
     * @param minInput : (double) value for minInput
     * @param maxInput : (double) value for maxInput
     */
    public void setInputRange(double minInput, double maxInput){
        // Checks to ensure that min ans max are numbers and min is less than max
        try {
            // Check if minInput is less than maxInput
            if (minInput > maxInput) {
                throw new IllegalArgumentException("Minimum input value must be less than maximum input value");
            }

            // Check if values are numbers
            if (!Double.isFinite(minInput) || !Double.isFinite(maxInput)) {
                throw new IllegalArgumentException("Input values must be finite numbers");
            }

            // Check if values are not NaN
            if (Double.isNaN(minInput) || Double.isNaN(maxInput)) {
                throw new IllegalArgumentException("Input values must not be NaN");
            }

            // Assign entered values
            this.minInput = minInput;
            this.maxInput = maxInput;

        } catch (IllegalArgumentException e) {
            System.out.println(e.getMessage());
        }
    }

    /**
     * This method returns the minInput and maxInput values
     * as an array.
     */
    public double[] getInputRange(){
        return new double[] {minInput, maxInput};
    }

    /**
     * This method adjust the input value to be between minInput and maxInput.
     * @param input : (double) given value for a input
     */
    public double adjustInput(double input){
        // Adjust the input value
        input = Math.max(input, minInput);
        input = Math.min(input, maxInput);

        return input;
    }

    /**
     * This method sets the range of valid output values (minimum and maximum output) for the PID
     * controller.
     * @param minOutput : (double) value for minOutput
     * @param maxOutput : (double) value for maxOutput
     */
    public void setOutputRange(double minOutput, double maxOutput){
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }

    /**
     * This method returns the minOutput and maxOutput values as an array.
     */
    public double[] getOutputRange(){
        return new double[] {minOutput, maxOutput};
    }

    /**
     * This method adjust the output value to be between minInput and maxInput.
     * @param output : (double) given value for a output
     */
    public double adjustOutput(double output){
        // Adjust the output value
        output = Math.max(output, minOutput);
        output = Math.min(output, maxOutput);

        return output;
    }

    /**
     * This method sets the continuous mode flag for the PID controller. If set to true, the input
     * values will be treated as continuous, meaning that the input value will "wrap around" when
     * it exceeds the maximum or minimum input value.
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

    /**
     * This method sets the target value (setpoint) for the PID controller.
     * @param target: (double) the target value
     */
    public void setTarget(double target) {
        try {
            this.target = target;
            this.integral = 0;
            this.derivative = 0;
        } catch (NullPointerException e) {
            System.out.println("NullPointerException caught in setTarget: " + e.getMessage());
        } catch (Exception e) {
            System.out.println("Exception caught in setTarget: " + e.getMessage());
        }
    }

    /**
     * This method returns the target value (setpoint) for the PID controller.
     * @return the target value
     */
    public double getTarget() {
        try {
            return target;
        } catch (NullPointerException e) {
            System.out.println("NullPointerException caught in getTarget: " + e.getMessage());
            return 0.0;
        } catch (Exception e) {
            System.out.println("Exception caught in getTarget: " + e.getMessage());
            return 0.0;
        }
    }

    /**
     * This method sets the starting position for the PID controller.
     * @param startValue: (double) the starting position
     */
    public void setStart(double startValue) {
        try {
            this.start = startValue;
        } catch (NullPointerException e) {
            System.out.println("NullPointerException caught in setStart: " + e.getMessage());
        } catch (Exception e) {
            System.out.println("Exception caught in setStart: " + e.getMessage());
        }
    }

    /**
     * This method returns the starting position for the PID controller.
     * @return the starting position
     */
    public double getStart() {
        try {
            return start;
        } catch (NullPointerException e) {
            System.out.println("NullPointerException caught in getStart: " + e.getMessage());
            return 0.0;
        } catch (Exception e) {
            System.out.println("Exception caught in getStart: " + e.getMessage());
            return 0.0;
        }
    }

    /**
     * This method sets the absolute tolerance value for the PID controller.
     * @param tolerance: (double) the tolerance value
     */
    public void setTolerance(double tolerance){
        try {
            if (tolerance < 0) {
                throw new IllegalArgumentException("Tolerance must be positive");
            }
            this.tolerance = tolerance;
        } catch (NullPointerException e) {
            System.out.println("NullPointerException caught in setAbsoluteTolerance: " + e.getMessage());
        } catch (Exception e) {
            System.out.println("Exception caught in setAbsoluteTolerance: " + e.getMessage());
        }
    }

    public double getTolerance(){
        return tolerance;
    }

    /**
     * This method sets the percentage tolerance value for the PID controller.
     * @param tolerance: (double) the tolerance value as a percentage of the setpoint
     */
    public void setPercentTolerance(double tolerance) {
        try {
            if (tolerance < 0 || tolerance > 100) {
                throw new IllegalArgumentException("Tolerance must be a value between 0 and 100");
            }
            this.tolerance = tolerance / 100 * target;
        } catch (NullPointerException e) {
            System.out.println("NullPointerException caught in setPercentTolerance: " + e.getMessage());
        } catch (Exception e) {
            System.out.println("Exception caught in setPercentTolerance: " + e.getMessage());
        }
    }

    /**
     * This method returns the percentage tolerance value for the PID controller.
     * @return the tolerance value as a percentage of the setpoint
     */
    public double getPercentageTolerance() {
        try {
            return tolerance / target * 100;
        } catch (NullPointerException e) {
            System.out.println("NullPointerException caught in getPercentageTolerance: " + e.getMessage());
            return 0.0;
        } catch (Exception e) {
            System.out.println("Exception caught in getPercentageTolerance: " + e.getMessage());
            return 0.0;
        }
    }

    /**
     * This method returns a boolean value indicating whether the error between the start and the
     * target positions is within the specified tolerance.
     * @return true if the error is within tolerance, false otherwise
     */
    public boolean onTarget(){
        try {
            double error = Math.abs(start - target);
            return error < tolerance;
        } catch (NullPointerException e) {
            System.out.println("NullPointerException caught in onTarget: " + e.getMessage());
            return false;
        } catch (Exception e) {
            System.out.println("Exception caught in onTarget: " + e.getMessage());
            return false;
        }
    }

    /**
     * This method returns the current error value.
     * @return the current error value
     */
    public double getCurrentError() {
        try {
            return totalError;
        } catch (NullPointerException e) {
            System.out.println("NullPointerException caught in getCurrentError: " + e.getMessage());
            return 0.0;
        } catch (Exception e) {
            System.out.println("Exception caught in getCurrentError: " + e.getMessage());
            return 0.0;
        }
    }

    /**
     * This method returns the average error of the PID controller.
     * @param currentPosition : (double) current position
     * @return avgError : (double) average error
     */
    public double getAvgError(double currentPosition) {
        try {
            totalError += getCurrentError();
            return totalError / elapsedTime.seconds();
        } catch (NullPointerException e) {
            telemetry.addData("Error:", "NullPointerException caught in getAvgError: " + e.getMessage());
            telemetry.update();
            return 0;
        } catch (Exception e) {
            telemetry.addData("Error:", "Exception caught in getAvgError: " + e.getMessage());
            telemetry.update();
            return 0;
        }
    }

    /**
     * This method calculates the PID output applying the PID equation to the given set point target
     * and current input value.
     *
     * @return PID output value.
     */
    public double calculate(){
        // Set new start position
        //this.start = current;

        // Send telemetry message
        //setTelemetry(telemetry);
        //telemetry.addData("PIDController - ", 0);
        //telemetry.update();

        // Calculating the error
        double currentError = target - start;

        // Check if PIDController is at its first run or not
        if (firstRun) {
            // Set previous error and integral to initial values on first run
            previousError = currentError;
            integral = 0;
            derivative = 0;
            firstRun = false;
        }

        // Proportional term
        double p = kP * currentError;

        double previousTime = currentTime;
        currentTime = elapsedTime.nanoseconds()/1000000000.0;
        deltaTime = currentTime - previousTime;

        // Integral term
        if (kI != 0){
            double potentialGain = (totalError + currentError * deltaTime) * kI;
            if (potentialGain >= maxOutput) {
                totalError = maxOutput / kI;
            }
            else if (potentialGain > minOutput)
            {
                totalError += currentError * deltaTime;
            }
            else
            {
                totalError = minOutput / kI;
            }
        }
        else {
            totalError = 0.0;
        }

        integral = totalError * deltaTime;
        double i = kI * integral;

        // Derivative term
        if (deltaTime != 0) {
            derivative = (currentError - previousError) / deltaTime;
        } else {
            derivative = 0;
        }
        double d = kD * derivative;

        // Save the current error and time as the previous values for the next iteration
        previousError = currentError;

        // Feed forward term (feed forward is the target)
        double f = kF * target;

        double output;
        // Check if currentError, i (integral) or d (derivative) is NaN or Infinite
        if (Double.isNaN(currentError) || Double.isNaN(i) || Double.isNaN(d)
                || Double.isInfinite(currentError) || Double.isInfinite(i) || Double.isInfinite(d)) {
            // Set the output to a default value
            output = 0;
        } else {
            // Calculate the output with integral and derivative
            output = p + i + d + f;
        }

        // Check if output is in the desired range or adjust the output value to be in the desired
        // range or throw an exception
        try {
            // Check if output is outside of specified range
            if (output < minOutput || output > maxOutput) {
                adjustOutput(output);
                //throw new IllegalArgumentException("PID controller output is outside of specified range");
            }
        } catch (IllegalArgumentException e) {
            System.out.println("Message of Exception : "  + e.getMessage());
        }

        // Checking if the error is within the tolerance
        if (Math.abs(currentError) < tolerance) {
            integral = 0;
            derivative = 0;
        }

        return output;
    }

    /**
     * This method resets the PID controller.
     */
    public void reset() {
        totalError = 0;
        integral = 0;
        derivative = 0;
        previousError = 0;
        elapsedTime.reset();
        firstRun = true;
    }

    /**
     * This method sends an error message to the telemetry.
     *
     * @param message the error message to send.
     */
    private void sendError(String message) {
        telemetry.addData("Error:", message);
        telemetry.update();
    }
}