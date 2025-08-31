package org.firstinspires.ftc.teamcode.aim.drive;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

class PIDControl {
    double  gain;
    double  ki;
    double  kd;
    double  accelLimit;
    double  defaultOutputLimit;
    double  liveOutputLimit;
    double  setPoint;
    double  tolerance;
    double deadband;
    boolean circular;
    boolean inPosition;
    double integral;
    double minNoneZeroOutput;
    
    double lastOutput;
    double lastError;
    double lastInput;
    double lastDeltaTime;

    ElapsedTime cycleTime = new ElapsedTime();

    public PIDControl(double kp, double ki,  double accelLimit, double outputLimit, double tolerance, double deadband, boolean circular) {
        this.gain = kp;
        this.ki = ki;
        this.accelLimit = accelLimit;
        this.defaultOutputLimit = outputLimit;
        this.liveOutputLimit = outputLimit;
        this.tolerance = tolerance;
        this.deadband = deadband;
        this.circular = circular;
        this.minNoneZeroOutput = 0;
        this.kd = 0;
        reset(0.0);
    }

    public PIDControl(PIDControlParams params) {
        this.gain = params.gain;
        this.ki = params.ki;
        this.accelLimit = params.accelLimit;
        this.defaultOutputLimit = params.outputLimit;
        this.liveOutputLimit = params.outputLimit;
        this.tolerance = params.tolerance;
        this.deadband = params.deadband;
        this.circular = params.circular;
        this.minNoneZeroOutput = params.minNonZeroOutput;
        this.kd = 0;
        reset(0.0);
    }

     private boolean validDouble(double v) {
        if (Double.isNaN(v) || !Double.isFinite(v)) {
            return false;
        }
        return true;
    }
    
    /**
     * Determines power required to obtain the desired setpoint value based on new input value.
     * Uses proportional gain, and limits rate of change of output, as well as max output.
     * @param input  Current live control input value (from sensors)
     * @return desired output power.
     */
    public double getOutput(double input) {
        this.lastInput = input;
        double error = setPoint - input;

        double deltaTime = cycleTime.seconds();
        if (!validDouble(deltaTime) || deltaTime <= 0.01) {
            deltaTime = 0.01;
        }
        this.lastDeltaTime = deltaTime;
        
        double dV = deltaTime * accelLimit;
        double output;

        // normalize to +/- 180 if we are controlling heading
        if (circular) {
            while (error > 180)  error -= 360;
            while (error <= -180) error += 360;
        }
        
        double tmpIntegral = error * deltaTime ;
        if (!validDouble(tmpIntegral)) {
            tmpIntegral = 0;
        }
        double total = this.integral + tmpIntegral;
        if (validDouble(total)) {
            this.integral = total;
        }
        
        this.integral += tmpIntegral ;
        if (!validDouble(this.integral)) {
            this.integral = 0;
        }
        
        // disable this as deltaTime might be 0
        double derivative = 0; //(error - lastError) / deltaTime;

        inPosition = (Math.abs(error) < tolerance);

        // Prevent any very slow motor output accumulation
        if (Math.abs(error) <= deadband) {
            output = 0;
        } else {
            // calculate output power using gain and clip it to the limits
            output = (error * gain + ki * this.integral);// + kd * derivative);
            output = Range.clip(output, -liveOutputLimit, liveOutputLimit);

            // Now limit rate of change of output (acceleration)
            if ((output - lastOutput) > dV) {
                output = lastOutput + dV;
            } else if ((output - lastOutput) < -dV) {
                output = lastOutput - dV;
            }
        }

        /*if (Double.isNaN(output)) {
            output = lastOutput;
        } else if (Math.abs(output) > 0 && Math.abs(output) < this.minNoneZeroOutput) {
            output = minNoneZeroOutput * Math.signum(output);
        }*/
        
         if (Double.isNaN(output)) {
            // do nothing
        } else if (Math.abs(output) > 0 && Math.abs(output) < this.minNoneZeroOutput) {
            if (output > 0) {
                output = minNoneZeroOutput;
            } else {
                output = - minNoneZeroOutput;
            }
        }
        
        lastError = error;
        lastOutput = output;
        cycleTime.reset();
        return output;
    }

    public String toString() {
        return String.format("Input: %f Time: %f Inte: %f Err: %f Out: %f",
                this.lastInput, this.lastDeltaTime, this.integral, this.lastError,
                this.lastOutput);
    }


    public boolean inPosition(){
        return inPosition;
    }
    public double getSetpoint() {
        return setPoint;
    }

    public double getTolerance() {
        return this.tolerance;
    }
    public void setTolerance( double tolerance) {
        this.tolerance = tolerance;
    }

    /**
     * Saves a new setpoint and resets the output power history.
     * This call allows a temporary power limit to be set to override the default.
     * @param setPoint
     * @param powerLimit
     */
    public void reset(double setPoint, double powerLimit) {
        liveOutputLimit = Math.abs(powerLimit);
        this.setPoint = setPoint;
        reset();
    }

    /**
     * Saves a new setpoint and resets the output power history.
     * @param setPoint
     */
    public void reset(double setPoint) {
        liveOutputLimit = defaultOutputLimit;
        this.setPoint = setPoint;
        reset();
    }

    /**
     * Leave everything else the same, Just restart the acceleration timer and set output to 0
     */
    public void reset() {
        cycleTime.reset();
        inPosition = false;
        lastOutput = 0.0;
        integral = 0.0;
        lastError = setPoint;
    }
}