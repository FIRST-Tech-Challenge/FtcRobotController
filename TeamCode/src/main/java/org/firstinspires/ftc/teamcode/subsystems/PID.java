package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Created by ericw on 10/21/2017.
 */

public abstract class PID {
    private double minPowOut = 0;
    double kP;
    double kI;
    double kD;
    double setPoint;
    double curIError;
    double curDError;
    double error;
    double lastError;
    double tolerance;
    long sleepTime;
    long prevTime;
    Thread T;
    boolean active;
    boolean invertOutput = false;
    double resetvalue = 0;


    /**********************************
     * Constructor - Set constants and sleep timer
     * @param kP The Proportional constant, reacts linearly to error
     * @param kI The Integral constant, calculates cumulative error of each cycle
     * @param kD The Derivative constant, calculates the change in error of each cycle
     * @param sleepT How quickly the input will update the error, try to match with sensor poling
     */
    public PID(double kP, double kI, double kD, long sleepT){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.sleepTime = sleepT;

        T = new Thread(new Runnable() {
            @Override
            public void run() {
                while (active){
                    error();
                    try {Thread.sleep(sleepTime);
                    } catch (InterruptedException e){}
                }
            }
        });

    }

    /**********************************
     * Constructor - Set constants and sleep timer
     * @param kP The Proportional constant, reacts linearly to error
     * @param kI The Integral constant, calculates cumulative error of each cycle
     * @param kD The Derivative constant, calculates the change in error of each cycle
     * @param sleepT How quickly the input will update the error, try to match with sensor poling
     * @param minPowOut Minimum output of controller
     */
    public PID(double kP, double kI, double kD, long sleepT, double minPowOut){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.sleepTime = sleepT;
        this.minPowOut = minPowOut;


        T = new Thread(new Runnable() {
            @Override
            public void run() {
                while (active){
                    error();
                    //handleOutput();
                    try {Thread.sleep(sleepTime);
                    } catch (InterruptedException e){}
                }
            }
        });

    }

    /********************************
     * Required to feed the error
     * @return Set as the variable which will be read. Be sure to make sure it will update.
     */
    protected abstract double getInput();

    /********************************
     * Required to be able to reset the value of sensor to zero when PID_greg tries to reset.
     */
    protected abstract void resetInput();

    /********************************
     * Required if you want to handle output while active.
     */
    protected abstract void handleOutput();

    /******
     * Used to give a stop signal to thing.
     */
    protected abstract void stop();

    /********************************
     * Start, Stop, and Reset methods.
     */
    public void Activate(){
        this.active = true;
        T.start();
    }

    public void Deactivate(){
        this.active = false;
    }

    public void Reset(){
        curIError = 0;
        curDError = 0;
        lastError = 0;
        resetInput();
        if (T.isAlive()) error();
    }

    /**************************************
     * Setters
     */
    public void setkP(double kP) {
        this.kP = kP;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    public void setkPID(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;}

    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
        this.error();
    }

    public void setInvertOutput(boolean invertOutput) {
        this.invertOutput = invertOutput;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    /**************************************
     * Getters
     */
    public double getOutput() {
        double output = (invertOutput ? -1:1)*(kP*error + kI* curIError + kD*(error- lastError));

        if (isInTolerance()){
            return 0;
        } else if (Math.abs(output) > this.minPowOut){
            return output;
        }
        else return Math.signum(output)*this.minPowOut;
    }

    public boolean isActive() {
        return active;
    }

    public boolean isInTolerance(){
        return Math.abs(getError()) < tolerance;
    }

    public double getError() {
        return error;
    }

    public double getSensorReading(){ return getInput();}


    /**************************************
     * Error Calculation - Don't do extraneous calculations if required.
     */


    private void error(){
        error = setPoint - getInput();
        if (kI != 0) calcIError();
        if (kD != 0) curDError = error - this.lastError;
        this.lastError = error;

    }

    private void calcIError(){
        long CurrentTime = System.currentTimeMillis();
        curIError += error * (double)(CurrentTime- prevTime);
        prevTime = CurrentTime;
    }

}
