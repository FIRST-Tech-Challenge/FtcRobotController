package org.firstinspires.ftc.teamcode.Util;

/**
 * This is a PDFL class intended for use in Project Overload. The PDFL controller handles
 * the running of a PDF Controller, with an added lower limit, L, to prevent undercorrection
 * due to friction inherent in systems.
 *
 * @ author Asher Childress - 9161 Overlaod
 */

public class PDFLController {

    private double p,d,f,l;

    private double target, error, oldError, dError;

    private long time, oldTime;

    public double dir;

    private boolean atTarget = false;

    /**
     * Constructor for PDFLController.
     * @param p The proportional gain
     * @param d The derivative gain
     * @param f The feedforward gain
     * @param l The lower limit of the controller
     */
    public PDFLController(double p,double d,double f,double l) {
        this.p = p;
        this.d = d;
        this.f = f;
        this.l = l;
    }

    /**
     * Runs the PDFL controller using the given position and target.
     * @return The output of the controller
     */
    public double runPDFL(int errorMin) {
        double returnVal = (error*p) + (dError*d);
        dir = error > 0 ? 1 : error < 0 ? -1 : 0;



        if (Math.abs(error) <= errorMin) {
            atTarget = true;
            return f;
        }

        atTarget = false;
        return ((Math.max(Math.abs(l), Math.abs(returnVal))) * dir) + f;
    }

    /**
     * Updates the PDFL controller.
     * @param position The current position of the system
     */
    public void update(double position) {
        oldError = error;
        error = target-position;

        oldTime = time;
        time = System.nanoTime();

        dError = (error-oldError) /((time-oldTime)/ Math.pow(10.0,9));
    }

    public boolean atSetTarget(){
        return atTarget;
    }

    /**
     * Sets the target of the controller.
     * @param target The target
     */
    public void setTarget(double target) {this.target = target;}

    /**
     * Sets proportional gain of the controller.
     * @param p The proportional gain
     */
    public void setP(double p) {this.p = p;}

    /**
     * Sets derivative gain of the controller.
     * @param d The derivative gain
     */
    public void setD(double d) {this.d = d;}

    /**
     * Sets feedforward gain of the controller.
     * @param f The feedforward gain
     */
    public void setF(double f) {this.f = f;}

    /**
     * Sets lower limit of the controller.
     * @param l The lower limit
     */
    public void setL(double l) {this.l = l;}

    public void setPDFL(double p, double d, double f, double l){
        this.p = p;
        this.d = d;
        this.f = f;
        this.l = l;
    }

    /**
     * Gets the proportional gain of the controller.
     * @return The proportional gain
     */
    public double getP() {return p;}

    /**
     * Gets the derivative gain of the controller.
     * @return The derivative gain
     */
    public double getD() {return d;}

    /**
     * Gets the feedforward gain of the controller.
     * @return The feedforward gain
     */
    public double getF() {return f;}

    /**
     * Gets the lower limit of the controller.
     * @return The lower limit
     */
    public double getL() {return l;}
}
