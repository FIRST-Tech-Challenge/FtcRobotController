package org.firstinspires.ftc.teamcode.ebotsenums;

import static java.lang.String.format;

public enum Accuracy{
    LOOSE (5.0, 3.0, 5.0, 2.50),
    STANDARD (2.0, 1.0, 3.0, 1.0),
    TIGHT (1.0, 0.50, 1.5, 0.5);

    /**  ENUM VARIABLES     **************/

    private double headingAccuracyDeg;
    private double positionalAccuracy;
    private double integratorUnwindLimit;
    private double spinIntegratorUnwindLimit;

    /**  CONSTRUCTOR    **************/

    Accuracy(double headingAccuracyDeg, double positionalAccuracy, double integratorUnwind, double spinIntegratorUnwind){
        this.headingAccuracyDeg = headingAccuracyDeg;
        this.positionalAccuracy = positionalAccuracy;
        this.integratorUnwindLimit = integratorUnwind;  // for translate position
        this.spinIntegratorUnwindLimit = spinIntegratorUnwind;  // for translate position
    }

    /**  ENUM GETTERS AND SETTERS  ***********/

    public double getHeadingAccuracyDeg() {
        return headingAccuracyDeg;
    }
    public double getPositionalAccuracy() {
        return positionalAccuracy;
    }
    public double getIntegratorUnwindLimit() {
        return integratorUnwindLimit;
    }
    public double getSpinIntegratorUnwindLimit() {
        return spinIntegratorUnwindLimit;
    }

    public double getTargetIntegratorUnwind(CsysDirection dir){
        double targetIntegratorUnwind = integratorUnwindLimit;  // for all but heading
        if (dir == CsysDirection.Heading){
            targetIntegratorUnwind = spinIntegratorUnwindLimit;
        }
        return targetIntegratorUnwind;
    }
    @Override
    public String toString(){
        return "Translate position tol: " + format("%.1f",positionalAccuracy)
                + ", Translate Integrator: " + format("%.1f", integratorUnwindLimit)
                + ", Heading angle tol: " + format("%.1f", headingAccuracyDeg)
                + ", Heading Integrator: " + format("%.1f", spinIntegratorUnwindLimit);
    }
}
