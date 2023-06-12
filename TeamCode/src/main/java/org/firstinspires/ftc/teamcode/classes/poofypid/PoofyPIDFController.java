package org.firstinspires.ftc.teamcode.classes.poofypid;

public class PoofyPIDFController extends PoofyPIDController{

    private double kF;

    public PoofyPIDFController(double kP, double kI, double kD, double kF) {
        super(kP, kI, kD);
        this.kF = kF;
    }

    @Override
    public double calculate(double measuredPosition) {
        double output = super.calculate(measuredPosition) + kF;
        return output;
    }

    public void setF(double kF) {
        this.kF = kF;
    }

    public void setPIDF(double kP, double kI, double kD, double kF) {
        super.setPID(kP, kI, kD);
        this.kF = kF;
    }
}
