package org.firstinspires.ftc.teamcode.classes.poofypid;

public class PoofyPIDSController extends PoofyPIDController{

    private double kS;

    public PoofyPIDSController(double kP, double kI, double kD, double kS) {
        super(kP, kI, kD);
        this.kS = kS;

    }

    @Override
    public double calculate(double measuredPosition) {
        double output = super.calculate(measuredPosition);
        double kStatic = calculatekS(output);
        output += kStatic;
        return output;
    }

    protected double calculatekS(double output) {
        if (output < 0) {
            return -kS;
        } else {
            return kS;
        }
    }

    public void setS(double kS) {
        this.kS = kS;
    }

    public void setPIDS(double kP, double kI, double kD, double kS) {
        super.setPID(kP, kI, kD);
        this.kS = kS;
    }
}
