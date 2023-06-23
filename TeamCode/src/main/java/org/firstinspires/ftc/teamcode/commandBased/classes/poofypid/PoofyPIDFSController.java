package org.firstinspires.ftc.teamcode.commandBased.classes.poofypid;

public class PoofyPIDFSController extends PoofyPIDController {

    private final PoofyPIDSController pids;

    private double kF;

    public PoofyPIDFSController(double kp, double ki, double kd, double kF, double kS) {
        super(kp, ki, kd);
        this.kF = kF;
        pids = new PoofyPIDSController(0, 0, 0, kS);
    }

    @Override
    public double calculate(double measuredPosition) {
        double output = super.calculate(measuredPosition);
        double kStatic = pids.calculatekS(output);
        output += kStatic + kF;
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
