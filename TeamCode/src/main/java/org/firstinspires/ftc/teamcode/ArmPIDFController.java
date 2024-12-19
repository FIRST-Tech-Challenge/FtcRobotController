package org.firstinspires.ftc.teamcode;

public class ArmPIDFController {
    private double Proportional;
    private double Integral;
    private double Derivative;
    private double Feedforward;
    private PIDController controller = new PIDController();
    public ArmPIDFController(double Proportional, double Integral, double Derivative, double Feedforward) {
        this.Proportional = Proportional;
        this.Integral = Integral;
        this.Derivative = Derivative;
        this.Feedforward = Feedforward;
        controller.init(this.Proportional, this.Integral, this.Derivative);
    }
    public void resetPIDF(double Proportional, double Integral, double Derivative, double Feedforward) {
        this.Proportional = Proportional;
        this.Integral = Integral;
        this.Derivative = Derivative;
        this.Feedforward = Feedforward;
        controller.resetPID(this.Proportional, this.Integral, this.Derivative);
    }
    public double getOutput(double state, double reference, double referenceInDegrees) {
        return controller.getOutput(state, reference) + Math.cos(referenceInDegrees) * Feedforward;
    }
}
