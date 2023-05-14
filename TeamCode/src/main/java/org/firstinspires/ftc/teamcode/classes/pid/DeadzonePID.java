package org.firstinspires.ftc.teamcode.classes.pid;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;

public class DeadzonePID extends PIDEx {

    private double deadzone;

    public DeadzonePID(PIDCoefficientsEx coefficients, double deadzone) {
        super(coefficients);
        this.deadzone = deadzone;
    }

    @Override
    public double calculate(double reference, double state) {
        if (reference < state + deadzone && reference > state - deadzone) {
            return 0;
        } else {
            return super.calculate(reference, state);
        }
    }
}
