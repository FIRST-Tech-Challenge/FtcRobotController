package org.firstinspires.ftc.teamcode.classes;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FeedbackController;

public class PIDOpenClosed {

    FeedbackController controller = null;
    double deadzone = 0;

    boolean comingToRest = false;

    enum InputChange{INCREASING, DECREASING}
    InputChange inputChange = null;

    double pastInput = 0;
    double output = 0;
    double target = 0;

    public PIDOpenClosed(FeedbackController controller, double deadzone) {
        this.controller = controller;
        this.deadzone = deadzone;
    }

    public double calculate(double input, double state) {
        if (Math.abs(input) > deadzone) {
            output = input;
            pastInput = state;
            target = state;
            comingToRest = true;
            if (input < 0) {
                inputChange = InputChange.DECREASING;
            } else if (input > 0) {
                inputChange = InputChange.INCREASING;
            }
        } else if (comingToRest) {
            if (inputChange == InputChange.INCREASING && pastInput > state) {
                    target = state;
                    comingToRest = false;
            } else if (inputChange == InputChange.DECREASING && pastInput < state) {
                    target = state;
                    comingToRest = false;
            }
            pastInput = state;
            output = -controller.calculate(target, state);
        } else {
            output = -controller.calculate(target, state);
        }
        return output;
    }

    public double getTarget() {
        return target;
    }

}
