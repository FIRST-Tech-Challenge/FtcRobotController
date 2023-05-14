package org.firstinspires.ftc.teamcode.classes.pid;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FeedbackController;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDOpenClosed {

    FeedbackController controller = null;
    private double deadzone = 0;

    private boolean comingToRest = false;

    private enum InputChange{INCREASING, DECREASING}
    private InputChange inputChange = null;

    private double pastState = 0;
    private double target = 0;

    public PIDOpenClosed(FeedbackController controller, double deadzone) {
        this.controller = controller;
        this.deadzone = deadzone;
    }

    public double calculate(double input, double state) {
        double output = 0;
        if (Math.abs(input) > deadzone) {
            output = input;
            pastState = state;
            target = state;
            comingToRest = true;
            if (input < 0) {
                inputChange = InputChange.DECREASING;
            } else if (input > 0) {
                inputChange = InputChange.INCREASING;
            }
        } else if (comingToRest) {
            if ((inputChange == InputChange.INCREASING && pastState < state) ||
                    (inputChange == InputChange.DECREASING && pastState > state)) {
                    target = state;
                    comingToRest = false;
            }
            pastState = state;
            output = -controller.calculate(target, state);
        } else {
            output = -controller.calculate(target, state);
        }
        return output;
    }

//    public double timeCalculate(double input, double state, double idle) {
//        double output = 0;
//        ElapsedTime timer = new ElapsedTime();
//        if (Math.abs(input) > deadzone) {
//            output = input;
//            pastState = state;
//            target = state;
//            comingToRest = true;
//            timer.reset();
//        } else if (comingToRest) {
//            if (timer.seconds() > idle) {
//                target = state;
//                comingToRest = false;
//            }
//            output = -controller.calculate(target, state);
//        } else {
//            output = -controller.calculate(target, state);
//        }
//        return output;
//    }

    public double getTarget() {
        return target;
    }

    public boolean isComingToRest() {
        return comingToRest;
    }
}
