package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import java.util.function.BooleanSupplier;

 public class Action {
        private final ActionFunction performAction; // A function that performs the action and returns true if the action is complete
        private final String msg;

        // Constructor that sets the action
        public Action(String msg, ActionFunction performAction) {
            this.performAction = performAction;
            this.msg = msg;
        }

        // Method to determine if the action is complete based on the BooleanSupplier
        public boolean actionComplete() {
            if (msg != null){
                telemetry.addLine(msg);
            }
            return performAction.evaluate();
        }
    }