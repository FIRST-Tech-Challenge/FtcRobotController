package org.firstinspires.ftc.teamcode.commandBased.classes.enums;

import org.firstinspires.ftc.teamcode.commandBased.Constants;

public class Stack {
    public enum Cone {
        FIRST(Constants.ELE_LOW),
        SECOND(Constants.ELE_LOW + Constants.STACK_INCREMENT * 1),
        THIRD(Constants.ELE_LOW + Constants.STACK_INCREMENT * 2),
        FOURTH(Constants.ELE_LOW + Constants.STACK_INCREMENT * 3),
        FIFTH(Constants.ELE_LOW + Constants.STACK_INCREMENT * 4);

        private final double value;

        Cone(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }
}
