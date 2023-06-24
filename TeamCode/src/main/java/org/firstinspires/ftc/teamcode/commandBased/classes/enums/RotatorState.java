package org.firstinspires.ftc.teamcode.commandBased.classes.enums;

import org.firstinspires.ftc.teamcode.commandBased.Constants;

public class RotatorState {
    public enum Pos {
        FRONT(Constants.ROTATOR_FRONT),
        BACK(Constants.ROTATOR_BACK);

        private final double value;

        Pos(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }
}
