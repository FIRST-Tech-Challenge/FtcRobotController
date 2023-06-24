package org.firstinspires.ftc.teamcode.commandBased.classes.enums;

import org.firstinspires.ftc.teamcode.commandBased.Constants;

public class ElevatorState {
    public enum Pos {
        LOW(Constants.ELE_LOW),
        IDLE(Constants.ELE_IDLE),
        MID(Constants.ELE_MID),
        HIGH(Constants.ELE_HIGH);

        private final double value;

        Pos(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }
}
