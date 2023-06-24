package org.firstinspires.ftc.teamcode.commandBased.classes.enums;

import org.firstinspires.ftc.teamcode.commandBased.Constants;

public class IntakeState {
    public enum Pos {
        INTAKING(1),
        IDLE(0),
        OUTTAKING(-1);

        private final double value;

        Pos(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }
}
