package org.firstinspires.ftc.teamcode.commandBased.classes.enums;

import org.firstinspires.ftc.teamcode.commandBased.Constants;

public class ArmState {
    public enum Pos {
        FRONT(Constants.ARM_ANGLE_FRONT),
        IDLE(Constants.ARM_ANGLE_IDLE),
        BACK(Constants.ARM_ANGLE_BACK);

        private final double value;

        Pos(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }
}
