package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

import org.firstinspires.ftc.teamcode.Settings;

public interface Extensor {

    // Enum for named positions
    enum Position {
        PICKUP(Settings.Hardware.Extensor.PICKUP),
        HOVER(Settings.Hardware.Extensor.HOVER),
        LOW_RUNG(Settings.Hardware.Extensor.LOW_RUNG),
        HIGH_RUNG(Settings.Hardware.Extensor.HIGH_RUNG);

        private final int value;

        Position(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    void setPosition(Position position);

    void extend();

    void retract();

    void ground();

    void ceiling();
}