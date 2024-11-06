package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

public interface Extensor {

    // Enum for named positions
    enum Position {
        PICKUP(0.0),
        HOVER(0.15),
        LOW_RUNG(0.66),
        HIGH_RUNG(1.0);

        private final double value;

        Position(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }

    void setPosition(Position position);

    void extend();

    void retract();

    void ground();
}