package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

public interface Extensor {

    // Enum for named positions
    enum Position {
        GROUND(0.0),
        HOVER(0.15),
        LEVEL0(0.3),
        LEVEL1(0.66),
        LEVEL2(1.0);

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