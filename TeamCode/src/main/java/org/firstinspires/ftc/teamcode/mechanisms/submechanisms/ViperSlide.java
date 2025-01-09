package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

import org.firstinspires.ftc.teamcode.Settings;

public interface ViperSlide {

    // Enum for named positions
    enum HorizontalPosition {
        COLLAPSED(Settings.Hardware.HorizontalSlide.COLLAPSED),
        LEVEL_1(Settings.Hardware.HorizontalSlide.LEVEL_1),
        LEVEL_2(Settings.Hardware.HorizontalSlide.LEVEL_2),
        EXPANDED(Settings.Hardware.HorizontalSlide.EXPANDED);

        private final int value;

        HorizontalPosition(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    enum VerticalPosition {
        TRANSFER(Settings.Hardware.VerticalSlide.TRANSFER),
//        Not needed anymore as low rung can be reached at lowest position
//        LOW_RUNG(Settings.Hardware.VerticalSlide.LOW_RUNG),
        LOW_BASKET(Settings.Hardware.VerticalSlide.LOW_BASKET),
        HIGH_RUNG(Settings.Hardware.VerticalSlide.HIGH_RUNG),
        HIGH_BASKET(Settings.Hardware.VerticalSlide.HIGH_BASKET);

        private final int value;

        VerticalPosition(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    void setPosition(double position);

    void extend();

    void retract();

    void min();

    void max();

    void increment();
    void decrement();
}