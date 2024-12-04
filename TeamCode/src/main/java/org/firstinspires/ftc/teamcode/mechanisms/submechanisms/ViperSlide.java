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
        PICKUP(Settings.Hardware.VerticalSlide.PICKUP),
        HOVER(Settings.Hardware.VerticalSlide.HOVER),
        LOW_RUNG(Settings.Hardware.VerticalSlide.LOW_RUNG),
        HIGH_RUNG(Settings.Hardware.VerticalSlide.HIGH_RUNG);

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
}