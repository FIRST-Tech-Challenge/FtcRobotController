package org.firstinspires.ftc.teamcode.mechanisms.submechanisms;

import org.firstinspires.ftc.teamcode.Settings;

public interface ExtensorV2 {

    // Enum for named positions
    enum HorizontalPosition {
        COLLAPSED(Settings.Hardware.HorizontalExtensor.COLLAPSED),
        LEVEL_1(Settings.Hardware.HorizontalExtensor.LEVEL_1),
        LEVEL_2(Settings.Hardware.HorizontalExtensor.LEVEL_2),
        EXPANDED(Settings.Hardware.HorizontalExtensor.EXPANDED);

        private final int value;

        HorizontalPosition(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    enum VerticalPosition {
        PICKUP(Settings.Hardware.VerticalExtensor.PICKUP),
        HOVER(Settings.Hardware.VerticalExtensor.HOVER),
        LOW_RUNG(Settings.Hardware.VerticalExtensor.LOW_RUNG),
        HIGH_RUNG(Settings.Hardware.VerticalExtensor.HIGH_RUNG);

        private final int value;

        VerticalPosition(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    void setPosition(HorizontalPosition horizontalPosition, VerticalPosition verticalPosition);

    void extend();

    void retract();

    void ground();

    void ceiling();
}