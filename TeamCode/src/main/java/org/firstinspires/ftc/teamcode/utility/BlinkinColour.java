package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public enum BlinkinColour {
    NO_INFORMATION(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE),
    BLUE_ALLIANCE(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE),
    RED_ALLIANCE(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE),
    ALLIANCE_SAMPLE(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE),
    NEUTRAL_SAMPLE(RevBlinkinLedDriver.BlinkinPattern.YELLOW),
    PIECE_GRABBED(RevBlinkinLedDriver.BlinkinPattern.LIME),
    APRIL_TAG_DETECTED(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);

    private final RevBlinkinLedDriver.BlinkinPattern pattern;

    BlinkinColour(RevBlinkinLedDriver.BlinkinPattern pattern) {
        this.pattern = pattern;
    }

    public RevBlinkinLedDriver.BlinkinPattern getPattern() {
        return pattern;
    }
}
