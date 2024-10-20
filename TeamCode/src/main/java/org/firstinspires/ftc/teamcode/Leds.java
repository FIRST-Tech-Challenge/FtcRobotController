package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class Leds {
    RevBlinkinLedDriver led;

    // private int cur = 0;
    final static RevBlinkinLedDriver.BlinkinPattern[] patterns = {
            RevBlinkinLedDriver.BlinkinPattern.BLUE,                        // 0
            RevBlinkinLedDriver.BlinkinPattern.RED,                         // 1
            RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE,       // 2
            RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE,     // 3
            RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE,    // 4
            RevBlinkinLedDriver.BlinkinPattern.CONFETTI,                    // 5
            RevBlinkinLedDriver.BlinkinPattern.WHITE,                       // 6
            RevBlinkinLedDriver.BlinkinPattern.GREEN,                       // 7
            RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE, //8
            RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE, //9
            RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE, //10
            RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE, //11
            RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE, //12
            RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE, //13
            RevBlinkinLedDriver.BlinkinPattern.CP1_END_TO_END_BLEND_TO_BLACK, //14
            RevBlinkinLedDriver.BlinkinPattern.HOT_PINK, //15
            RevBlinkinLedDriver.BlinkinPattern.CP1_2_SPARKLE_1_ON_2, //16
            RevBlinkinLedDriver.BlinkinPattern.CP1_2_SPARKLE_2_ON_1, //17
            RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT, //18
            RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE, //19
            RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY, //20
            RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED, //21
            RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE, //22
            RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES //23
    };

    public Leds(Robot robot) {
        led = robot.led;
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    void setPattern(int index) {
        led.setPattern(patterns[index]);
    }
}
