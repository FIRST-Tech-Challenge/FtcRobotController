package org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LED {
    Servo servo;

    LED(HardwareMap hardwareMap, String LED_Name) {
        servo = hardwareMap.servo.get(LED_Name);

    }

    private double getPosFromEnum(All_Colors color) {
        switch (color) {
            case RAINBOW_RAINBOW_PALETTE:
                return -0.99;
            case RAINBOW_PARTY_PALETTE:
                return -0.97;
            case RAINBOW_OCEAN_PALETTE:
                return -0.95;
            case RAINBOW_LAVE_PALETTE:
                return -0.93;
            case RAINBOW_FOREST_PALETTE:
                return -0.91;
            case RAINBOW_WITH_GLITTER:
                return -0.89;
            case CONFETTI:
                return -0.87;
            case SHOT_RED:
                return -0.85;
            case SHOT_BLUE:
                return -0.83;
            case SHOT_WHITE:
                return -0.81;
            case SINELON_RAINBOW_PALETTE:
                return -0.79;
            case SINELON_PARTY_PALETTE:
                return -0.77;
            case SINELON_OCEAN_PALETTE:
                return -0.75;
            case SINELON_LAVA_PALETTE:
                return -0.73;
            case SINELON_FOREST_PALETTE:
                return -0.71;
            case BEATS_PER_MINUTE_RAINBOW_PALETTE:
                return -0.69;
            case BEATS_PER_MINUTE_PARTY_PALETTE:
                return -0.67;
            case BEATS_PER_MINUTE_OCEAN_PALETTE:
                return -0.65;
            case BEATS_PER_MINUTE_LAVA_PALETTE:
                return -0.63;
            case BEATS_PER_MINUTE_FOREST_PALETTE:
                return -0.61;
            case FIRE_MEDIUM:
                return -0.59;
            case FIRE_LARGE:
                return -0.57;
            case TWINKLES_RAINBOW_PALETTE:
                return -0.55;
            case TWINKLES_PARTY_PALETTE:
                return -0.53;
            case TWINKLES_OCEAN_PALETTE:
                return -0.51;
            case TWINKLES_LAVA_PALETTE:
                return -0.49;
            case TWINKLES_FOREST_PALETTE:
                return -0.47;
            case COLOR_WAVES_RAINBOW_PALETTE:
                return -0.45;
            case COLOR_WAVES_PARTY_PALETTE:
                return -0.43;
            case COLOR_WAVES_OCEAN_PALETTE:
                return -0.41;
            case COLOR_WAVES_LAVA_PALETTE:
                return -0.39;
            case COLOR_WAVES_FOREST_PALETTE:
                return -0.37;
            case LARSON_SCANNER_RED_PATTERN:
                return -0.35;
            case LARSON_SCANNER_GRAY_PATTERN:
                return -0.33;
            case LIGHT_CHASE_RED_DIMMING_SPEED_BRIGHTNESS:
                return -0.31;
            case LIGHT_CHASE_BLUE_DIMMING_SPEED_BRIGHTNESS:
                return -0.29;
            case LIGHT_CHASE_GRAY_DIMMING_SPEED_BRIGHTNESS:
                return -0.27;
            case HEARTBEAT_RED:
                return -0.25;
            case HEARTBEAT_BLUE:
                return -0.23;
            case HEARTBEAT_WHITE:
                return -0.21;
            case HEARTBEAT_GRAY:
                return -0.19;
            case BREATH_RED:
                return -0.17;
            case BREATH_BLUE:
                return -0.15;
            case BREATH_GRAY:
                return -0.13;
            case STROBE_RED:
                return -0.11;
            case STROBE_BLUE:
                return -0.09;
            case STROBE_GOLD:
                return -0.07;
            case STROBE_WHITE:
                return -0.05;
            case END_TO_END_BLEND_TO_BLACK:
                return -0.03;
            case LARSON_SCANNER_PATTERN:
                return -0.01;
            case LIGHT_CHASE_DIMMING_SPEED_BRIGHTNESS:
                return 0.01;
            case HEARTBEAT_SLOW:
                return 0.03;
            case HEARTBEAT_MEDIUM:
                return 0.05;
            case HEARTBEAT_FAST:
                return 0.07;
            case BREATH_SLOW:
                return 0.09;
            case BREATH_FAST:
                return 0.11;
            case SHOT:
                return 0.13;
            case STROBE:
                return 0.15;
            case END_TO_END_BLEND_TO_BLACK2:
                return 0.17;
            case LARSON_SCANNER_PATTERN2:
                return 0.19;
            case LIGHT_CHASE_DIMMING_SPEED_BRIGHTNESS2:
                return 0.21;
            case HEARTBEAT_SLOW2:
                return 0.23;
            case HEARTBEAT_MEDIUM2:
                return 0.25;
            case HEARTBEAT_FAST2:
                return 0.27;
            case BREATH_SLOW2:
                return 0.29;
            case BREATH_FAST2:
                return 0.31;
            case SHOT2:
                return 0.33;
            case STROBE2:
                return 0.35;
            case SPARKLE_COLOR_1_ON_COLOR_2:
                return 0.37;
            case SPARKLE_COLOR_2_ON_COLOR_1:
                return 0.39;
            case COLOR_GRADIENT_COLOR_1_AND_2:
                return 0.41;
            case BEATS_PER_MINUTE:
                return 0.43;
            case END_TO_END_BLEND_COLOR_1_TO_2:
                return 0.45;
            case END_TO_END_BLEND:
                return 0.47;
            case COLOR_1_AND_COLOR_2_NO_BLENDING:
                return 0.49;
            case TWINKLES_COLOR_1_AND_2:
                return 0.51;
            case COLOR_WAVES_COLOR_1_AND_2:
                return 0.53;
            case SINELON:
                return 0.55;
            case HOT_PINK:
                return 0.57;
            case DARK_RED:
                return 0.59;
            case RED:
                return 0.61;
            case RED_ORANGE:
                return 0.63;
            case ORANGE:
                return 0.65;
            case GOLD:
                return 0.67;
            case YELLOW:
                return 0.69;
            case LAWN_GREEN:
                return 0.71;
            case LIME:
                return 0.73;
            case DARK_GREEN:
                return 0.75;
            case GREEN:
                return 0.77;
            case BLUE_GREEN:
                return 0.79;
            case AQUA:
                return 0.81;
            case SKY_BLUE:
                return 0.83;
            case DARK_BLUE:
                return 0.85;
            case BLUE:
                return 0.87;
            case BLUE_VIOLET:
                return 0.89;
            case VIOLET:
                return 0.91;
            case WHITE:
                return 0.93;
            case GRAY:
                return 0.95;
            case DARK_GRAY:
                return 0.97;
            case BLACK:
                return 0.99;
        }

        return 0;
    }

    private double getPosFromEnum(Solid_Color color) {
        switch (color) {
            case Hot_Pink:
                return .57;
            case Dark_Red:
                return .59;
            case Red:
                return .61;
            case Red_Orange:
                return .63;
            case Orange:
                return .65;
            case Gold:
                return .67;
            case Yellow:
                return .69;
            case Lawn_Green:
                return .71;
            case Lime:
                return .73;
            case Dark_Green:
                return .75;
            case Green:
                return .77;
            case Blue_Green:
                return .79;
            case Aqua:
                return .81;
            case Sky_Blue:
                return .83;
            case Dark_Blue:
                return .85;
            case Blue:
                return .87;
            case Blue_Violet:
                return .89;
            case Violet:
                return .91;
            case White:
                return .93;
            case Gray:
                return .95;
            case Dark_Gray:
                return .97;
            case Black:
                return .99;
        }
        return 0;
    }

    public void setColor(Solid_Color color) {
        servo.setPosition(getPosFromEnum(color));
    }

    public void setColor(All_Colors color) {
        servo.setPosition(getPosFromEnum(color));
    }

    public enum All_Colors {
        RAINBOW_RAINBOW_PALETTE,
        RAINBOW_PARTY_PALETTE,
        RAINBOW_OCEAN_PALETTE,
        RAINBOW_LAVE_PALETTE,
        RAINBOW_FOREST_PALETTE,
        RAINBOW_WITH_GLITTER,
        CONFETTI,
        SHOT_RED,
        SHOT_BLUE,
        SHOT_WHITE,
        SINELON_RAINBOW_PALETTE,
        SINELON_PARTY_PALETTE,
        SINELON_OCEAN_PALETTE,
        SINELON_LAVA_PALETTE,
        SINELON_FOREST_PALETTE,
        BEATS_PER_MINUTE_RAINBOW_PALETTE,
        BEATS_PER_MINUTE_PARTY_PALETTE,
        BEATS_PER_MINUTE_OCEAN_PALETTE,
        BEATS_PER_MINUTE_LAVA_PALETTE,
        BEATS_PER_MINUTE_FOREST_PALETTE,
        FIRE_MEDIUM,
        FIRE_LARGE,
        TWINKLES_RAINBOW_PALETTE,
        TWINKLES_PARTY_PALETTE,
        TWINKLES_OCEAN_PALETTE,
        TWINKLES_LAVA_PALETTE,
        TWINKLES_FOREST_PALETTE,
        COLOR_WAVES_RAINBOW_PALETTE,
        COLOR_WAVES_PARTY_PALETTE,
        COLOR_WAVES_OCEAN_PALETTE,
        COLOR_WAVES_LAVA_PALETTE,
        COLOR_WAVES_FOREST_PALETTE,
        LARSON_SCANNER_RED_PATTERN,
        LARSON_SCANNER_GRAY_PATTERN,
        LIGHT_CHASE_RED_DIMMING_SPEED_BRIGHTNESS,
        LIGHT_CHASE_BLUE_DIMMING_SPEED_BRIGHTNESS,
        LIGHT_CHASE_GRAY_DIMMING_SPEED_BRIGHTNESS,
        HEARTBEAT_RED,
        HEARTBEAT_BLUE,
        HEARTBEAT_WHITE,
        HEARTBEAT_GRAY,
        BREATH_RED,
        BREATH_BLUE,
        BREATH_GRAY,
        STROBE_RED,
        STROBE_BLUE,
        STROBE_GOLD,
        STROBE_WHITE,
        END_TO_END_BLEND_TO_BLACK,
        LARSON_SCANNER_PATTERN,
        LIGHT_CHASE_DIMMING_SPEED_BRIGHTNESS,
        HEARTBEAT_SLOW,
        HEARTBEAT_MEDIUM,
        HEARTBEAT_FAST,
        BREATH_SLOW,
        BREATH_FAST,
        SHOT,
        STROBE,
        END_TO_END_BLEND_TO_BLACK2,
        LARSON_SCANNER_PATTERN2,
        LIGHT_CHASE_DIMMING_SPEED_BRIGHTNESS2,
        HEARTBEAT_SLOW2,
        HEARTBEAT_MEDIUM2,
        HEARTBEAT_FAST2,
        BREATH_SLOW2,
        BREATH_FAST2,
        SHOT2,
        STROBE2,
        SPARKLE_COLOR_1_ON_COLOR_2,
        SPARKLE_COLOR_2_ON_COLOR_1,
        COLOR_GRADIENT_COLOR_1_AND_2,
        BEATS_PER_MINUTE,
        END_TO_END_BLEND_COLOR_1_TO_2,
        END_TO_END_BLEND,
        COLOR_1_AND_COLOR_2_NO_BLENDING,
        TWINKLES_COLOR_1_AND_2,
        COLOR_WAVES_COLOR_1_AND_2,
        SINELON,
        HOT_PINK,
        DARK_RED,
        RED,
        RED_ORANGE,
        ORANGE,
        GOLD,
        YELLOW,
        LAWN_GREEN,
        LIME,
        DARK_GREEN,
        GREEN,
        BLUE_GREEN,
        AQUA,
        SKY_BLUE,
        DARK_BLUE,
        BLUE,
        BLUE_VIOLET,
        VIOLET,
        WHITE,
        GRAY,
        DARK_GRAY,
        BLACK
    }

    public enum Solid_Color {
        Hot_Pink,
        Dark_Red,
        Red,
        Red_Orange,
        Orange,
        Gold,
        Yellow,
        Lawn_Green,
        Lime,
        Dark_Green,
        Green,
        Blue_Green,
        Aqua,
        Sky_Blue,
        Dark_Blue,
        Blue,
        Blue_Violet,
        Violet,
        White,
        Gray,
        Dark_Gray,
        Black
    }
}
